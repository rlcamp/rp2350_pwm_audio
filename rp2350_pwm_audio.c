#include <math.h>
#include <complex.h>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#define PWM_PIN 3

#define IDMA_PWM 0

void yield(void) {
    /* we could do context switching here for cooperative multitasking if we wanted */
    __dsb();
    __wfe();
}

#define BUFFER_WRAP_BITS 12
#define BYTES_PER_CHUNK 2048

#define SAMPLES_PER_CHUNK (BYTES_PER_CHUNK / sizeof(uint16_t))

__attribute((aligned(sizeof(uint16_t) * 2 * SAMPLES_PER_CHUNK)))
static uint16_t buffer[2][SAMPLES_PER_CHUNK];
_Static_assert(1U << BUFFER_WRAP_BITS == sizeof(buffer), "wtf");

static float cmagsquaredf(const float complex x) {
    return crealf(x) * crealf(x) + cimagf(x) * cimagf(x);
}

static uint64_t xorshift64star(void) {
    /* marsaglia et al., yields 64 bits, most significant are most random */
    static uint64_t x = 1; /* must be nonzero */
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    return x * 0x2545F4914F6CDD1DULL;
}

static float frand_minus_frand(void) {
    /* generate 64 random bits, of which we will use the most significant 46, in two groups of 23 */
    const uint64_t bits = xorshift64star();

    /* generate two random numbers each uniformly distributed on [1.0f, 2.0f) */
    const union { uint32_t u; float f; } x = { .u = 0x3F800000U | ((bits >> 41) & 0x7FFFFFU) };
    const union { uint32_t u; float f; } y = { .u = 0x3F800000U | ((bits >> 18) & 0x7FFFFFU) };

    /* and subtract them, yielding a triangular distribution on (-1.0f, +1.0f) */
    return x.f - y.f;
}

int main() {
    /* enable sevonpend, so that we don't need nearly-empty ISRs */
    scb_hw->scr |= M33_SCR_SEVONPEND_BITS;

    set_sys_clock_48mhz();

    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    const unsigned slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    /* set up pwm to tick at 48 MHz (assuming sys is 48 MHz) and wrap 46875 times per second */
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 1);
#define TOP 1024U
    pwm_config_set_wrap(&config, TOP);

    dma_channel_claim(IDMA_PWM);
    dma_channel_config cfg = dma_channel_get_default_config(IDMA_PWM);
    channel_config_set_dreq(&cfg, pwm_get_dreq(slice_num));
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_ring(&cfg, false, BUFFER_WRAP_BITS);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);

    dma_channel_configure(IDMA_PWM,
                          &cfg,
                          (uint16_t *)((void *)&pwm_hw->slice[slice_num].cc) + (PWM_PIN % 2),
                          &buffer[0],
                          SAMPLES_PER_CHUNK | (1U << 28),
                          false);

    /* enable interrupt for dma, but leave it disabled in nvic */
    dma_channel_acknowledge_irq0(IDMA_PWM);
    dma_channel_set_irq0_enabled(IDMA_PWM, true);
    __dsb();
    irq_set_enabled(DMA_IRQ_0, false);

    dma_channel_start(IDMA_PWM);

    const float sample_rate = 48e6f / TOP;

    /* this can be any value between dc and fs/2, does not need to be an integer */
    const float tone_frequency = 900.0f;

    /* multiplier relative to full scale */
    const float tone_amplitude = 1.0f;

    const float complex advance = cexpf(I * 2.0f * (float)M_PI * tone_frequency / sample_rate);

    /* this will evolve along the unit circle */
    float complex carrier = -1.0f;

    size_t ichunk_filled = 0;
    while (1) {
        uint16_t * const dst = buffer[ichunk_filled % 2];
        for (size_t ival = 0; ival < SAMPLES_PER_CHUNK; ival++) {
            const float sample = crealf(carrier) * tone_amplitude;

            /* rotate complex sinusoid at the desired frequency */
            carrier *= advance;

            /* renormalize carrier to unity */
            carrier = carrier * (3.0f - cmagsquaredf(carrier)) / 2.0f;

            /* map [-1.0, 1.0] to [0, TOP] */
            dst[ival] = ((0.5f + 0.5f * sample) * TOP) + 0.5f + frand_minus_frand();
        }

        /* if we just filled the first chunk and have not enabled the pwm yet, enable it */
        if (0 == ichunk_filled && !(pwm_hw->slice[slice_num].csr & (1U << PWM_CH0_CSR_EN_LSB)))
            pwm_init(slice_num, &config, true);
        else {
            /* run other tasks or low power sleep until next dma interrupt */
            while (!(dma_hw->intr & 1U << IDMA_PWM))
                yield();

            /* acknowledge and clear the interrupt in both dma and nvic */
            dma_hw->ints0 = 1U << IDMA_PWM;
            irq_clear(DMA_IRQ_0);
        }

        ichunk_filled++;
    }
}
