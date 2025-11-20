#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <stdint.h>

void spi_master_init(void) {
    /* Habilitar relojes */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);

    /* PA5=SCK, PA6=MISO, PA7=MOSI en AF5 */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

    /* PA4 = NSS */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    gpio_set(GPIOA, GPIO4);   // NSS HIGH

    /* Config SPI1 como maestro */
    spi_reset(SPI1);
    spi_set_master_mode(SPI1);
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_64);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_unidirectional_mode(SPI1);
    spi_set_dff_8bit(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);
}

void spi_send_byte(uint8_t data) {
    spi_send(SPI1, data);
    spi_read(SPI1);      // leer basura del esclavo
}

int main(void) {
    spi_master_init();

    uint8_t mensaje[4] = {0x11, 0x22, 0x33, 0x44};

    while (1) {
        gpio_clear(GPIOA, GPIO4); // NSS LOW

        for (int i = 0; i < 4; i++) {
            spi_send_byte(mensaje[i]);
        }

        gpio_set(GPIOA, GPIO4);   // NSS HIGH

        /* Pequeño delay */
        for (volatile int i = 0; i < 2000000; i++);
    }
}
