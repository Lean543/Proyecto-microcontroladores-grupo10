#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <stdint.h>

void spi_slave_init(void) {
    /* Habilitar relojes */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_SPI1);

    /* PA4=SS, PA5=SCK, PA6=MISO, PA7=MOSI AF5 */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO6 | GPIO7);

    /* Config SPI1 como esclavo */
    spi_reset(SPI1);
    spi_set_slave_mode(SPI1);
    spi_set_dff_8bit(SPI1);
    spi_set_full_duplex_mode(SPI1);
    spi_set_unidirectional_mode(SPI1);
    spi_set_clock_polarity_0(SPI1);
    spi_set_clock_phase_0(SPI1);
    spi_enable(SPI1);
}

uint8_t spi_receive_byte(void) {
    return spi_read(SPI1);
}

int main(void) {
    spi_slave_init();

    uint8_t buffer[4];

    while (1) {
        /* El esclavo recibe automáticamente cuando NSS baja y el maestro genera reloj */
        for (int i = 0; i < 4; i++) {
            buffer[i] = spi_receive_byte();
        }

        /* Aquí podés imprimir por UART, prender un LED, o calcular CRC */
        /* Ejemplo: si recibí 0x11 0x22 0x33 0x44, lo guardo en buffer[] */

        for (volatile int i = 0; i < 100000; i++);
    }
}
