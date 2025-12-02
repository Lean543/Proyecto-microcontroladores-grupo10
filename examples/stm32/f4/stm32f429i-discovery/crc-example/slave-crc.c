/*
 * STM32F429I-Discovery
 * SPI5 SLAVE - CRC monitor
 * El slave NO necesita conocer el mensaje del master.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/crc.h>

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define FRAME_START 0xAA
#define FRAME_MAX   200      // capacidad máxima de recepción

static uint8_t rx_frame[FRAME_MAX];

/*********** USART REDIRECT ***********/
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n')
            usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, ptr[i]);
    }
    return len;
}

/*********** CRC ***********/
static uint32_t crc32_hw_bytes(const uint8_t *data, size_t len)
{
    crc_reset();
    uint32_t word = 0;
    int count = 0;
    uint32_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        word |= ((uint32_t)data[i]) << (8 * count);
        count++;

        if (count == 4) {
            crc = crc_calculate(word);
            word = 0;
            count = 0;
        }
    }

    if (count > 0)
        crc = crc_calculate(word);

    return crc;
}

/*********** CLOCKS ***********/
static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_GPIOG);

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_SPI5);
    rcc_periph_clock_enable(RCC_CRC);
}

/*********** USART ***********/
static void usart_setup(void)
{
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);  // Solo TX
    usart_enable(USART1);
}


/*********** GPIO ***********/
static void gpio_setup(void)
{
    /* SPI5: PF7=SCK, PF8=MISO, PF9=MOSI */
    gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9);
    gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

    /* USART1 */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    /* LEDs: PG2 verde (OK), PG3 rojo (ERROR) */
    gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_clear(GPIOG, GPIO2 | GPIO3);
}

/*********** SPI SLAVE ***********/
static void spi_setup(void)
{
    spi_disable(SPI5);

    spi_enable_software_slave_management(SPI5);
    spi_set_nss_low(SPI5);

    spi_init_master(
        SPI5,
        SPI_CR1_BAUDRATE_FPCLK_DIV_64,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1,
        SPI_CR1_DFF_8BIT,
        SPI_CR1_MSBFIRST);

    spi_set_slave_mode(SPI5);

    spi_enable(SPI5);
}

/*********** RECEIVE FRAME ***********/
static void spi_receive_frame(void)
{
    uint8_t b = 0;

    /* esperar hasta detectar FRAME_START */
    while (b != FRAME_START) {
        b = spi_xfer(SPI5, 0x00);
    }
    rx_frame[0] = b;

    /* recibir longitud */
    uint8_t len = spi_xfer(SPI5, 0x00);
    rx_frame[1] = len;

    /* recibir payload */
    for (uint8_t i = 0; i < len; i++) {
        rx_frame[2 + i] = spi_xfer(SPI5, 0x00);
    }

    /* recibir CRC */
    for (uint8_t i = 0; i < 4; i++) {
        rx_frame[2 + len + i] = spi_xfer(SPI5, 0x00);
    }
}

/*********** MAIN ***********/
int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();

    printf("\n[SLAVE] Monitor SPI listo.\n");

    while (1) {
        spi_receive_frame();

        uint8_t len = rx_frame[1];

        uint32_t crc_rx =
              (uint32_t)rx_frame[2 + len]
            | (uint32_t)rx_frame[3 + len] << 8
            | (uint32_t)rx_frame[4 + len] << 16
            | (uint32_t)rx_frame[5 + len] << 24;

        uint32_t crc_calc = crc32_hw_bytes(&rx_frame[2], len);

        printf("\n[SLAVE] Recibido mensaje (%u bytes): ", len);
        for (int i = 0; i < len; i++)
            usart_send_blocking(USART1, rx_frame[2 + i]);

        printf("\nCRC RX   = 0x%08lX\n", crc_rx);
        printf("CRC CALC = 0x%08lX\n", crc_calc);

        if (crc_rx == crc_calc) {
            printf(">>> CRC OK\n");
            gpio_set(GPIOG, GPIO2);   // LED verde ON
            gpio_clear(GPIOG, GPIO3); // LED rojo OFF
        } else {
            printf(">>> CRC ERROR\n");
            gpio_clear(GPIOG, GPIO2);
            gpio_set(GPIOG, GPIO3);
        }
    }
}

