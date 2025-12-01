/*
 * STM32F429I-Discovery
 * SPI5 SLAVE + CRC + USART + LEDs OK/ERR
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
#define MSG_LEN     ((uint8_t)(sizeof("Hola desde la STM32 MASTER con CRC") - 1))
#define FRAME_LEN   (1 + 1 + MSG_LEN + 4)

static uint8_t rx_frame[FRAME_LEN];

/*********** NUEVO: LEDs ***********/
#define LED_OK_PORT   GPIOG
#define LED_OK_PIN    GPIO13

#define LED_ERR_PORT  GPIOG
#define LED_ERR_PIN   GPIO14
/***********************************/

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
    int byte_count = 0;
    uint32_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        word |= ((uint32_t)data[i]) << (8 * byte_count);
        byte_count++;

        if (byte_count == 4) {
            crc = crc_calculate(word);
            word = 0;
            byte_count = 0;
        }
    }

    if (byte_count > 0)
        crc = crc_calculate(word);

    return crc;
}

/*********** CLOCKS ***********/
static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_SPI5);

    rcc_periph_clock_enable(RCC_CRC);

    /*********** NUEVO: GPIOG para LEDs ***********/
    rcc_periph_clock_enable(RCC_GPIOG);
}

/*********** GPIO ***********/
static void gpio_setup(void)
{
    /* SPI5: PF7=SCK, PF8=MISO, PF9=MOSI */
    gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO8 | GPIO9);
    gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
                            GPIO7 | GPIO8 | GPIO9);
    gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

    /* USART1 PA9=TX PA10=RX */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

    /*********** NUEVO: LEDs OK / ERROR ***********/
    gpio_mode_setup(LED_OK_PORT,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_OK_PIN);
    gpio_mode_setup(LED_ERR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERR_PIN);

    gpio_clear(LED_OK_PORT, LED_OK_PIN);
    gpio_clear(LED_ERR_PORT, LED_ERR_PIN);
}

/*********** USART ***********/
static void usart_setup(void)
{
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX | USART_MODE_RX);
    usart_enable(USART1);
}

/*********** SPI SLAVE ***********/
static void spi_setup(void)
{
    spi_disable(SPI5);

    SPI_CR1(SPI5) =
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE |
        SPI_CR1_CPHA_CLK_TRANSITION_1 |
        SPI_CR1_DFF_8BIT |
        SPI_CR1_MSBFIRST;

    spi_set_full_duplex_mode(SPI5);

    spi_disable_software_slave_management(SPI5);
    spi_set_nss_low(SPI5);

    spi_set_slave_mode(SPI5);

    spi_enable(SPI5);
}

/*********** RECEIVE FRAME ***********/
static void spi_receive_frame_blocking(void)
{
    uint8_t b = 0;

    while (b != FRAME_START) {
        b = spi_xfer(SPI5, 0x00);
    }

    rx_frame[0] = b;

    for (size_t i = 1; i < FRAME_LEN; i++) {
        rx_frame[i] = spi_xfer(SPI5, 0x00);
    }
}

/*********** MAIN ***********/
int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();

    printf("\n[SLAVE] Inicializado correctamente\n");
    printf("[SLAVE] Esperando datos del MASTER...\n");

    while (1) {

        /* Apagar LEDs antes de siguiente recepción */
        gpio_clear(LED_OK_PORT, LED_OK_PIN);
        gpio_clear(LED_ERR_PORT, LED_ERR_PIN);

        spi_receive_frame_blocking();

        printf("\n[SLAVE] Trama recibida\n");

        uint8_t len = rx_frame[1];
        uint32_t crc_rx = rx_frame[2 + len] |
                         (rx_frame[3 + len] << 8) |
                         (rx_frame[4 + len] << 16) |
                         (rx_frame[5 + len] << 24);

        uint32_t crc_calc = crc32_hw_bytes(&rx_frame[2], len);

        printf("[SLAVE] Mensaje: ");
        for (int i = 0; i < len; i++)
            usart_send_blocking(USART1, rx_frame[2 + i]);
        printf("\n");

        printf("CRC RX = 0x%08lX\n", crc_rx);
        printf("CRC CALC = 0x%08lX\n", crc_calc);

        if (crc_rx == crc_calc) {
            printf(">>> CRC OK\n");
            gpio_set(LED_OK_PORT, LED_OK_PIN);
        } else {
            printf(">>> CRC ERROR\n");
            gpio_set(LED_ERR_PORT, LED_ERR_PIN);
        }
    }

    return 0;
}


