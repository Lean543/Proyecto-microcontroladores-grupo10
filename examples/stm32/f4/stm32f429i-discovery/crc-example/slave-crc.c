/* stm32f429i_slave_crc_spi.c
 *
 * Demo: SLAVE SPI5 + CRC
 * Placa: STM32F429I-Discovery
 *
 * Pines usados (ajusta si usas otros):
 *  - SPI5:
 *      PF7  = SPI5_SCK
 *      PF8  = SPI5_MISO
 *      PF9  = SPI5_MOSI
 *  - USART1 para logs: PA9 (TX) -> adaptador USB-Serie / ST-LINK VCP
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/crc.h>

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* La trama debe ser coherente con el MASTER */

#define FRAME_START   0xAA

/* Copiamos la misma longitud de mensaje que en el master */
#define MSG_LEN   ((uint8_t)(sizeof("Hola desde la STM32 MASTER con CRC") - 1))
#define FRAME_LEN (1 + 1 + MSG_LEN + 4)

static uint8_t rx_frame[FRAME_LEN];

/* USART1 para logs */
#define USART_CONSOLE USART1
#define USART_PORT    GPIOA
#define USART_TX_PIN  GPIO9
#define USART_RX_PIN  GPIO10

/* --- Utilidades --- */

static void delay(volatile uint32_t count)
{
    while (count--) {
        __asm__("nop");
    }
}

/* Redirección de printf hacia USART1 */
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            usart_send_blocking(USART_CONSOLE, '\r');
        }
        usart_send_blocking(USART_CONSOLE, ptr[i]);
    }
    return len;
}

/* CRC hardware sobre bytes (misma función que en el master) */
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

    if (byte_count > 0) {
        crc = crc_calculate(word);
    }

    return crc;
}

/* --- Inicialización básica --- */

static void clock_setup(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);

    rcc_periph_clock_enable(RCC_SPI5);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_CRC);
}

static void gpio_setup(void)
{
    /* SPI5 PF7=SCK, PF8=MISO, PF9=MOSI */
    gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO7 | GPIO8 | GPIO9);
    gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO7 | GPIO8 | GPIO9);
    gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

    /* USART1: PA9=TX, PA10=RX */
    gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    USART_TX_PIN | USART_RX_PIN);
    gpio_set_af(USART_PORT, GPIO_AF7, USART_TX_PIN | USART_RX_PIN);
}

static void usart_setup(void)
{
    usart_set_baudrate(USART_CONSOLE, 115200);
    usart_set_databits(USART_CONSOLE, 8);
    usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
    usart_set_mode(USART_CONSOLE, USART_MODE_TX);  /* solo TX */
    usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
    usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);
    usart_enable(USART_CONSOLE);
}

static void spi_setup(void)
{
    /* Reutilizamos la misma configuración base que en master,
     * pero luego ponemos el periférico en modo SLAVE
     */
    spi_init_master(SPI5,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_set_full_duplex_mode(SPI5);

    /* Gestión NSS por software para evitar problemas con la línea NSS hardware */
    spi_enable_software_slave_management(SPI5);
    spi_set_nss_high(SPI5);

    /* Ahora pasamos a modo SLAVE */
    spi_set_slave_mode(SPI5);

    spi_enable(SPI5);
}

/* Recepción bloqueante de una trama completa:
 * Espera al byte FRAME_START, luego lee FRAME_LEN-1 bytes más
 */
static void spi_receive_frame_blocking(void)
{
    uint8_t b;

    /* Esperar primer byte de inicio */
    while (1) {
        b = (uint8_t)spi_xfer(SPI5, 0x00);
        if (b == FRAME_START) {
            rx_frame[0] = b;
            break;
        }
    }

    /* Leer el resto de la trama */
    for (size_t i = 1; i < FRAME_LEN; i++) {
        rx_frame[i] = (uint8_t)spi_xfer(SPI5, 0x00);
    }
}

static void print_frame_hex(const uint8_t *buf, size_t len)
{
    printf("[SLAVE] Trama HEX (%u bytes):", (unsigned)len);
    for (size_t i = 0; i < len; i++) {
        printf(" %02X", buf[i]);
    }
    printf("\n");
}

/* --- main --- */

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();

    printf("\n[SLAVE] Demo CRC + SPI STM32F429I\n");
    printf("[SLAVE] Esperando tramas desde el MASTER...\n");
    printf("[SLAVE] FRAME_LEN = %u bytes, MSG_LEN = %u bytes\n",
           (unsigned)FRAME_LEN, (unsigned)MSG_LEN);

    while (1) {
        spi_receive_frame_blocking();

        uint8_t len = rx_frame[1];

        if (len != MSG_LEN) {
            printf("\n[SLAVE] Longitud inesperada en trama: len=%u (esperado=%u)\n",
                   (unsigned)len, (unsigned)MSG_LEN);
            print_frame_hex(rx_frame, FRAME_LEN);
            continue;
        }

        /* Extraer CRC recibido */
        size_t idx_crc = 2 + len;
        uint32_t crc_rx =  (uint32_t)rx_frame[idx_crc + 0]
                         | ((uint32_t)rx_frame[idx_crc + 1] << 8)
                         | ((uint32_t)rx_frame[idx_crc + 2] << 16)
                         | ((uint32_t)rx_frame[idx_crc + 3] << 24);

        /* Recalcular CRC sobre los datos */
        uint32_t crc_calc = crc32_hw_bytes(&rx_frame[2], len);

        printf("\n[SLAVE] Trama recibida.\n");
        printf("[SLAVE] Mensaje (%u bytes): \"", (unsigned)len);
        for (uint8_t i = 0; i < len; i++) {
            char c = (char)rx_frame[2 + i];
            if (c >= 32 && c <= 126) {
                printf("%c", c);
            } else {
                printf(".");
            }
        }
        printf("\"\n");

        printf("[SLAVE] CRC RX   = 0x%08lX\n", (unsigned long)crc_rx);
        printf("[SLAVE] CRC calc = 0x%08lX\n", (unsigned long)crc_calc);

        if (crc_rx == crc_calc) {
            printf("[SLAVE] >>> CRC OK: datos correctos.\n");
        } else {
            printf("[SLAVE] >>> CRC ERROR: datos corruptos.\n");
            print_frame_hex(rx_frame, FRAME_LEN);
        }

        /* Pequeña pausa antes de aceptar otra trama */
        delay(200000);
    }

    return 0;
}
