/* stm32f429i_master_crc_spi.c
 *
 * Demo: MASTER SPI5 + CRC + LED + Botón
 * Placa: STM32F429I-Discovery
 *
 * Pines usados (ajusta si usas otros):
 *  - SPI5:
 *      PF7  = SPI5_SCK
 *      PF8  = SPI5_MISO
 *      PF9  = SPI5_MOSI
 *  - LED1 externo: PG0 (salida, LED + resistencia a 3V3)
 *  - Botón externo: PG1 (entrada con pull-up interno, botón a GND)
 *  - USART1 para logs: PA9 (TX) -> adaptador USB-Serie / ST-LINK VCP
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/crc.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* --- Configuración de mensaje y trama --- */

#define FRAME_START   0xAA

static const char mensaje[] = "Hola desde la STM32 MASTER con CRC";
#define MSG_LEN   ((uint8_t)(sizeof(mensaje) - 1))  /* sin el '\0' */

#define FRAME_LEN  (1 + 1 + MSG_LEN + 4)
/* 1 byte start + 1 byte len + datos + 4 bytes CRC */

static uint8_t frame[FRAME_LEN];
static int inject_error = 0;  /* 0 = mensaje OK, 1 = mensaje corrupto */

/* --- Pines (ajustables) --- */

/* LED y botón externos en puerto G */
#define LED1_PORT   GPIOG
#define LED1_PIN    GPIO0

#define BTN_PORT    GPIOG
#define BTN_PIN     GPIO1

/* SPI5 en puerto F */
#define SPI_PORT    GPIOF
#define SPI_SCK     GPIO7
#define SPI_MISO    GPIO8
#define SPI_MOSI    GPIO9

/* Chip Select manual como GPIO */
#define SPI_CS_PORT GPIOF
#define SPI_CS_PIN  GPIO10

/* USART1 para logs */
#define USART_CONSOLE USART1
#define USART_PORT    GPIOA
#define USART_TX_PIN  GPIO9
#define USART_RX_PIN  GPIO10

/* --- Utilidades básicas --- */

static void delay(volatile uint32_t count)
{
    while (count--) {
        __asm__("nop");
    }
}

static int button_pressed(void)
{
    /* Botón a GND, pull-up interno: nivel bajo = pulsado */
    return gpio_get(BTN_PORT, BTN_PIN) == 0;
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

/* --- CRC sobre bytes usando hardware CRC --- */

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

/* --- Inicialización de relojes, GPIO, USART y SPI --- */

static void clock_setup(void)
{
    /* 168 MHz a partir de HSE 8 MHz (típico en F429I-Discovery) */
    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    /* GPIOs */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_GPIOG);

    /* SPI5 */
    rcc_periph_clock_enable(RCC_SPI5);

    /* USART1 */
    rcc_periph_clock_enable(RCC_USART1);

    /* CRC */
    rcc_periph_clock_enable(RCC_CRC);
}

static void gpio_setup(void)
{
    /* LED1 como salida push-pull */
    gpio_mode_setup(LED1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED1_PIN);
    gpio_set_output_options(LED1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, LED1_PIN);
    gpio_clear(LED1_PORT, LED1_PIN);  /* LED apagado inicialmente */

    /* Botón como entrada con pull-up */
    gpio_mode_setup(BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BTN_PIN);

    /* Pines SPI5: PF7=SCK, PF8=MISO, PF9=MOSI */
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    SPI_SCK | SPI_MISO | SPI_MOSI);
    gpio_set_output_options(SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            SPI_SCK | SPI_MISO | SPI_MOSI);
    gpio_set_af(SPI_PORT, GPIO_AF5, SPI_SCK | SPI_MISO | SPI_MOSI);

    /* CS manual en PF10 */
    gpio_mode_setup(SPI_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_PIN);
    gpio_set_output_options(SPI_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SPI_CS_PIN);
    gpio_set(SPI_CS_PORT, SPI_CS_PIN);  /* CS inactivo alto */

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

    /* Modo master, 8 bits, MSB primero, CPOL=0, CPHA=1 (modo SPI 1) */
    spi_init_master(SPI5,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT,
                    SPI_CR1_MSBFIRST);

    spi_set_full_duplex_mode(SPI5);

    /* No usamos el pin NSS hardware: gestión por software para evitar MODE FAULT */
    spi_enable_software_slave_management(SPI5);
    spi_set_nss_high(SPI5);

    spi_enable(SPI5);
}

/* --- Construcción y envío de la trama --- */

static void build_frame(void)
{
    frame[0] = FRAME_START;
    frame[1] = MSG_LEN;
    memcpy(&frame[2], mensaje, MSG_LEN);

    /* 1) Calcular el CRC sobre los DATOS correctos */
    uint32_t crc = crc32_hw_bytes((const uint8_t *)&frame[2], MSG_LEN);

    /* 2) Guardar el CRC en la trama */
    size_t idx = 2 + MSG_LEN;
    frame[idx + 0] = (uint8_t)(crc & 0xFFu);
    frame[idx + 1] = (uint8_t)((crc >> 8) & 0xFFu);
    frame[idx + 2] = (uint8_t)((crc >> 16) & 0xFFu);
    frame[idx + 3] = (uint8_t)((crc >> 24) & 0xFFu);

    /* Introducir un fallo, se modifica un byte de los datos después de haber calculado el CRC. Así el CRC ya no corresponde.*/
    if (inject_error) {

        frame[2] ^= 0x01;  /* flip del bit 0 del primer byte de datos */
    }
}


static void spi_send_frame(void)
{
    /* LED fijo encendido mientras está en "modo envío" */
    gpio_set(LED1_PORT, LED1_PIN);

    /* Selecciona al slave (CS bajo) */
    gpio_clear(SPI_CS_PORT, SPI_CS_PIN);
    delay(10000);

    for (size_t i = 0; i < FRAME_LEN; i++) {
        /* Parpadeo visible: toggle por byte + pequeña pausa */
        gpio_toggle(LED1_PORT, LED1_PIN);
        spi_xfer(SPI5, frame[i]);
        delay(500000);
    }

    /* Desseleccionar slave */
    gpio_set(SPI_CS_PORT, SPI_CS_PIN);

    /* LED vuelve al estado apagado cuando termina el envío */
    gpio_clear(LED1_PORT, LED1_PIN);
}

/* --- main --- */

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();

    printf("\n[MASTER] Demo CRC + SPI STM32F429I\n");
    printf("[MASTER] Longitud mensaje: %u bytes\n", (unsigned)MSG_LEN);
    printf("[MASTER] Texto mensaje: \"%s\"\n", mensaje);
    printf("[MASTER] Pulsa el boton para enviar la trama.\n");

    build_frame();

    while (1) {
        if (button_pressed()) {
            /* Debounce sencillo */
            delay(500000);
            if (button_pressed()) {

                /* Construir la trama según el modo actual (OK o corrupta) */
                build_frame();

                if (inject_error) {
                    printf("\n[MASTER] Boton pulsado, ENVIANDO trama CORRUPTA (%u bytes)...\n",
                        (unsigned)FRAME_LEN);
                } else {
                    printf("\n[MASTER] Boton pulsado, ENVIANDO trama CORRECTA (%u bytes)...\n",
                        (unsigned)FRAME_LEN);
                }

                spi_send_frame();

                printf("[MASTER] Trama enviada.\n");

                /* Alternar el modo para la próxima vez:
                *  - Primera vez: inject_error = 0 -> OK
                *  - Luego se vuelve 1 -> CORRUPTA
                *  - Luego 0 -> OK
                *  - etc.
                */
                inject_error ^= 1;

                /* Esperar a que se suelte el botón */
                while (button_pressed()) {
                    /* nada */
                }
            }
        }
    }

    return 0;
}
