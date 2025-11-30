# IE0624 - Laboratorio de Microcontroladores  
## Actividad X – CRC sobre SPI entre dos STM32F429I

## Descripción

En esta actividad se estudiarán los siguientes aspectos:

- Repaso de la comunicación **SPI** entre dos microcontroladores.
- Uso del periférico **CRC hardware** del STM32F429I para generar un CRC32.
- Diseño de un **formato de trama** con datos + CRC.
- Verificación de integridad de datos en el **slave** mediante CRC.
- Detección de **corrupción de datos** (CRC mismatch) y señalización visual con LEDs externos.
- Uso de `printf` sobre **USART1 + VCP** para observar los resultados en `minicom`.

---

# IE0624 - Laboratorio de Microcontroladores

## Actividad X – CRC sobre SPI entre dos STM32F429I

---

## Descripción

En esta actividad se estudiarán los siguientes aspectos:

- Repaso de la comunicación **SPI** entre dos microcontroladores.
- Uso del periférico **CRC hardware** del STM32F429I para generar un CRC32.
- Diseño de un **formato de trama** con datos + CRC.
- Verificación de integridad de datos en el **slave** mediante CRC.
- Detección de **corrupción de datos** (CRC mismatch) y señalización visual con LEDs externos.
- Uso de `printf` sobre **USART1 + VCP** para observar los resultados en `minicom`.

---

## Materiales

- 2 × placas **STM32F429I-Discovery**
- 1 × protoboard
- 3 × LEDs (1 verde para el master, 1 verde y 1 rojo para el slave)
- 3 × resistencias (220–330 Ω) para los LEDs
- 1 × pulsador (botón)
- Jumpers hembra–hembra / macho–macho según se requiera
- PC con Linux, `arm-none-eabi-gcc`, `openocd`, `minicom` y repositorio con ejemplos `libopencm3`

---

## Instrucciones

### Parte 1 – Comunicación SPI + CRC correcto

En esta primera parte se construye una comunicación básica entre dos STM32 donde el **master** envía un mensaje con CRC y el **slave** verifica que el CRC sea correcto.

#### 1. Preparación del proyecto

1. Diríjase al directorio de ejemplos para la STM32F429I-Discovery (ajuste la ruta a su entorno):

   ```bash
   cd ~/laboratorio_microcontroladores/Proyecto-microcontroladores-grupo10
   cd examples/stm32/f4/stm32f429i-discovery
   ```

2. Configure el Makefile para generar dos binarios (por ejemplo `master-crc` y `slave-crc`) usando la misma estructura que otros ejemplos del directorio.

   **E:** Entregue el Makefile y una captura de la salida de make donde se vea que se compilan ambos binarios sin errores.

#### 2. Conexión SPI entre las dos STM

En ambas placas utilice el bloque SPI5 con los siguientes pines:

- **PF7** = SPI5_SCK
- **PF8** = SPI5_MISO
- **PF9** = SPI5_MOSI
- **PF10** = línea de CS lógico (chip select manual como GPIO)

Conecte así MASTER ↔ SLAVE usando jumpers:

- PF7 (MASTER) ↔ PF7 (SLAVE) – reloj (SCK)
- PF9 (MASTER) ↔ PF9 (SLAVE) – MOSI (datos master→slave)
- PF8 (MASTER) ↔ PF8 (SLAVE) – MISO (datos slave→master, opcional en este demo)
- PF10 (MASTER) ↔ PF10 (SLAVE) – CS lógico (solo para referencia)
- GND (MASTER) ↔ GND (SLAVE) – referencia común

   **E:** Entregue una foto o diagrama donde se vean claramente las conexiones SPI y GND entre ambas placas.

#### 3. LED y botón externos en el MASTER

En el master utilice:

- **PG0** → LED verde externo (estado de transmisión)
- **PG1** → botón externo (disparo del envío)

Conexión en la protoboard:

**LED:**

- PG0 → resistencia (220–330 Ω) → ánodo del LED verde (pata larga)
- Cátodo del LED → GND de la STM

**Botón:**

- PG1 → una pata del pulsador
- Otra pata del pulsador → GND de la STM

En el código configure PG1 como entrada con pull-up interno.

#### 4. MASTER – Envío de una trama con CRC

Implemente en `master_crc_spi.c`:

**Inicialización:**

- Reloj a 168 MHz (como en otros ejemplos de la placa)
- GPIO para SPI5, LED (PG0), botón (PG1)
- CRC hardware (RCC_CRC)
- USART1 (PA9/PA10) para logs por VCP (115200 bps)

Mensaje fijo en el master, por ejemplo:

```c
static const char mensaje[] = "Hola desde la STM32 MASTER con CRC";
```

**Formato de trama** (en un `uint8_t frame[]`):

- Byte 0: `FRAME_START = 0xAA`
- Byte 1: `LEN` (longitud del mensaje en bytes)
- Bytes 2..(1+LEN): datos del mensaje
- Últimos 4 bytes: CRC32 calculado con el periférico CRC sobre los datos del mensaje

Función para calcular CRC32 usando el periférico CRC e ir alimentando de a 32 bits.

**Comportamiento principal:**

1. Al iniciar, mostrar por printf el mensaje y la longitud
2. Esperar a que se presione el botón (PG1 a GND)
3. Al detectar el botón:
  - Construir la trama (`build_frame()`)
  - Encender y hacer parpadear el LED de PG0 mientras se envían los bytes por SPI5 con `spi_xfer()`
  - Apagar el LED al finalizar
  - Imprimir en la consola del master que la trama fue enviada

   **E:** Entregue el archivo `master_crc_spi.c`.


#### 5. SLAVE – Recepción y verificación del CRC

En `slave_crc_spi.c`:

**Inicialice:**

- Reloj, GPIO para SPI5 y USART1 como en el master
- CRC hardware
- Configure SPI5 en modo SLAVE, con la misma polaridad/fase (CPOL=0, CPHA=1), 8 bits, MSB first

Implemente una función de recepción bloqueante de la trama:

1. Esperar a recibir el byte `FRAME_START`
2. Leer el resto de bytes hasta completar `FRAME_LEN`
3. Extraer:
  - `LEN = rx_frame[1]`
  - Datos: `rx_frame[2 .. 1+LEN]`
  - CRC recibido en los 4 bytes finales
4. Recalcular el CRC sobre los datos usando el periférico CRC
5. Comparar `CRC_RX` vs `CRC_CALC`
6. Imprimir en minicom:
  - El mensaje recibido como texto
  - El CRC recibido y el CRC calculado en hexadecimal
  - Un mensaje "CRC OK" si coinciden

   **E:** Entregue un video corto (o captura de pantalla animada) donde se vea:
   - El master enviando la trama al presionar el botón
   - El slave mostrando el mensaje recibido y el CRC coincidente en minicom

---

### Parte 2 – Inyección de errores y detección de corrupción

En esta segunda parte, el master debe ser capaz de enviar tramas intencionalmente corruptas, y el slave debe detectarlas con el CRC y señalizarlas con LEDs externos.

#### 6. LEDs externos de estado en el SLAVE

En el slave, agregue dos LEDs externos:

- LED verde (OK) → PG2
- LED rojo (ERROR) → PG3

Conexión en la protoboard (en la placa SLAVE):

**LED verde:**
- PG2 → resistencia (220–330 Ω) → ánodo LED verde
- Cátodo LED verde → GND

**LED rojo:**
- PG3 → resistencia (220–330 Ω) → ánodo LED rojo
- Cátodo LED rojo → GND

En el código:

- Configure PG2 y PG3 como salidas
- Inicialice ambos apagados
- Cuando `CRC_RX == CRC_CALC`:
  - Encender LED verde (PG2), apagar LED rojo (PG3)
- Cuando `CRC_RX != CRC_CALC`:
  - Apagar LED verde, encender LED rojo

#### 7. MASTER – Introducir una corrupción en los datos

Modifique el código del master para poder alternar entre enviar tramas correctas y tramas corruptas. Una forma sencilla:

Definir una variable global:

```c
static int inject_error = 0;  /* 0 = trama correcta, 1 = trama corrupta */
```

En `build_frame()`:

- Calcular el CRC sobre el mensaje correcto
- Guardar el CRC en la trama
- Si `inject_error == 1`, modificar uno de los bytes de datos después de calcular el CRC (por ejemplo, hacer `frame[2] ^= 0x01;`)

En el `main()` del master:

1. Cada vez que se presione el botón:
  - Llamar a `build_frame()` (que toma en cuenta `inject_error`)
  - Enviar la trama
  - Alternar `inject_error ^= 1;` para que la siguiente vez se envíe el otro tipo de trama

Así, el primer envío será correcto, el segundo corrupto, el tercero correcto, etc.

   **E:** Entregue el fragmento de código donde se introduce la corrupción en los datos del frame y una explicación breve (1–2 líneas) de qué está cambiando exactamente.

#### 8. SLAVE – Detección de errores y alerta visual

Con el mecanismo anterior:

- Cuando reciba una trama correcta:
  - `CRC_RX == CRC_CALC`
  - En minicom se ve "CRC OK"
  - LED verde del slave (PG2) encendido, LED rojo apagado
- Cuando reciba una trama corrupta:
  - `CRC_RX != CRC_CALC`
  - En minicom se ve "CRC ERROR" o "CRC mismatch"
  - LED rojo del slave encendido, LED verde apagado

   **E:** Entregue un video donde se observe claramente:
   - Varios pulsos del botón en el master
   - Alternancia entre tramas correctas y corruptas (según el mensaje de log)
   - El cambio de estado de los LEDs externos del slave (verde ↔ rojo) dependiendo del resultado del CRC

---

## Preguntas para reflexionar

**P1:** ¿Por qué es conveniente incluir un byte de inicio (`FRAME_START`) y un byte de longitud (`LEN`) en la trama, en lugar de enviar solo los datos y el CRC?

**P2:** Suponga que, en lugar de modificar un bit de los datos, decidiera modificar uno de los bytes del CRC enviado por el master. ¿Seguiría siendo detectada la corrupción por el slave? Justifique brevemente.

**P3:** Compare el uso de un CRC con el de una suma simple de verificación (checksum de 8 bits). ¿En qué casos un CRC ofrece mayor capacidad de detección de errores que un checksum sencillo?

_(Puede responder estas preguntas en el informe del laboratorio o como texto adicional en el README de su ejemplo.)_
