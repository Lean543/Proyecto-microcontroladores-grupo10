## 2. Conexión SPI entre las dos STM

En las dos placas usamos el mismo bloque **SPI5** y sus pines:

- `PF7` = `SPI5_SCK`
- `PF8` = `SPI5_MISO`
- `PF9` = `SPI5_MOSI`
- `PF10` = `CS` (chip select)

Conexión **MASTER ↔ SLAVE** usando jumpers hembra–hembra:

- **Reloj (SCK)**  
  `PF7 (MASTER)` ↔ `PF7 (SLAVE)`

- **MOSI (datos de MASTER hacia SLAVE)**  
  `PF9 (MASTER)` ↔ `PF9 (SLAVE)`

- **MISO (datos de SLAVE hacia MASTER)**  
  `PF8 (MASTER)` ↔ `PF8 (SLAVE)`

- **Chip Select (CS)**  
  `PF10 (MASTER)` ↔ `PF10 (SLAVE)`

- **Referencia común**  
  `GND (MASTER)` ↔ `GND (SLAVE)`

---

## LED 1 externo en el MASTER

En el código usamos `PG0` como pin del LED.

**Componentes:**

- 1 × LED
- 1 × resistencia (220 Ω a 1 kΩ; típico 330 Ω)

**Conexión en la protoboard (solo al MASTER):**

1. Llevar un jumper desde `PG0` de la STM MASTER a la protoboard.
2. En la protoboard:
   - `PG0` → un extremo de la **resistencia**.
   - Otro extremo de la resistencia → **ánodo** del LED (pata larga).
   - **Cátodo** del LED (pata corta) → **GND** de la STM (un pin GND llevado a la protoboard).

Esquema lógico:

```text
PG0 (MASTER) ──[ 330 Ω ]──►|── GND
                            LED
