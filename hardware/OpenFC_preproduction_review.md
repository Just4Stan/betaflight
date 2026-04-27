# OpenFC Pre-Production Design Review

**Board**: OpenFC -- Custom Betaflight Flight Controller
**MCU**: RP2354B (QFN-80, 48 GPIO, 2MB internal flash)
**Date**: 2026-02-21
**Schematic**: `hardware/OpenFC.kicad_sch` (KiCad 9.0.6)
**Method**: kicad-cli netlist export + automated cross-referencing against datasheets and Betaflight source

---

## Design Summary

| Subsystem | Component | Part Number |
|-----------|-----------|-------------|
| MCU | RP2354B | QFN-80, 2MB internal flash |
| IMU | LSM6DSV16XTR | 6-DoF gyro/accel, SPI0 |
| ELRS RX | ESP32-C3FH4 + SX1281IMLTRT | 2.4 GHz LoRa, onboard |
| Blackbox Flash | BY25Q128ASWIG | 128 Mbit SPI NOR, SPI1 |
| Barometer | BMP388 | I2C0 |
| Compass | LIS3MDLTR | I2C0, DNP |
| OSD | GS8052-FR + PI3A223ZMEX | PIO-based analog video overlay |
| 12V Buck | LMR51420YDDCR (U6) | 4.5-36V in |
| 5V Buck | LMR51420YDDCR (U7) | 4.5-36V in, always on |
| Power Mux | TPS2116DRLR | USB/Buck auto-switchover |
| 3.3V LDO (main) | TPS746-3.3DRV | 500 mA, ultra-low-noise |
| 3.3V LDO (ELRS) | WL9005D4-33 | Separate RF supply |
| 1.8V LDO (gyro) | NCV8187AMT180TAG | With power-good output |
| Level Shifter | TXU0101DCKR | 3.3V to 5V for LED strip |
| USB ESD | USBLC6-2SC6 | USB-C data line protection |

### Power Tree
```
+BATT (2S-8S LiPo, 7.4-33.6V)
  +-- D21 (Schottky clamp to GND)
  +-- LMR51420 (U6) --> +12V (R14=100k/R28=5.23k, Vref=0.6V => 12.07V)
  +-- LMR51420 (U7) --> +5V_BUCK (R16=100k/R31=13.7k => 4.98V)
  |     +-- D3 (Schottky bypass to +5V)
  +-- TPS2116 (U33): VIN1=5V_BUCK, VIN2=5V_USB --> +5V
        +-- TPS746 (U13) --> +3.3V (main)
        |     +-- RP2354B internal VREG --> +1.1V (DVDD)
        +-- WL9005D4-33 (U12) --> +3.3V_ELRS
        +-- NCV8187 (U21) --> +1.8V_GYRO
```

---

## CRITICAL -- Must Fix Before Ordering

### CRIT-1: SX1281 VDD_IN Has No Power Supply Connection

**Subsystem**: ELRS Receiver
**Component**: U20 (SX1281IMLTRT), pins 2 (VDD_IN) and 12 (DCC_FB)
**Confidence**: HIGH (verified in fresh kicad-cli netlist export)

The SX1281's internal regulator input (VDD_IN) and DC-DC feedback (DCC_FB) are tied together with decoupling caps C1 (10nF) and C7 (470nF) to GND, but **no connection exists to any power rail**. VBAT (pin 15) and VBAT_IO (pin 11) are correctly connected to +3.3V_ELRS, but VDD_IN is isolated.

The SX1281 operates in LDO mode (DCC_SW unconnected). In this mode, VDD_IN must be tied to VBAT per the datasheet reference design.

**Impact**: The SX1281 radio will not power up. ELRS receiver is completely non-functional.

**Fix**: Connect the VDD_IN/DCC_FB net to +3.3V_ELRS. A wire or 0R jumper from the VDD_IN/DCC_FB net to VBAT (+3.3V_ELRS) is sufficient.

---

### CRIT-2: LED Resistors R4 and R22 (30R) Cause Destructive Overcurrent

**Subsystem**: Status LEDs
**Components**: R4 (30R, 0402), D4 (green LED, 0603); R22 (30R, 0402), D6 (green LED, 0603)
**Confidence**: HIGH

- D4: R4(30R) from +3.3V rail. Current = (3.3V - 2.0V) / 30R = **43mA**. Typical 0603 LED max is 20mA.
- D6: R22(30R) from GPIO12. Current = (3.3V - 2.0V) / 30R = **43mA**. The RP2354B GPIO max source current is ~12mA.

**Impact**: D6 will exceed GPIO current limit and may damage the MCU pin. Both LEDs will be over-driven.

**Fix**: Change R4 and R22 to **200R** (for ~6.5mA) or **330R** (for ~4mA).

---

### CRIT-3: Output Cap C38 (22uF 0603) on 12V Rail Exceeds Voltage Rating

**Subsystem**: Power -- 12V Buck
**Component**: C38 (22uF, 0603)
**Confidence**: HIGH

A 22uF MLCC in 0603 package is typically rated 6.3V or 10V max. The 12V output exceeds both common ratings. Operating an MLCC above its voltage rating risks dielectric breakdown, cracking, or catastrophic short circuit.

**Impact**: C38 failure could short the 12V rail or cause fire.

**Fix**: Replace C38 with a 22uF cap in **0805 or 1210 package rated >= 25V**. Alternatively use 2x 10uF 0805 25V in parallel.

---

### CRIT-4: Motor Pin Numbering Reversed Between Schematic and Firmware Config

**Subsystem**: Betaflight Target Config
**Files**: `hardware/*.kicad_sch` vs `src/config/configs/OPENFC_RP2350B/config.h`
**Confidence**: HIGH (verified from netlist and config)

| GPIO | Schematic Label | Config Definition |
|------|----------------|-------------------|
| GPIO28 (PA28) | M4 | MOTOR1_PIN |
| GPIO29 (PA29) | M3 | MOTOR2_PIN |
| GPIO30 (PA30) | M2 | MOTOR3_PIN |
| GPIO31 (PA31) | M1 | MOTOR4_PIN |

The connector P1 (JST SH) has: Pin 3=M4 (GPIO28), Pin 4=M3, Pin 5=M2, Pin 6=M1 (GPIO31).

**Impact**: If board silkscreen says "M1" next to J3/P1.6, users will connect Motor 1 ESC there, but firmware sends MOTOR1 on GPIO28 (labeled M4). Motors will spin with wrong mapping.

**Fix**: Either update config.h to match schematic labels, or update schematic labels to match config. Ensure board silkscreen matches whichever is chosen.

---

### CRIT-5: 22uF Capacitors in 0402 Package -- Voltage Derating Concern

**Subsystem**: Power / Decoupling
**Components**: C2 (22uF, 0402), C16 (22uF, 0402), C17 (22uF, 0402)
**Confidence**: MEDIUM (depends on exact part voltage rating)

22uF in 0402 typically has 4V max rating. Class II ceramics (X5R/X7R) derate heavily under DC bias:
- C16/C17 on 3.3V rail: 3.3V on a 4V-rated part = ~50-70% capacitance loss, leaving only 7-10uF effective. Only 0.7V headroom (17%).
- C2 on 1.8V rail: Less severe derating, ~15uF effective. Acceptable.

**Impact**: LDO output caps may have insufficient effective capacitance for stability. TPS746 requires min 1uF output cap.

**Fix**: Verify LCSC C105226 voltage rating. If 4V, consider upgrading C16/C17 to 6.3V-rated parts (likely 0603 package) or adding parallel 100nF caps to ensure minimum capacitance.

---

## WARNING -- Should Fix

| # | Issue | Subsystem | Fix |
|---|-------|-----------|-----|
| W-1 | SX1281 NSS/CS line has no pull-up resistor -- floats during ESP32 boot | ELRS | Add 10k pull-up to +3.3V_ELRS |
| W-2 | ESP32-C3 LNA_IN / FL2 RF path dangling (FH4 has internal antenna) | ELRS | Remove FL2 or connect to antenna |
| W-3 | SWCLK/SWDIO unconnected -- no debug or emergency recovery | MCU | Add test pads |
| W-4 | OSD sync hysteresis only ~3.3mV -- vulnerable to motor EMI | OSD | Decrease R25 to 10k |
| W-5 | D3 Schottky bypasses TPS2116 power mux (intentional?) | Power | Clarify intent, document or remove |
| W-6 | U6 EN tied to BATT, not GPIO11-controlled as labels suggest | Power | Route GPIO11 to U6 EN or update docs |
| W-7 | Shared input caps (C35/C37) between both buck converters | Power | Add dedicated 4.7uF per converter |
| W-8 | No TVS on battery input for motor-induced spikes | Power | Add SMBJ36A or similar |
| W-9 | D21 reverse polarity clamp only 100mA -- destroyed instantly by reversed LiPo | Power | Document limitation or add P-FET protection |
| W-10 | I2C pull-ups R42/R43 6.8k marginal for FM at >50pF bus capacitance | I2C | Reduce to 4.7k |
| W-11 | IMU Vdd_IO decoupling -- verify 100nF within 2-3mm of pin 5 on layout | IMU | PCB layout check |
| W-12 | SX1281 VR_PA cap C9=10nF may be undersized (datasheet recommends 470nF) | ELRS | Increase C9 to 470nF |
| W-13 | ELRX_TX label typo in root sheet (should be ELRS_TX) | ELRS | Rename label |
| W-14 | ADC_AVDD filter cap C71=4.7uF vs RP2350 recommended 100nF | MCU | Consider 100nF or add 100nF in parallel |
| W-15 | No OSD pin definitions in Betaflight config yet (GPIO32-37) | Config | Reserve pins when PIO OSD driver exists |
| W-16 | MCU_FLASH_SIZE=8MB in target.mk may be wrong for RP2354B (2MB internal) | Config | Update to 2048 if RP2354B-specific |
| W-17 | D5 LED current ~21mA borderline for 0603 (R29=470R from 12V) | Power | Increase R29 to 680R |

---

## SUGGESTION -- Nice to Have

| # | Suggestion | Subsystem |
|---|-----------|-----------|
| S-1 | Add 10k pull-up on IMU CS pin (GPIO14) for deterministic power-on state | IMU |
| S-2 | Add RUN pin test pad or reset button for field recovery | MCU |
| S-3 | USB series resistors 30R -> 27R for precise USB 2.0 compliance | USB |
| S-4 | Verify crystal X1 load capacitance spec matches 20pF load caps | MCU |
| S-5 | Add dedicated ELRS flashing header (EN, BOOT, TX, RX, 3.3V, GND) | ELRS |
| S-6 | Verify 100nF caps placed close to SX1281 VBAT pins (11, 15) on layout | ELRS |
| S-7 | Disable I2C/I3C in LSM6DSV16X firmware driver (add register write) | IMU/FW |
| S-8 | Verify IMU Vdd power sequencing (1.8V should rise before 3.3V Vdd_IO) | IMU |
| S-9 | Add VID_BUF test point on OSD output for amplifier characterization | OSD |
| S-10 | Add ESD protection (TVS array) on exposed UART/SBUS pads | Protection |
| S-11 | Reduce SBUS gate pull-down R17 from 100k to 47k for noise immunity | SBUS |
| S-12 | Add pull resistors on QSPI pins 70-74 to prevent floating at power-on | MCU |
| S-13 | Document DC-coupled video input requirement (no AC-coupled sources) | OSD |
| S-14 | SBUS invert circuit present but USE_SERIALRX_SBUS undefined in firmware | Config |

---

## Per-IC Summary

| IC | Status | Critical | Warnings | Confidence | Datasheet Source |
|----|--------|----------|----------|------------|-----------------|
| RP2354B (U36) | Pin mapping verified, power correct | 2 (LEDs, motor numbering) | 5 | HIGH | RP2350 HW design guide |
| LSM6DSV16XTR (U14) | SPI wiring correct, voltages correct | 0 | 1 (Vdd_IO cap) | HIGH | ST LSM6DSV16X DS |
| ESP32-C3FH4 (U2) | UART correct, boot control correct | 0 | 2 (LNA_IN, label) | HIGH | Espressif ESP32-C3 DS |
| SX1281IMLTRT (U20) | **VDD_IN disconnected** | 1 | 2 (NSS, VR_PA) | HIGH | Semtech SX1281 DS |
| BY25Q128ASWIG (U9) | SPI1 wiring correct | 0 | 0 | HIGH | Standard SPI flash |
| BMP388 (U30) | I2C correct, addr 0x76 | 0 | 0 | HIGH | Bosch BMP388 DS |
| LIS3MDLTR (U31) | DNP, I2C correct | 0 | 0 | HIGH | ST LIS3MDL DS |
| GS8052-FR (U37) | Confirmed dual op-amp, gain-of-2 correct | 0 | 1 (hysteresis) | HIGH | Gainsil GS8052 DS |
| PI3A223ZMEX (U11) | Mux wiring correct, BW adequate | 0 | 0 | HIGH | Diodes Inc DS |
| LMR51420 (U6, U7) | Vref=0.6V, dividers correct | 1 (C38 voltage) | 3 | HIGH | TI LMR51420 DS |
| TPS2116DRLR (U33) | PR1 divider correct | 0 | 1 (D3 bypass) | HIGH | TI TPS2116 DS |
| TPS746-3.3DRV (U13) | 5V to 3.3V, adequate | 0 | 0 | HIGH | TI TPS746 DS |
| WL9005D4-33 (U12) | 5V to 3.3V_ELRS | 0 | 0 | MEDIUM | Willsemi DS |
| NCV8187 (U21) | 5V to 1.8V, PG to LED | 0 | 0 | HIGH | onsemi NCV8187 DS |
| TXU0101DCKR (U42) | Level shifter correct | 0 | 0 | HIGH | TI TXU0101 DS |
| USBLC6-2SC6 (D8) | USB ESD correct | 0 | 0 | HIGH | ST USBLC6 DS |

---

## Betaflight Config Cross-Reference

**File**: `src/config/configs/OPENFC_RP2350B/config.h`

All pin-to-peripheral assignments verified against netlist and RP2350B hardware pin function tables:
- SPI0 (IMU): SCK=PA18, SDI=PA20, SDO=PA19 -- all valid SPI0 pins
- SPI1 (Flash): SCK=PA42, SDI=PA44, SDO=PA43 -- all valid SPI1 pins for RP2350B
- UART0: TX=PA0, RX=PA1 -- valid UART0 pins
- UART1: TX=PA22, RX=PA21 -- valid UART1 pins, crossover to ESP32-C3 correct
- I2C0: SDA=PA16, SCL=PA17 -- valid (16%4=0 SDA, 17%4=1 SCL)
- ADC: VBAT=PA41(ch1), CURR=PA40(ch0), RSSI=PA45(ch5), EXT=PA47(ch7) -- all valid
- Motors: PA28-PA31 -- all in 16-31 window for PIO DSHOT

**PIO Block Allocation** (no conflicts):
- PIO0: DSHOT motors (GPIO28-31, 4 state machines)
- PIO1: PIO UARTs (GPIO2,3,26,27, 4 state machines)
- PIO2: LED strip (GPIO23, 1 SM) -- 3 SMs available for future OSD

---

## Action Priority

1. **Before ordering PCBs**: Fix CRIT-1 through CRIT-5
2. **Before assembly**: Fix W-1 through W-9
3. **During layout review**: Verify W-10, W-11
4. **During firmware development**: Address W-15, W-16, S-7, S-14
5. **For robustness**: Consider all SUGGESTION items
