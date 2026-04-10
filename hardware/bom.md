# Bill of Materials (BOM)

Hardware components used to build the STM32-based Digital Differential Relay prototype.

| Component | Specification | Qty | Purpose |
|---|---|---|---|
| STM32F411CEU6 Black Pill | ARM Cortex-M4 @ 100 MHz | 1 | Main MCU |
| ST-LINK V2 | USB Programmer | 1 | Firmware upload |
| CT MSQ-0.66 | 30A/5A, Class 0.5, Solid-Core | 6 | 3-phase current sensing (primary + secondary) |
| OLED 1.3" SH1106 | I2C 0x3C, 128×64 px | 1 | Display I_diff, I_bias, and fault status |
| Buzzer MH-FMD | Active 3.3–5V, connected to PB0 | 1 | Audible fault alarm |
| R 0.1 Ω, 2W, 5% | Burden resistor | 6 | CT burden (R_burden) |
| R 10 kΩ, Metal Film, 1/4W, 1% | Voltage divider | 12 | DC bias 1.65 V per channel |
| C 10 µF, 16V, Electrolytic | Bypass capacitor | 6 | Power supply decoupling |
| C 100 nF, Ceramic (104) | Filter capacitor | 6 | High-frequency noise suppression |
| Zener 3.3V, 0.5W | Clamp diode | 6 | ADC over-voltage protection |

## Signal Conditioning Circuit (per channel)

Each of the 6 CT channels is conditioned as follows:

```
CT secondary → R_burden (0.1 Ω) → Voltage divider (DC bias 1.65 V) → Zener clamp (3.3 V) → STM32 ADC pin
```

- **CT ratio:** 30A/5A → `CT_RATIO = (30/5) / 0.1 = 60.0 A/V`
- **ADC resolution:** 12-bit (0–4095), Vref = 3.3 V → 1 LSB ≈ 0.806 mV
- **DC bias:** 1.65 V mid-rail so that the AC waveform swings symmetrically within 0–3.3 V

## Pin Assignment (STM32F411 Black Pill)

| Pin | Signal |
|---|---|
| PA0 | Primary current — Phase A |
| PA1 | Secondary current — Phase A |
| PA2 | Primary current — Phase B |
| PA3 | Secondary current — Phase B |
| PA4 | Primary current — Phase C |
| PA5 | Secondary current — Phase C |
| PB12 | Page button (INPUT_PULLUP) |
| PB0 | Buzzer (active LOW) |
| PB1 | Fault LED — Phase A |
| PB3 | Fault LED — Phase B |
| PB4 | Fault LED — Phase C |
| PB6/PB7 | I2C SDA/SCL (OLED SH1106) |
