# Differential Relay for Three-Phase Transformer Protection

A low-cost digital differential relay prototype built on the **STM32F411 (Black Pill)** microcontroller, designed to detect **Inter-Turn Faults (ITF)** in three-phase power transformers. Developed as a senior project for course 01026323 Power System Protection, KMITL, Semester 2/2025.

---

## Overview

Conventional overcurrent relays often fail to detect inter-turn faults in their early stages because the secondary load current does not rise significantly. This project implements a **Dual-Slope Percentage Bias Differential Protection** algorithm that compares the primary and secondary currents of each phase, tripping when the differential current exceeds a bias-dependent threshold.

**Key results:**
- Detection rate: **100%** across 112 recorded fault signals
- False positive rate: **0%**
- Algorithm verified under worst-case hardware noise conditions

---

## Repository Structure

```
differential-relay-stm32/
├── matlab/
│   ├── script_1_plot_selected_signal.m          # Plot healthy/fault signal regions
│   ├── script_2_bias_characteristic.m           # Bias scatter plot + Grid Search
│   ├── script_3_simulate_relay.m                # Simulate firmware v9 on CSV data
│   └── script_4_simulate_relay_worstcase_noise.m # Worst-case noise simulation
│
├── firmware/
│   └── STM32_Differential_Relay_V9/
│       └── STM32_Differential_Relay_V9.ino      # Arduino (STM32duino) firmware
│
├── data/
│   └── Selected_Signals/                        # 112 CSV files from Hioki PQ3198
│       ├── Signal_G01/
│       ├── Signal_G02/
│       └── ...  (G01–G15, ~70 MB total)
│
├── docs/
│   └── Differential_Relay_Report.pdf            # Full project report (Thai)
│
└── hardware/
    └── bom.md                                   # Bill of Materials + pin assignment
```

---

## Hardware

| Component | Specification |
|---|---|
| MCU | STM32F411CEU6 Black Pill (ARM Cortex-M4 @ 100 MHz) |
| Current Transformers | MSQ-0.66, 30A/5A, Class 0.5 × 6 |
| Display | OLED 1.3" SH1106 (I2C) |
| Alarm | Active Buzzer MH-FMD |
| Burden resistor | 0.1 Ω / 2W per channel |

See [`hardware/bom.md`](hardware/bom.md) for the full BOM and pin assignments.

---

## Protection Algorithm

The firmware uses a **Dual-Slope Percentage Bias** characteristic:

```
I_diff = |I_pri_comp - I_sec|
I_bias = (I_pri_comp + I_sec) / 2
I_pri_comp = I_pri_rms / EFFECTIVE_RATIO

Threshold:
  if I_bias < KNEE_POINT:  thr = SLOPE1 × I_bias + PICKUP
  else:                    thr = SLOPE2 × I_bias - OFFSET2

Trip if I_diff > thr
```

**Tuned parameters (from Grid Search on 112 signals):**

| Parameter | Value |
|---|---|
| PICKUP | 0.45 A |
| SLOPE1 | 0.10 |
| SLOPE2 | 0.50 |
| KNEE_POINT | 5.0 A |
| OFFSET2 | 1.55 A |

`EFFECTIVE_RATIO` is computed automatically at boot from healthy-state measurements, making the relay configuration-agnostic (works for Yy, Yd, Dy, Dd transformers without any hardcoded turn ratio).

---

## Dataset

Signals were recorded with a **Hioki PQ3198 Power Quality Analyzer** at **20 kHz** sampling rate from a real three-phase transformer with simulated inter-turn faults.

| Group | Description |
|---|---|
| G01–G05 | Various fault tap levels, Phase A |
| G07–G08 | Phase B fault conditions |
| G10–G12 | Phase C fault conditions |
| G13–G15 | Mixed conditions including Dy configuration |

Each CSV file contains: `EventNo, Date, Time, U1, U2, U3, U4, I1, I2, I3, I4`
- Column 8 (`I1`) = Primary line current [A]
- Column 11 (`I4`) = Secondary line current [A]

---

## MATLAB Scripts

Run all scripts from the **project root** (the folder containing `Selected_Signals/` and `matlab/`).

### script_1 — Plot Signals
```matlab
cd matlab
script_1_plot_selected_signal
```
Generates annotated plots of each signal with Healthy/Fault regions highlighted. Output saved to `Signal_Plots/`.

### script_2 — Bias Characteristic + Grid Search
```matlab
script_2_bias_characteristic
```
1. Computes I_diff and I_bias for every 40 ms window across all files
2. Plots a scatter diagram (Healthy vs Fault) on the I_bias–I_diff plane
3. Runs a grid search over (PICKUP, SLOPE1, SLOPE2, KNEE_POINT) to find parameters that achieve FP = 0% and Detection = 100%

Output saved to `Simulation_Output/`.

### script_3 — Algorithm Simulation
```matlab
script_3_simulate_relay
```
Faithfully replicates firmware v9 logic on all 112 recorded signals. Reports per-file detection rate and false positive rate.

### script_4 — Worst-Case Noise Simulation
```matlab
script_4_simulate_relay_worstcase_noise
```
Injects worst-case hardware noise (ADC offset + CT Class 0.5 error) and re-evaluates the algorithm to verify safety margins are maintained.

---

## Firmware

Developed with **Arduino IDE** using the [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) core.

**Required libraries:**
- `Adafruit GFX Library`
- `Adafruit SH110X` (for SH1106 OLED)

**Upload steps:**
1. Install STM32duino core in Arduino IDE
2. Board: `Generic STM32F4 series` → `STM32F411CEUx (Black Pill)`
3. Upload method: `STLink`
4. Open `firmware/STM32_Differential_Relay_V9/STM32_Differential_Relay_V9.ino` and upload

**Boot sequence:**
1. **DC Offset Calibration** (3 s, no load required) — measures ADC mid-point per channel
2. **Effective Ratio Calibration** (10 s, load must be applied) — computes `EFFECTIVE_RATIO = I_pri_avg / I_sec_avg` from healthy-state current

After boot, the relay enters continuous protection mode, evaluating I_diff every 40 ms.

---

## Authors

Project team — KMITL Electrical Engineering, Semester 2/2025:

Project Team Leader : Pongpada Anoma
Team Members
Jiratchayaporn Sirimongkolchaikul
Chavalwit Ruammitsamak
Chotiwut Samranmak
Chanwit Kaewyai
Nadchaporn Sanlek
Natthaporn Duagchai
Nattaphol Seena
Nattawat Sirikururat
Thanatip Kaothaisong
Tanate Sooksri
Patchara Kunaporn
Rachata Pongsutti
Ratchata Ngiaophai
Wittawach Inkaew

Advisor: Assoc. Prof. Dr. Attaphol Ngaopitakkul

---

## License

This project is released for academic reference. The dataset and report are property of KMITL. Please cite appropriately if used in research.
