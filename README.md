# ESPHome RS485 gateway for Ambientika ADVANCED+ / ADVANCED B+

Local Home Assistant control for **Ambientika ADVANCED+** and **ADVANCED B+** decentralised ventilation units with heat recovery, via their RS485 interface. Runs on an **M5Stack Atom Lite** with the **M5Stack Atomic RS485 Base**. No cloud, no extra gateway, about 20 to 25 euros of hardware.

Provided by SÜDWIND GmbH, Eppan (Italy), manufacturer of [Ambientika](https://www.ambientika.eu). Community solution, see [Disclaimer](#disclaimer).

> Using an **Ambientika SMART** (Wi-Fi)? Use the Home Assistant integration instead: [HomeAssistant-integration-for-Ambientika](https://github.com/ambientika-eu/HomeAssistant-integration-for-Ambientika).

**Application note AN-2026-04 (PDF):** [Deutsch](docs/Ambientika_AN-2026-04_HomeAssistant_ESPHome_AtomLite_DE.pdf) · [Italiano](docs/Ambientika_AN-2026-04_HomeAssistant_ESPHome_AtomLite_IT.pdf) · [English](docs/Ambientika_AN-2026-04_HomeAssistant_ESPHome_AtomLite_EN.pdf)

## What you get in Home Assistant

| Entity | Function |
|---|---|
| Ambientika Betriebsart | Heat recovery, extract, supply, all extract, all supply, motor pause, off (flap closed) |
| Ambientika Geschwindigkeit | Night, level 1 to 3 |
| Ambientika Feuchtestufe | Level 1 manual; levels 2 and 3 automatic (the unit's humidity sensor starts dehumidification itself) |
| Ambientika Boost 20 min / Boost beenden | Extract level 3 for 20 minutes, then back to the previous mode |
| Ambientika Filter zuruecksetzen | Reset the filter indicator |
| Ambientika Daemmerung, Filteralarm, Feuchtealarm | Status of the unit (polled every minute) |
| Ambientika Statusantwort | Connection to the unit |

Entity names are German; rename them in Home Assistant as you like. The button on the Atom toggles the boost. The Atom LED shows green (unit answers), blue (boost) or red (no answer).

Firmware default: heat recovery, humidity level 2, day. This matches the usual AUTO operation of the ADVANCED B+ in a bathroom.

## Hardware

- M5Stack Atom Lite (ESP32)
- M5Stack Atomic RS485 Base (A131)
- USB-C power supply 5 V
- 120 ohm resistor
- Twisted, shielded cable, e.g. J-Y(St)Y 2×2×0.8 mm

## Wiring

| Atomic RS485 Base | Ambientika terminal |
|---|---|
| A | 4 (A) |
| B | 3 (B) |
| GND | 5 (GND) |

- 120 ohm between terminals 3 and 4.
- Power the Atom Lite via USB-C (5 V). Leave the 12 V input of the base unused: the unit terminals carry no supply voltage.
- Do not run the bus and 230 V in the same cable.
- Set the unit to **master with wall panel (DIP1 ON, DIP2 ON)** and power-cycle it. No wall panel on the same line. One gateway per master.

Pin note: the ESP32 uses **TX = GPIO19, RX = GPIO22**. The table in the M5Stack datasheet lists the pins from the base's point of view.

## Installation

1. In Home Assistant, install the **ESPHome Device Builder** app and create a new device.
2. Replace its configuration with [`ambientika-atom-lite.yaml`](ambientika-atom-lite.yaml).
3. Create `secrets.yaml` from [`secrets.example.yaml`](secrets.example.yaml): Wi-Fi, access point password, API key (32 bytes, Base64, e.g. `openssl rand -base64 32`), web interface password.
4. Flash via USB the first time, then over Wi-Fi.
5. Add the discovered device "Ambientika Gateway" in Home Assistant with the API key.

About 10 seconds after start-up the LED turns green.

## Protection of the drive

The firmware sends the mode command every 500 ms as the protocol requires and holds **at least 15 seconds of motor pause** before every change of direction, before switching off and after every restart. Please do not change `pause_ms` (a compile-time check enforces at least 15 s). Before an over-the-air update, select *Motor Pause*.

## Tests

The control logic is tested in a C++ simulation that takes the lambdas straight from the YAML (10.2 million checkpoints, including fuzzing with timing jitter, restarts and `millis()` overflow) and a mutation test that must detect 17 deliberate faults:

```sh
cd tests
pip install pyyaml
python3 gen.py ../ambientika-atom-lite.yaml sim_gen.h
g++ -std=c++17 -O2 -o harness harness.cpp && ./harness
./mutate.sh
```

## Deutsch

Fertiges ESPHome-Gateway, um ADVANCED+ und ADVANCED B+ über RS485 lokal in Home Assistant einzubinden, mit M5Stack Atom Lite und Atomic RS485 Base. Anleitung: [AN-2026-04 (PDF)](docs/Ambientika_AN-2026-04_HomeAssistant_ESPHome_AtomLite_DE.pdf).

## Italiano

Gateway ESPHome pronto per integrare ADVANCED+ e ADVANCED B+ in Home Assistant via RS485, in locale, con M5Stack Atom Lite e Atomic RS485 Base. Istruzioni: [AN-2026-04 (PDF)](docs/Ambientika_AN-2026-04_HomeAssistant_ESPHome_AtomLite_IT.pdf).

## Disclaimer

This is a community solution provided without warranty. The M5Stack components are not Ambientika accessories and are not supplied by SÜDWIND. Work on the 230 V installation must be carried out by a qualified electrician. SÜDWIND accepts no liability for damage resulting from its use. No intervention in the ventilation unit is needed or permitted; the unit terminals are used as intended. Questions about the firmware: please open an issue in this repository.

## License

[MIT](LICENSE) © 2026 SÜDWIND GmbH
