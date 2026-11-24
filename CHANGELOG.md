# Changelog

## 1.1.0 (2026-11-24)

First public release.

- RS485 gateway for Ambientika ADVANCED+ / ADVANCED B+ on M5Stack Atom Lite + Atomic RS485 Base (TX GPIO19, RX GPIO22).
- Operating mode, speed, humidity level, 20-minute boost (Home Assistant buttons or the Atom button), filter reset.
- Status: twilight, filter alarm, humidity alarm, connection; status LED (green / blue / red).
- Drive protection: at least 15 s motor pause before every change of direction, before switching off and after every restart; 500 ms command cycle; gap detection.
- Secured by default: API encryption (also used for OTA), digest login for the web interface, no firmware upload via the web interface.
- Tested in a C++ simulation (10.2 million checkpoints) and by mutation testing (17/17 faults detected); verified on an ADVANCED B+.
