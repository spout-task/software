# SPOUT · software

Per-system control software for **SPOUT** — part of the [spout-task](https://github.com/spout-task) project. Split
into `SPOUT1/` (one spout) and `SPOUT2/` (two spouts); each contains its Teensy firmware
and its MATLAB GUI.

```
SPOUT1/  teensy_firmware/ (teensy1spout.ino)   matlab_GUI/ (SPOUT1.mlapp + AppArduinoConnection.m)
SPOUT2/  teensy_firmware/ (teensy2spout.ino)   matlab_GUI/ (SPOUT2.mlapp + AppArduinoConnection.m)
```

## Firmware
The Teensy runs the full task state machine (the host is never in the real-time loop). Open
the `.ino` in the Arduino IDE with **Teensyduino** and upload to the Teensy.

## MATLAB GUI
`SPOUTn.mlapp` (MATLAB App Designer) connects to the Teensy, loads a task settings file,
sets parameters, runs the session, and reports performance live. `AppArduinoConnection.m`
handles the serial link.

## Task modes
**SPOUT1:** go/no-go · stop-signal · fixed-ratio · progressive-ratio · Pavlovian conditioning
**SPOUT2:** uninstructed LL/LR · instructed LL/LR · two-armed bandit (2ABT) · delayed-response (DR) · Pavlovian conditioning

Predefined settings for each task live in the separate [settings](https://github.com/spout-task/settings) repository.

## Requirements
- **MATLAB** [version TODO] with App Designer. Toolboxes: [TODO].
- **Arduino IDE** with **Teensyduino** for flashing firmware.

## Related repositories
[hardware](https://github.com/spout-task/hardware) · [analyses](https://github.com/spout-task/analyses) · [settings](https://github.com/spout-task/settings) · [example_data](https://github.com/spout-task/example_data)

## License
**GPL-3.0-or-later** ([`LICENSE`](LICENSE)). See also [`CITATION.cff`](CITATION.cff).
