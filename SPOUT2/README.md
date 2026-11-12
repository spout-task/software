# SPOUT2, two-spout software

- `teensy_firmware/teensy2spout/teensy2spout.ino`, two-spout state-machine firmware.
- `matlab_GUI/SPOUT2.mlapp`, MATLAB App Designer GUI (`AppArduinoConnection.m` handles serial).

**Tasks:** uninstructed LL/LR, instructed LL/LR, two-armed bandit (2ABT), delayed-response (DR), Pavlovian conditioning.
Predefined settings: [settings repo, SPOUT2](https://github.com/spout-task/settings/tree/main/SPOUT2).

## Pins and options in `teensy2spout.ino`

All pins are set at the top of the sketch. To change a pin, edit its number and upload the sketch again.
The board connectors are explained in the
[Teensy breakout board README](https://github.com/spout-task/hardware/tree/main/teensy_breakout_board).

| Pin | Variable | Function | Default use in SPOUT2 |
|---|---|---|---|
| 2 | `SpeakerPinL` | Left speaker | Used |
| 15 | `SpeakerPinR` | Right speaker | Used |
| 23 | `StartCueSpeakerPin` | Dedicated start (go) cue speaker | Tasks with a start cue, such as delayed response |
| 3 | `SyncPin` | Pseudo random sync pulses for alignment with other systems | Used |
| 22 | `ExtSyncPin` | External sync input (3.3 V maximum), timestamped by the Teensy | Optional |
| 14 | `LeftLickPin` | Left lick input from the lick detector | Used |
| 41 | `RightLickPin` | Right lick input from the lick detector | Used |
| 5 | `LeftRewardPin` | Left solenoid (SLN1) | Used |
| 4 | `RightRewardPin` | Right solenoid (SLN2) | Used |
| 17 | `LeftCueIndicatorPin` | TTL copy of the left tone | Optional |
| 18 | `RightCueIndicatorPin` | TTL copy of the right tone | Optional |
| 16 | `StartCueIndicatorPin` | TTL copy of the start cue | Tasks with a start cue |
| 6 | `SpeakerTTLPin` | TTL copy of both speakers | Optional |
| 19 | `LeftRewardIndicatorPin` | TTL copy of the left solenoid | Optional |
| 20 | `RightRewardIndicatorPin` | TTL copy of the right solenoid | Optional |
| 8 | `ITIIndicatorPin` | TTL during the inter trial interval | Optional |
| 24 | `DelayIndicatorPin` | TTL during the delay period | Delayed response task |
| 10 | `StartStopIndicatorPin` | Short pulse at session start and stop | Optional |
| 40 | `OptoPin` | Optogenetics TTL (light on or off) | Used with optogenetics |
| 1 | `OptoTTLPin` | Copy of the optogenetics TTL (reporter LED) | Optional |
| 9 | `OptoModPin` | PWM for the [optogenetics power modulator](https://github.com/spout-task/hardware/tree/main/optogenetics_power_modulator) | Used with power modulation |
| 21 | `RandomLightPin` | Random background light | Optional |
| 7 | `TrialStartPin` | TTL at trial start | Optional |
| 11 | `CameraTriggerPin` | Camera trigger (2 ms pulses at 50 Hz) | Optional |
| 13 | `ErrorLightPin` | Error light after a wrong lick | Optional |
| 12 | `ExtSyncReportPin` | Mirrors the external sync input on an LED | Optional |
| 39 | `StartStopButtonPin` | Hardware start and stop button | Only if `buttonsAreWiredUp` is `true` |
| 38 | `LeftRewardButtonPin` | Manual left reward button | Only if `buttonsAreWiredUp` is `true` |
| 37 | `RightRewardButtonPin` | Manual right reward button | Only if `buttonsAreWiredUp` is `true` |

Other options at the top of the sketch:

* `SpoutNotButton`: `true` for spouts with the lick detector (a lick is a LOW input), `false` to test with push buttons (a press is a HIGH input). Keep `true` for experiments.
* `buttonsAreWiredUp`: set to `true` only if hardware buttons are connected to pins 37, 38 and 39.
