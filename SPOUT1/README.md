# SPOUT1, one-spout software

- `teensy_firmware/teensy1spout/teensy1spout.ino`, one-spout state-machine firmware.
- `matlab_GUI/SPOUT1.mlapp`, MATLAB App Designer GUI (`AppArduinoConnection.m` handles serial).

**Tasks:** go/no-go, stop-signal, fixed-ratio, progressive-ratio, Pavlovian conditioning.
Predefined settings: [settings repo, SPOUT1](https://github.com/spout-task/settings/tree/main/SPOUT1).

## Pins and options in `teensy1spout.ino`

All pins are set at the top of the sketch. To change a pin, edit its number and upload the sketch again.
A pin set to 66 is switched off (the Teensy 4.1 has no pin 66). The board connectors are explained in the
[Teensy breakout board README](https://github.com/spout-task/hardware/tree/main/teensy_breakout_board).

| Pin | Variable | Function | Default use in SPOUT1 |
|---|---|---|---|
| 2 | `SpeakerPinA` | Tone for cue A | Used |
| 15 | `SpeakerPinB` | Tone for cue B (second speaker) | Optional; the default rig plays both cues on one speaker |
| 3 | `SyncPin` | Pseudo random sync pulses for alignment with other systems | Used |
| 22 | `ExtSyncPin` | External sync input (3.3 V maximum), timestamped by the Teensy | Optional |
| 14 | `LickPin` | Lick input from the lick detector | Used |
| 5 | `RewardPin` | Solenoid (SLN1) | Used |
| 41 | `UnusedLickPin` | Kept so the same board works for SPOUT1 and SPOUT2 | Not used |
| 4 | `UnusedRewardPin` | Kept so the same board works for SPOUT1 and SPOUT2 | Not used |
| 17 | `CueAIndicatorPin` | TTL, high during cue A | Optional |
| 18 | `CueBIndicatorPin` | TTL, high during cue B | Optional |
| 6 | `SpeakerTTLPin` | TTL copy of both speakers | Optional |
| 19 | `RewardIndicatorPin` | TTL, high during reward | Optional |
| 20 | `UnusedRewardIndicatorPin` | Kept for SPOUT2 compatibility | Not used |
| 8 | `ITIIndicatorPin` | TTL during the inter trial interval | Optional |
| 10 | `StartStopIndicatorPin` | Pulse at session start and stop | Optional |
| 40 | `OptoPin` | Optogenetics TTL (light on or off) | Used with optogenetics |
| 1 | `OptoTTLPin` | Copy of the optogenetics TTL (reporter LED) | Optional |
| 9 | `OptoModPin` | PWM for the [optogenetics power modulator](https://github.com/spout-task/hardware/tree/main/optogenetics_power_modulator) | Used with power modulation |
| 21 | `RandomLightPin` | Random background light | Optional |
| 7 | `TrialStartPin` | TTL at trial start | Optional |
| 12 | `ExtSyncReportPin` | Mirrors the external sync input on an LED | Optional |
| 66 | `CameraTriggerPin` | Camera trigger (2 ms pulses at 50 Hz) | Off; set a free pin (for example 11, as in SPOUT2) to use it |
| 66 | `ErrorLightPin` | Error light | Off; set a free pin (for example 13) to use it |
| 66 | `StartStopButtonPin`, `RewardButtonPin` | Hardware buttons for start and stop, and manual reward | Off |

Other options at the top of the sketch:

* `SpoutNotButton`: `true` for spouts with the lick detector (a lick is a LOW input), `false` to test with push buttons (a press is a HIGH input). Keep `true` for experiments.
* `buttonsAreWiredUp`: set to `true` only if hardware buttons are connected to the button pins.
