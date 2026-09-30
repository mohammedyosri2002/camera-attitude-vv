# Hardware

Machine-readable copy: [`config/hardware.json`](../config/hardware.json).

Values are quoted for identification. Consult the manufacturers' own datasheets for
authoritative specifications; datasheets are **not** redistributed in this repository.

## Camera

| | |
|---|---|
| Manufacturer / model | Basler AG, **ace acA1920-40um** |
| Interface | USB 3.0, monochrome |
| Sensor | Sony IMX249 |
| Acquisition resolution used | 1920 × 1200 |
| Pixel format | Mono8 |
| Exposure used | 5000 µs |
| Gain | 0 dB |
| Auto exposure / auto gain | **disabled** |

Intrinsics and distortion are recorded in **each run's own camera metadata JSON**, and the
reproduction pipeline reads them from there. `config/camera_intrinsics.json` is an
informational copy only — a run can never be reprocessed with intrinsics it was not
acquired with.

## Lens

| | |
|---|---|
| Manufacturer / model | Ricoh, **FL-CC0814A-2M** |
| Focal length | 8 mm, fixed |
| Mount | C-mount |
| Aperture range | F1.4–F16 |

## IMU

| | |
|---|---|
| Module | SparkFun **ISM330DHCX** |
| Host | Teensy 4.1 |
| Sensors | three-axis accelerometer + three-axis gyroscope |
| Logging | microSD, raw, **no on-board filtering** |
| Configured ODR | **208 Hz** |

**On sample cadence.** The cadence derived from the logged 64-bit microsecond timestamps
differs from the configured ODR. Where a cadence is reported anywhere in this repository or
the paper it is described as a **measured logging cadence**, never as the physical ODR of
the sensor. The configured ODR is 208 Hz; that is the value to quote as a device setting.

Teensy firmware was **unchanged for the entire campaign**
(`src/original/teensy41_imu_sd_logger_final.ino`).

## Motor

| | |
|---|---|
| Manufacturer / model | UIROBOT **UIM5756PM** |
| Control | STEP / DIR |
| Commanded resolution | **25 600 pulses per revolution** |
| Command resolution | 360 / 25 600 = **0.0140625 deg/pulse** |
| Round-to-nearest quantization | **±0.00703125 deg** |
| Wiring | DIR = D8, STEP = D9, ENABLE = D7 |

ENABLE is asserted for the whole session, so the motor holds position by torque between
phases — including at the commanded pre-position working point.

### Terminology, non-negotiable

The motor signal is a **pulse-derived commanded reference** (or **commanded reference**).

It is **not** ground truth, **not** an actual angle, and **not** a measured motor angle.
The drive may contain an internal closed loop, but **no independent encoder-position
telemetry was logged for this campaign**, so nothing in this dataset constitutes a
measurement of where the platform actually went. Where the platform's true position
matters, the independent accelerometer-derived gravity tilt is the cross-check — see
[`METRICS.md`](METRICS.md) and [`LIMITATIONS.md`](LIMITATIONS.md).
