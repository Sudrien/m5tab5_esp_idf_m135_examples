# M5Stack Tab5 — ESP-IDF Module GNSS (M135) example

Bringing up the M135's GNSS receiver, barometer, magnetometer and second IMU
with plain ESP-IDF. No M5Unified, no BSP. Reports position and 1PPS from the
NEO-M9N, pressure and altitude from the BMP280, raw field from the BMM150, and
both BMI270s — the module's and the Tab5's own — side by side.

Companion to
[m5tab5_esp_idf_usb_host_example](https://github.com/Sudrien/m5tab5_esp_idf_usb_host_example);
like that one, this never touches the panel, so the screen stays dark on
purpose. Everything goes to the console over USB-C.

Loose ends and half-finished probing live in
[Tab5_M135_Experiments](https://github.com/Sudrien/Tab5_M135_Experiments). This
repo is the part that works.

## Why this exists

The M5-Bus 5V pin has no power at boot, and nothing says so.

### EXT5V_EN is not a GPIO

Same shape of bug as `USB5V_EN` in the USB-A example, different expander pin.
`EXT_5V_BUS` sits on the **first PI4IOE5V6408, at I2C `0x43`, pin `P2`**, and it
gates the 5V rail feeding the rear M5-Bus, the side 2.54-10P header, and the
HY2.0-4P port all at once.

Until P2 is driven high the module is electrically dead. The I2C bus is fine,
the UART driver installs, the scan runs clean — and the M135 is simply not on
it. The failure mode is indistinguishable from a module that is not seated,
which is where the debugging time goes, because reseating it is the obvious
thing to try and it does not help.

The high-impedance half applies here too: `OUT_SET` (`0x05`) sets the value,
`OUT_H_IM` (`0x07`) decides whether the pin drives it at all. Writing only
`OUT_SET` leaves P2 floating and the rail off, with exactly the symptom above.
Both registers, in that order, after `IO_DIR`.

Note this is the *other* expander from the USB one. `0x43` carries EXT5V_EN,
LCD_RST and TP_RST; `0x44` carries USB5V_EN and the charger controls. Get them
backwards and you turn on the USB-A port while wondering why the module is
still dark.

### The module shares the Tab5's internal I2C bus

M5-Bus pins 17 and 18 are `G31`/`G32`, which are not a spare bus — they are
*the* bus. The touch controller, the RTC, the audio codec, the power monitor,
both I/O expanders and the Tab5's own IMU are all on the same two wires as the
M135's sensors.

Two consequences. First, if anything else in your firmware owns that bus
(M5Unified does, the BSP does), you cannot also drive it with a raw
`i2c_master` handle; pick one owner. This example is the owner because nothing
else here is running.

Second, the address map is shared, so read it before assuming a scan result:

| Address | Part | Where |
| ------- | ---- | ----- |
| `0x10` | ES8388 codec | Tab5 |
| `0x32` | RX8130CE RTC | Tab5 |
| `0x40` | ES7210 mic ADC | Tab5 |
| `0x41` | INA226 | Tab5 |
| `0x43` | PI4IOE5V6408 #1 | Tab5 |
| `0x44` | PI4IOE5V6408 #2 | Tab5 |
| `0x55` | ST7123/ST7121 touch | Tab5 |
| `0x68` | **BMI270** | Tab5 |
| `0x69` | **BMI270** | M135 |
| `0x76` | BMP280 | M135 |

Two further addresses sometimes answer on a real Tab5 that are not in M5's
published table: `0x28` and `0x36`. They are logged as unknown rather than
guessed at.

*Sometimes* is the interesting part. Across boots of an identical binary the
scan reports either 12 devices or 9 — and the three that come and go are
`0x28`, `0x36` and `0x55`, the last being the ST7123/ST7121 touch controller.
That they appear and vanish together points at the display/touch IC: this
example never initialises the panel, so whether those addresses respond
depends on what state the previous firmware left the display in across a warm
reset. A cold power cycle is the way to get a repeatable scan.

Worth knowing before you spend time on it, because a bus scan that changes
between runs of the same image otherwise reads as a hardware fault.

`0x68` and `0x69` are both real and both answer. The M135's DIP switches can
move its BMI270 between the two, and if you land it on `0x68` you get one
device that appears to work and is actually two chips fighting over the same
address. If your gyro readings are erratic in a way that looks like noise but
does not scale with motion, check the DIP before the mounting.

### The magnetometer is not on the bus at all, and needs Bosch's driver

The BMM150 hangs off the BMI270's **auxiliary I2C master**, not the host bus.
It never shows up in a scan, no matter how correct everything else is. The
schematic confirms the topology: its SDI/SCK go to the BMI270's `ASDX`/`ASCX`
pins as `BMM_SDA`/`BMM_SCL`, it is powered directly from `+3.3V` with no gate,
and `SDO` is tied low, fixing the address at `0x10`.

Reaching it is the hard part, and this example does not attempt it by hand.
The aux master is not a bus you can drive by writing the obvious registers in
the obvious order:

- The write primitive loads `AUX_WR_DATA` **before** `AUX_WR_ADDR`, because
  writing the address is what triggers the transaction. Address-then-data
  sends whatever byte the previous call left behind.
- An `aux_busy` bit in `STATUS` has to be polled between operations.
- Advanced power save has to be dropped around each transfer and restored
  after.

A hand-rolled sequence here failed with `aux_err` (`ERR_REG` bit 7) on every
single transaction, through several rounds of debugging that ruled out
timing, the address shift, the power-control ordering, and the internal
pull-up selection in `AUX_IF_TRIM`. None of the failures pointed at the cause.
M5's own library wraps Bosch's API rather than doing this by hand, and the
one M5 example that reads the magnetometer has a bug that prints the
accelerometer values instead — so it was probably never watched working.

So: vendor the reference drivers.

```
./tools/fetch_bosch_drivers.sh
idf.py build flash monitor
```

That pulls `bmi2.c`, `bmi270.c` and `bmm150.c` into
`components/bosch_sensortec/`, and `main/bosch_aux.c` is a thin glue layer
over them. Both are BSD-3-Clause and gitignored, for the same reason as the
config blob: byte-exact fetches from pinned upstreams under a different
licence to this repo.

Accelerometer and gyroscope work without any of this. Skip the script and you
get everything except the magnetometer, with a message saying so rather than
a silent failure.

Two consequences worth knowing. The Bosch driver reconfigures the M135's
BMI270 ranges during magnetometer bring-up, so `imu_example.c` re-reads
`ACC_RANGE` and `GYR_RANGE` afterwards and derives its scale factors from the
hardware — assuming the range you set earlier silently halves every reading.
And once the drivers are vendored, `tools/fetch_bmi270_config.sh` is
redundant: the 8 KB config image ships inside `bmi270.c`.

### The BMI270 needs an 8 KB firmware upload before it does anything

There is no usable image in ROM. Fresh out of reset the BMI270 will return its
chip ID (`0x24`) and cheerfully ignore every other request: accelerometer and
gyroscope read as zero, the aux interface does not work, and no register
reports an error. It looks like a wiring problem and is not one.

The blob lives in Bosch's reference driver, is BSD-3-Clause, and is 8192 bytes:

```
./tools/fetch_bmi270_config.sh
idf.py build flash monitor
```

That pulls `bmi270.c` from `boschsensortec/BMI270-Sensor-API`, extracts
`bmi270_config_file[]`, and writes `main/bmi270_config_file.h`. The extraction
matches on hex bytes rather than on upstream's line formatting, and refuses to
write anything under 4096 bytes or of odd length, because a partially-matched
array produces a header that compiles and a sensor that stays dead.

`imu_example.c` guards on `__has_include`, so the project builds without the
header — it just logs the reason and reports chip IDs only. That is deliberate:
a fresh clone should build, not fail with a missing include and no explanation.

`./tools/fetch_bmi270_config.sh --revert` deletes the generated header.

Unlike `components/fatfs/` in the USB-A example, this one is **gitignored**.
It is a byte-exact fetch from a pinned upstream rather than a local patch, and
it carries Bosch's licence header rather than this repo's, so regenerating is
cleaner than committing. The script deletes `build/` for the same reason the
exFAT script does — a stale build directory reuses old objects and the sensors
stay dead with a perfectly good header sitting right there.

The upload itself is word-addressed: `INIT_ADDR_0` takes the low nibble of the
*word* offset and `INIT_ADDR_1` the next eight bits. Feed it a byte offset and
the image lands scrambled, `INTERNAL_STATUS` never reaches `0x01`, and nothing
tells you which of the several plausible causes it was.

### GNSS UART pins depend on how the DIPs are set

The M135 routes `NEO_TXD` and `NEO_RXD` to several M5-Bus pins under DIP
switch control, because it predates the Tab5 and has to suit Basic, Core2 and
CoreS3 as well. Cross-referencing the two pinmaps, the options on a Tab5 are:

| M5-Bus pin | M135 signal | Tab5 GPIO |
| ---------- | ----------- | --------- |
| 15 | NEO_TXD | `G7` |
| 16 | NEO_RXD | `G6` |
| 21 | NEO_RXD | `G2` |
| 22 | NEO_TXD | `G48` |
| 23 | NEO_RXD | `G47` |
| 24 | NEO_RXD / PPS | `G35` |
| 2 | NEO_TXD / PPS | `G16` |
| 26 | NEO_TXD / PPS | `G51` |

This example expects **pins 15/16**, so `G7` is the P4's RX and `G6` its TX.
Change the two defines at the top of `gnss_example.c` if your switches are set
differently. The Tab5 is not on the module's silkscreen table, so there is
nothing on the board to copy from.

### PPS is one of three pins and the board will not tell you which

The PPS DIP has three positions, landing on M5-Bus pins **2, 24 and 26** —
`G16`, `G35` and `G51` on a Tab5. That set is confirmed two independent ways:
the module's own bus table lists PPS on exactly those three pins, and M5's
[Core-series pinmap](https://docs.m5stack.com/en/learn/interface/mbus) maps
them to `G35`/`G0`/`G34` on a Basic and `G10`/`G0`/`G14` on a CoreS3 —
precisely the pin triples the module's DIP legend prints for those two
controllers.

What none of that gives you is which switch position is which, for a
controller the legend does not cover, and there is no way to read a DIP
position from software. It is a per-board constant you confirm once.

On the board this was developed against it is **`G51`, M5-Bus pin 26**:

```
tab5_gnss: watching 1PPS on G51 (M5-Bus pin 26)
tab5_gnss: PPS 30 pulses, last interval 1000023 us (+23 us from 1 s)
```

If yours is silent while the receiver has a good fix, the switch is on one of
the other two. Arm an interrupt on all three at once, see which ticks, and set
`GNSS_PPS_PIN` to it — that takes one boot and beats reasoning about switch
orientation. Use pull-downs when you do: two of the three are floating by
definition, and a floating input with an edge interrupt counts noise
indefinitely and produces a convincing second candidate.

Note also that all three PPS pins are shared with `NEO_TXD` or `NEO_RXD` — one
DIP position or the other, not both. Route the data line to a PPS pin and you
do not get PPS at all, and the silent message every ten seconds is correct
rather than a fault.

## What it reports

At boot: a bus scan, annotated, so you can tell Tab5 silicon from module
silicon at a glance.

| | |
| --- | --- |
| GNSS | fix quality, satellite count, HDOP, position, altitude and geoid separation, UTC time, ground speed and course |
| 1PPS | pulse count and measured interval against the nominal second |
| BMP280 | pressure, temperature, and altitude against a sea-level reference |
| BMI270 x2 | acceleration in g and rotation in dps, module and onboard on adjacent lines |
| BMM150 | raw field counts through the aux interface |

NMEA parsing matches on the sentence type and ignores the talker ID, since
a multi-constellation fix emits `$GNGGA` rather than `$GPGGA` and matching the
whole prefix silently drops everything once a second satellite system locks.

## Build

```
idf.py build flash monitor
```

`sdkconfig.defaults` pins `CONFIG_IDF_TARGET="esp32p4"`, so no
`idf.py set-target` is needed and deleting `sdkconfig` cannot silently drop you
back to the default `esp32` target and a toolchain error about
`xtensa-esp32-elf-gcc`.

Switching IDF versions? Use a fresh shell and wipe the generated config:

```
rm -rf build sdkconfig managed_components dependencies.lock
```

## Expected output

```
tab5_bus: internal I2C up on SDA=G31 SCL=G32 @ 400000 Hz
tab5_bus: M5-Bus 5V on (expander 0x43, P2)
tab5_bus: scanning internal I2C bus
tab5_bus:   0x10  ES8388 codec (Tab5)
tab5_bus:   0x32  RX8130CE RTC (Tab5)
tab5_bus:   0x40  ES7210 mic ADC (Tab5)
tab5_bus:   0x41  INA226 power monitor (Tab5)
tab5_bus:   0x43  PI4IOE5V6408 #1 (Tab5)
tab5_bus:   0x44  PI4IOE5V6408 #2 (Tab5)
tab5_bus:   0x68  BMI270 (Tab5 onboard IMU)
tab5_bus:   0x69  BMI270 (M135 module IMU)
tab5_bus:   0x76  BMP280 (M135 barometer)
tab5_bus: 9 devices
tab5_bus:   (BMM150 is behind the BMI270's aux bus and never appears here)
tab5_gnss: NEO-M9N on UART1, RX=G7 TX=G6 @ 38400 baud
tab5_gnss: watching 1PPS on G51 (M5-Bus pin 26)
tab5_baro: BMP280 at 0x76, chip ID 0x58
tab5_imu: Tab5: BMI270 at 0x68
tab5_imu: Tab5: config uploaded, internal status ok
tab5_imu: Tab5: scale +/-4 g, +/-2000 dps
tab5_imu: M135: BMI270 at 0x69
tab5_imu: M135: config uploaded, internal status ok
tab5_imu: M135: scale +/-4 g, +/-2000 dps
bosch_aux: bmm150_init ok, chip ID 0x32
tab5_imu: M135: scale +/-8 g, +/-2000 dps
tab5_imu: BMM150 at aux 0x10 behind the M135 BMI270
```

With a clear sky:

```
tab5_gnss: fix          GPS, 12 satellites used, HDOP 0.83
tab5_gnss:   position   42.338458, -83.470656
tab5_gnss:   altitude   220.8 m MSL, geoid sep -34.7 m
tab5_gnss:   UTC        00:06:43 on 2026-08-16
tab5_gnss:   speed      0.560 kn, course - deg
tab5_gnss: PPS 30 pulses, last interval 1000023 us (+23 us from 1 s)
tab5_baro: 994.87 hPa, 29.76 C, 154.2 m (vs 1013.25 hPa reference)
tab5_imu: ---
tab5_imu:   Tab5  acc  -0.019  -0.135  -0.974 g   gyr    -0.12    +0.06    +0.06 dps
tab5_imu:   M135  acc  +0.141  -0.016  -1.024 g   gyr    -0.24    -0.24    +0.18 dps
tab5_imu:   mag     +29.00  +78.00  -35.00 uT  (|B| 90.3)
```

Indoors, where the fix is marginal or absent:

```
tab5_gnss: no fix, 12 satellites used
tab5_gnss: PPS silent on G51 (no time fix yet, or DIP on another pin)
```

Without the BMI270 config file, which is the state of a fresh clone:

```
W tab5_imu: built without bmi270_config_file.h
W tab5_imu: run ./tools/fetch_bmi270_config.sh, then rebuild
W tab5_imu: M135: no config file, sensor stays in suspend (see README)
```

Cold start with no backup battery charge can take several minutes even with a
clear sky. The M135's coin cell holds ephemeris across power cycles, so the
second start is much faster than the first — if the first fix is quick and
every subsequent one is slow, suspect the cell rather than the antenna.

## Design notes

The two BMI270s are driven through one struct instantiated twice rather than
two drivers. They are the same part with different address straps, and the
redundancy is the interesting bit: two accelerometers on rigidly coupled PCBs
should agree, and the size of the disagreement is a usable measure of how much
to trust either. The onboard one is treated as optional — failing to open it
logs a warning and the module continues.

The BMM150 output is compensated microtesla, because Bosch's driver applies
the factory trim values from the sensor's NVM. Earth's field runs 25-65 uT
depending on latitude, so the magnitude is a quick sanity check: far outside
that range means something nearby is magnetised. On a Tab5 the most likely
culprit is the speaker, a few centimetres from the module. That offset is
fixed in sensor frame and can be calibrated out by rotating the assembly
through as many orientations as you can manage and taking the midpoint of
each axis; the field from the voice coil during playback cannot, since it
varies with the audio.

A marginal fix produces a confident, stable, badly wrong altitude. Indoors
this example reported 1885 m for minutes at a stretch, drifting smoothly, at a
location near 270 m. Nothing about the output flags it: the sentence parses
correctly, the geoid separation is right for the area, horizontal position is
within a few metres, and the number is steady enough to look measured rather
than noisy. Carrying the same board outdoors dropped it to 259 m on the first
epoch and settled near 240 m.

The tell is in GSA and GSV rather than GGA. Indoors the fix ran on five
satellites with VDOP near 4.0 and almost every GSV entry showing a blank SNR
field — tracked but too weak to use. Outdoors it was twelve satellites at
35–48 dB with VDOP under 1.0. Vertical error is always the worst component,
and multipath off a roof inflates it far beyond what the DOP figure alone
suggests.

Do not trust GNSS altitude without checking the satellite count and VDOP
first, and do not conclude a parser is broken because the number is stable.

Altitude from the BMP280 is against a hardcoded 1013.25 hPa standard
atmosphere, not your local QNH, so treat it as relative unless you feed it a
real reference. The IIR filter is set to x16 — without it the altitude jitters
by several metres and reads as a broken sensor.

The GNSS parser handles GGA and RMC and ignores the rest. GSV would give
per-satellite signal strength and GSA the fix mode and DOP breakdown, both of
which are more useful for diagnosing a bad sky view than anything here — the
sentence splitter is the reusable part, and adding a handler is a dozen lines.
Build with `-DGNSS_LOG_RAW` to dump every accepted sentence verbatim, which is
the fastest way to settle whether a suspicious field is a parsing error or the
receiver's own opinion.

Speed, course and date come from RMC but are printed by the GGA handler. The
M9N emits RMC first within each epoch, so printing it on arrival puts the
speed above the fix it belongs to, one line out of step forever.

Empty NMEA fields are preserved as empty strings rather than skipped. A
receiver with no fix signals it by leaving fields blank, not by omitting them,
so a splitter that collapses runs of commas shifts every subsequent field by
one and produces confident nonsense.

Nothing here configures the NEO-M9N. It runs at its factory defaults, which is
NMEA at 1 Hz. The update rate goes to 25 Hz and the output can be switched to
UBX, both via `UBX-CFG-VALSET` on the same UART — that is a natural next thing
and is not in this example.

## Licence

MIT, see `LICENSE`. The BMI270 configuration image and the BMI270/BMM150
reference drivers, fetched by `tools/fetch_bmi270_config.sh` and
`tools/fetch_bosch_drivers.sh`, are Bosch's, BSD-3-Clause, used unmodified.
