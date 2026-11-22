# tof-tester

ESP-IDF firmware for an M5StickC Plus that reads up to four DFRobot SEN0628 8x8 matrix ToF sensors on the Grove port and streams each matrix out the USB serial console.

## What you see

The screen lists I2C addresses `0x30`–`0x33`:

- grey `-` means that address did not ACK
- green `+` means a sensor is on the bus
- yellow `*` is the sensor whose 8x8 distances are drawn (near = yellow, far = blue, no return = dark)

The large number is the center distance in millimetres. Button A cycles the drawn sensor. Button B rescans the bus. A sensor plugged in later is picked up within a couple of seconds. The same two actions are on the serial console (`n` and `s`).

## Serial console

**115200 8N1**, USB serial. Open it with:

```
idf.py -p COM21 monitor
```

`Ctrl+]` closes the monitor. Commands are a single character, no Enter. Upper or lower case.

| Key | Same as | What it does |
| --- | --- | --- |
| `n` | Button A | Cycle which sensor the screen draws |
| `s` | Button B | Rescan I2C, drop addresses that disappeared, and arm any new SEN0628 |

Anything else typed is ignored. The bus is also scanned every 2 seconds. That automatic scan adds new sensors and does not drop ones that went quiet; `s` is the scan that drops them.

Boot prints three lines, then a scan:

```
BOOT tof-tester SEN0628 (esp-idf)
BUS SDA=32 SCL=33 100kHz pullup=internal-3v3
Expect SEN0628 at 0x30-0x33, switch in I2C mode
SCAN n=1 0x32
```

`SCAN` is printed only when the set of addresses changes. `n=` is how many devices ACKed. Each `0xNN` follows. `SCAN n=0 none` means the bus is empty.

A new SEN0628 then prints:

```
MODE 0x32 ok, arming
```

`MODE 0x32 FAIL` means the sensor ACKed but did not accept 8x8 mode. After a successful mode change the sensor needs about 5 seconds before the first frame. During that wait, and whenever a later read misses, the stick prints one line and no grid:

```
MAT 0x32 wait
MAT 0x32 mode-fail
```

`wait` means it is armed and the read did not return a fresh frame. `mode-fail` means 8x8 mode is not set and it will try again. A fresh frame is the full matrix, distances in millimetres, row `r0` at the top through `r7`:

```
MAT 0x32 ok min=641 max=2196 ctr=0
r0 1419 1523 1614 1717 1870 1949 2196 4000
r1 1605 1734 4000 4000 4000 4000 4000 4000
r2 ...
r7 ...
```

`min` and `max` ignore cells under 20 mm and cells at `4000`. `ctr` is the mean of the middle four cells, or `0` when those four are no-return. `4000` is no return. Frames arrive about every 200 ms, one block per sensor.

Other lines:

| Line | When |
| --- | --- |
| `HB t=<ms> found=<n>` | Once a second while no SEN0628 is armed. `t` is milliseconds since boot. `found` counts every address that ACKed, not only `0x30`–`0x33`. |
| `DROP 0x32` | An `s` or Button B scan no longer sees a sensor that was armed. |

## Wiring

M5StickC Plus Grove port, latch up, left to right: **GND, 5V, SDA (GPIO32, yellow), SCL (GPIO33, white)**.

The SEN0628 Gravity cable is a different order: **VCC, GND, SCL, SDA** (red, black, blue, green). Do not plug that cable straight into the Grove port. Match the signals:

| Sensor | Stick Grove |
| --- | --- |
| VCC | 5V (red) |
| GND | GND (black) |
| SDA | GPIO32 (yellow) |
| SCL | GPIO33 (white) |

The sensor accepts 3.3–5 V. Its onboard I2C pull-ups were removed so they cannot feed 5 V back into the ESP32. The firmware enables the ESP32 internal pull-ups to 3.3 V. Put the sensor switch in **I2C** mode and power-cycle it after changing the address (`0x30`, `0x31`, `0x32`, `0x33`).

## Build and flash

ESP-IDF 5.5. The StickC Plus USB chip enumerates as an FTDI port (COM21 on this machine). Flash at 1500000 baud; the console stays at 115200.

```
. C:\Espressif\frameworks\esp-idf-v5.5.2-2\export.ps1
idf.py set-target esp32
idf.py -p COM21 -b 1500000 flash
idf.py -p COM21 monitor
```
