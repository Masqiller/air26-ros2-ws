# ESP32-S3 micro-ROS: reboot / session-churn fix, and hard-won hardware constants

Debugging notes for the AIR26 AprilTag-following robot (ESP32-S3 vehicle + ESP32-CAM,
micro-ROS over WiFi/UDP, ROS 2 Humble).

Everything below was measured on the actual hardware, not assumed. Where a number was
originally a textbook guess that turned out wrong, both values are recorded so the mistake
isn't repeated.

---

## 1. The main bug: agent session churn / spontaneous reboots

### Symptom

The micro-ROS agent log looped forever, roughly every 3–15 seconds:

```
create_client   | client_key: 0x299E1DD7
establish_session
create_participant / 4x create_publisher / create_subscriber
delete_client   | client_key: 0x299E1DD7
destroy_session
create_client   | client_key: 0x1D2D9029      <- new key, starts over
...
```

Topics appeared in `ros2 topic list` but data was unreliable, and the follower kept losing
the vehicle.

### How the log was read

Two details narrowed it down before touching any code:

- `delete_client` is **client-initiated**. The agent was not dropping the board; the board
  was tearing down its own session. That points at the firmware's reconnect state machine,
  not at the agent.
- One destroy → create pair was **36 microseconds** apart
  (`...081.612200` → `...081.612236`). A reboot needs seconds to rejoin WiFi, so at least
  some cycles were pure state-machine churn, not crashes.
- Other gaps were 1.5–2 s, which *is* consistent with a reboot.

That mix of instant and slow reconnects meant there were **two independent faults**.

### Root cause A — encoder ISRs were calling into flash (caused real reboots)

This was the serious one, and the same failure class as the well-known
`Wire`-plus-interrupts crash on ESP32.

The Arduino core in use is built **without** `CONFIG_ARDUINO_ISR_IRAM`:

```c
/* framework-arduinoespressif32/cores/esp32/esp32-hal.h */
#if CONFIG_ARDUINO_ISR_IRAM
#define ARDUINO_ISR_ATTR IRAM_ATTR
#define ARDUINO_ISR_FLAG ESP_INTR_FLAG_IRAM
#else
#define ARDUINO_ISR_ATTR              /* <-- expands to NOTHING */
#define ARDUINO_ISR_FLAG (0)
#endif
```

So `digitalRead()` is **not** in IRAM. Confirmed by inspecting the built image:

```
__digitalRead     @ 0x42006f48   <- FLASH
__onPinInterrupt  @ 0x4208ae64   <- FLASH  (Arduino's GPIO dispatcher)
isr_enc_left      @ 0x4037530c   <- IRAM
```

The ISRs *were* marked `IRAM_ATTR`, but the first thing they did was call `digitalRead()`
and jump straight back into flash:

```cpp
// BROKEN: IRAM_ATTR is defeated by the flash call inside it
void IRAM_ATTR isr_enc_left() {
  if (digitalRead(ENC_L_B) == digitalRead(ENC_L_A)) enc_left_ticks++; else enc_left_ticks--;
}
```

When an encoder edge arrives while the flash cache is momentarily disabled (routine during
SPI-flash access), the CPU faults with *"Cache disabled but cached memory region accessed"*
and the chip reboots. While driving, both wheels generate roughly **560 edges/second**, so
that window gets hit frequently.

**Fix** — read the GPIO register directly. `REG_READ` is a macro over a volatile load, so it
inlines and never leaves IRAM:

```cpp
#include <soc/gpio_reg.h>

void IRAM_ATTR isr_enc_left() {
  uint32_t g = REG_READ(GPIO_IN_REG);
  if (((g >> ENC_L_B) & 1U) == ((g >> ENC_L_A) & 1U)) enc_left_ticks++;
  else                                                enc_left_ticks--;
}
```

Side benefit: one register read samples A and B at the **same instant**, which is more
correct than two sequential `digitalRead()` calls that can straddle an edge.

All four encoder pins are below 32, so `GPIO_IN_REG` covers them. Pins ≥ 32 would need
`GPIO_IN1_REG`.

### Root cause B — the ping timeout was 10x too tight (caused the instant churn)

The two boards were never configured the same:

| board | ping | interval | behaviour |
|---|---|---|---|
| vehicle (before) | `rmw_uros_ping_agent(100, 3)` | 200 ms | churned constantly |
| ESP32-CAM | `rmw_uros_ping_agent(1000, 3)` | 200 ms | never churned |

A 100 ms round-trip budget is unrealistic on 2.4 GHz shared with the camera's MJPEG stream.
Ordinary latency spikes failed all three attempts, so the firmware ran `destroy_entities()`
and immediately rebuilt — the 36-microsecond reconnects.

**Fix**: `rmw_uros_ping_agent(500, 3)` checked every 1000 ms. Five times more tolerant,
while keeping worst-case blocking bounded at 1.5 s (and only when the link really is dead).

### Root cause C — blocking sensor reads starved the executor

`on_timer` (10 Hz) read all three HC-SR04s back-to-back. `pulseIn` blocks for up to
`US_MAX_M * 6000` µs ≈ **12 ms** when there is no echo, so the callback stalled for ~36 ms
out of every 100 ms, adding latency that made the tight ping above even more fragile.

**Fix**: round-robin, one sensor per tick (~12 ms worst case). Each sensor still refreshes
at ~3.3 Hz and all three are still published every tick. This is the same pattern
`test/PID.cpp` already used, for the same reason.

### Root cause D — unsupervised motor output during a stall (safety)

With bang-bang control, a blocked `loop()` means the wheels keep their last PWM and drive
blind. A ping can stall the loop for up to ~1.5 s.

**Fix**: a starvation guard in the control step. If the gap since the previous step exceeds
200 ms it cuts the motors, resets the controllers, and resyncs the encoder tick baseline so
the stall isn't misread as a huge speed spike.

### How to verify the fix

**ISRs are flash-free** (the point of fix A):

```bash
ELF=~/Documents/Platoon/.pio/build/esp32-s3-devkitc-1/firmware.elf
OBJDUMP=$(find ~/.platformio/packages -name "xtensa-esp32s3-elf-objdump" | head -1)

# addresses must be 0x403xxxxx (IRAM), never 0x42xxxxxx (flash)
$(dirname $OBJDUMP)/xtensa-esp32s3-elf-nm "$ELF" | grep isr_enc

# must print NO call instructions
"$OBJDUMP" -d --disassemble=isr_enc_left "$ELF" | grep -E "call[048x]|callx"
```

Confirmed result: `isr_enc_left @ 0x4037530c`, `isr_enc_right @ 0x40375340`, zero calls.

**Reboot vs churn** — the boot banner distinguishes them:

```
[microbot] boot #1   reset reason: POWERON (clean power-up)   <- healthy
[microbot] boot #7   reset reason: PANIC (firmware crashed)   <- was fix A
[microbot] boot #3   reset reason: BROWNOUT  <<< THE SUPPLY RAIL COLLAPSED
```

A climbing `boot #N` means it is rebooting. A steady `boot #1` with churning sessions means
the network, i.e. fix B.

### Residual risk

`__onPinInterrupt`, the Arduino core's own GPIO dispatcher, is still in flash
(`0x4208ae64`) and cannot be moved without a custom IDF build. In practice the crash window
is dominated by flash *writes*, and the main source of those is already avoided:
`WiFi.persistent(false)` in the join path stops WiFi credentials being rewritten to NVS on
every connect. If panics ever return, that dispatcher is the next thing to attack.

---

## 2. Other bugs found along the way

### Encoder feedback signs differ per side

Measured with `test/MainFinal.cpp` (same pins, same ISR, same RISING edge), both motors
driven forward at PWM 180:

| wheel | ticks / 200 ms | rate | counts |
|---|---|---|---|
| left | −56 | ≈ −280 tps | **backward** → `ENC_L_SIGN = -1` |
| right | +57 | ≈ +285 tps | **forward** → `ENC_R_SIGN = +1` |

Equal magnitude, **opposite sense**, so the two signs must differ. An older note in
`test/PID.cpp` claimed both were backward; that predates the ESP32-S3 swap and the 5 V
rewiring, which evidently flipped the right encoder's A/B.

A wrong `ENC_R_SIGN` makes that wheel's loop **positive feedback**: it saturates at the PWM
cap while the runaway guard repeatedly resets both controllers, so the *other* wheel never
spins up. Symptom: "only the right motor spins, the left barely moves."

`ENC_*_SIGN` is also applied to `/joint_states`, so published wheel angles increase when
driving forward. `wheel_odometry` therefore runs with `encoder_direction = +1`. **Both must
be changed together** or the sign double-flips and the robot appears to spin in RViz
instead of driving.

### Feedforward was 5.8x wrong

`test/PID.cpp` assumed `KFF = 0.11` from "110 PWM → 1000 tps". The direct measurement above
gives **PWM 180 → 280 tps**, i.e. `KFF = 180/280 = 0.64`. Consequences of the old value:

```
0.12 m/s needed ~124 PWM of feedforward but got only 21
MAX_PWM=110 could only ever reach 172 tps = 0.106 m/s
```

The integrator had to do all the work and the loop sat saturated — very likely why the
original PID only worked "to an extent". Its `CRUISE_TPS = 300` was unreachable at
`MAX_PWM = 110`.

### Camera focal length was 3.3x wrong

`fx = 250` assumes the usual ~65° QVGA lens. Measured on this ESP32-CAM: a tag at a
tape-measured **0.56 m** produced `side_px ≈ 118`, so

```
fx = side_px * distance / tag_size = 118 * 0.56 / 0.08 ≈ 820
```

which implies only a **~22° horizontal FOV**. This board **windows/crops the sensor** for
grayscale QVGA rather than downscaling the full field, so the usual
`fx ≈ (width/2) / tan(FOV/2)` shortcut does not apply here.

With `fx = 250` every distance read **3.3x too small** (0.17 m instead of 0.56 m), which put
the follower permanently in its "too close, reverse" zone so it never followed anything.

Consequences of the correction, with an 80 mm tag:

| distance | tag size on screen | note |
|---|---|---|
| 0.30 m | 219 px (91 % of frame height) | nearly overflows |
| 0.40 m | 164 px (68 %) | practical minimum |
| 0.45 m | 146 px (61 %) | stop distance |
| 1.00 m | 66 px (27 %) | |
| 2.00 m | 33 px (14 %) | near decode limit |

Acquisition range improved from ~0.74 m to **~2.4 m**.

### AprilTag size: use the black square, not the total

The generator reports two numbers. For `tag25h9` (7 black modules, 9 including the quiet
border):

```
TAG SIZE    =  80.0 mm   <- the black square. THIS is tag_size.
TOTAL SIZE  = 102.9 mm   <- includes the white quiet zone (80 x 9/7)
```

The detector's corners sit on the **outer black border**, so `tag_size = 0.080`. Using the
total would under-read every distance by 9/7.

---

## 3. Verified constants (single source of truth)

| constant | value | where | how established |
|---|---|---|---|
| `ENC_L_SIGN` | `-1` | firmware | measured, `MainFinal.cpp` |
| `ENC_R_SIGN` | `+1` | firmware | measured, `MainFinal.cpp` |
| `KFF` | `0.64` | firmware | measured, PWM 180 → 280 tps |
| `MAX_PWM` | `200` | firmware | so 0.19 m/s is reachable at KFF 0.64 |
| `encoder_direction` | `+1` | `wheel_odometry.py` | firmware already sign-corrects |
| `fx`, `fy` | `820` | detector + all followers | measured at 0.56 m |
| `tag_size` | `0.080` m | detector + all followers | generator "TAG SIZE" |
| `safe_distance` | `0.45` m | all followers | requested stop distance |
| wheel separation | `0.251` m | firmware | |

### Still unverified — treat absolute speeds with suspicion

`TICKS_PER_REV = 330` (assumed 11 PPR × 30:1) and the 65 mm wheel diameter have **never**
been physically checked. Every "m/s" in the system scales with both, so commanded speed may
not be true speed. `/joint_states` cannot detect the error because it is computed *using*
`TICKS_PER_REV`, so the error cancels.

The 30-second test: flash `test/singlemotrTEst.cpp` (motor off, prints left ticks as the
wheel is turned by hand), mark the wheel, rotate exactly one full turn, read the tick delta.
That number **is** `TICKS_PER_REV`. Measure the wheel with calipers while you are there.

---

## 4. Drivetrain floor: why bang-bang, not PID

These motors need roughly **70 PWM** to break stiction, which at `KFF = 0.64` is already
~110 ticks/s ≈ **0.07 m/s**. Anything slower cannot be held with a *continuous* PWM, so a
velocity PID either stalls or lurches as its integrator winds up and collapses. Asking for
0.04 m/s made the wheels stop entirely.

Bang-bang sidesteps this: kick a healthy PWM, coast once past target, repeat. The wheel
pulses, but the **average** speed can sit far below the motor's minimum continuous speed —
exactly what a slow follower needs, and there are no gains to tune.

```
speed_along < |target| - hyst  ->  DRIVE at BANG_PWM
speed_along > |target| + hyst  ->  COAST (0), friction bleeds it off
in between                     ->  hold previous   (hysteresis, no chatter)
```

`VELOCITY_MODE` selects `VEL_OPEN`, `VEL_BANG` (default) or `VEL_PID`; the tested PID is
retained, not deleted.

Visible pulsing at low speed is inherent to bang-bang, not a fault. If it is too lumpy,
widen `BANG_HYST_TPS` rather than raising the speed.

---

## 5. Environment gotchas that cost time

**DDS profile.** The CogniPilot block in `~/.bashrc` sets `ROS_DOMAIN_ID=7` +
CycloneDDS, but the micro-ROS Agent speaks **Fast DDS** and its clients land on
**domain 0**. Mixing them makes the robot's topics invisible even though the agent shows
them connected. `dds_fast` / `dds_cyclone` / `dds_status` switch profiles; the default is
Fast DDS on domain 0.

**Camera HTTP stream serves one client at a time.** Running a second `camera_stream` (for
example by launching `hardware.launch.py` twice, or `follow_all.launch.py` on top of it)
starves the first.

**Synthetic tests must remap the input topic too.** Publishing test frames to
`/camera/image_raw` while the real camera is running interleaves both streams and produces
nonsense (detection appeared to flicker every frame). Remap input *and* output.

**Supply.** A 3S pack through the L298N's onboard 78M05 cannot run the ESP32: at 12.6 V in
it dissipates ~2.7 W at WiFi TX current and thermally shuts down. Use a buck converter to
5 V, feed the `5V` pin (never `3V3`), and put 470–1000 µF plus 100 nF at each board.
