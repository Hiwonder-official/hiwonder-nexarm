# HX-30HM Serial Bus Servo — Complete Control Protocol

## Overview

| Item | Value |
|------|-------|
| Interface | Single-wire half-duplex UART |
| Baudrate | 1,000,000 bps (1 Mbps, default) |
| Position resolution | 12-bit (0 – 4095) |
| Position range | 0° – 300° |
| Servo ID range | 1 – 253 |
| Broadcast ID | 0xFE (254) — no reply |

---

## Frame Format

All packets (host → servo and servo → host) share the same structure:

```
[0xFF] [0xFF] [ID] [LEN] [CMD] [PARAM1] ... [PARAMN] [CHECKSUM]
```

| Field | Size | Description |
|-------|------|-------------|
| `0xFF 0xFF` | 2 bytes | Fixed header |
| `ID` | 1 byte | Servo ID (1–253) or broadcast 0xFE |
| `LEN` | 1 byte | Byte count from CMD through CHECKSUM = N_params + 2 |
| `CMD` | 1 byte | Command / instruction code |
| `PARAM1…N` | N bytes | Parameters (0 or more) |
| `CHECKSUM` | 1 byte | `(~(ID + LEN + CMD + PARAM1 + … + PARAMN)) & 0xFF` |

**Reply packet** uses the same format. The servo echoes its own ID, returns the error byte as CMD, and places read data in PARAMS.

---

## Signed Value Encoding

HX-30HM uses a sign-magnitude scheme (not two's complement) for signed values:

- **Host → Servo**: bit 15 (or specified bit) = sign bit, remaining bits = magnitude.
  - Positive: value as-is.
  - Negative: `(-value) | (1 << sign_bit)`
- **Servo → Host**: bit 15 set means negative; value = `-(raw & ~(1 << sign_bit))`

Examples for position (sign bit = 15), speed (sign bit = 15), load (sign bit = 10):

```c
// encode
uint16_t encode(int16_t val, uint8_t bit) {
    return val < 0 ? (uint16_t)((-val) | (1U << bit)) : (uint16_t)val;
}
// decode
int16_t decode(uint16_t raw, uint8_t bit) {
    return (raw & (1U << bit)) ? -(int16_t)(raw & ~(1U << bit)) : (int16_t)raw;
}
```

---

## Command Table

| CMD | Code | Direction | Reply | Description |
|-----|------|-----------|-------|-------------|
| PING | 1 | Host → Servo | Yes | Check if servo is alive |
| READ | 2 | Host → Servo | Yes | Read N bytes from register address |
| WRITE | 3 | Host → Servo | No* | Write registers immediately |
| REG_WRITE | 4 | Host → Servo | No* | Stage a write (execute on ACTION) |
| ACTION | 5 | Host → Servo | No* | Execute all staged REG_WRITE commands |
| RESET | 6 | Host → Servo | No | Factory reset |
| SYNC_WRITE | 131 (0x83) | Host → Multi | No | Write same registers to multiple servos at once |
| SYNC_READ | 130 (0x82) | Host → Multi | Yes×N | Read same registers from multiple servos |

*Reply suppressed when using broadcast ID `0xFE`.

---

## Register Map

### EEPROM (persistent, survives power cycle)

| Addr | Name | Bytes | R/W | Range | Description |
|------|------|-------|-----|-------|-------------|
| 3 | Main Version | 1 | R | — | Firmware main version |
| 4 | Sub Version | 1 | R | — | Firmware sub version |
| 5 | ID | 1 | R/W | 1–253 | Servo ID |
| 6 | Baudrate | 1 | R/W | 0–7 | See baudrate table |
| 21 | PID_P | 1 | R/W | 0–254 | Proportional gain |
| 22 | PID_D | 1 | R/W | 0–254 | Derivative gain |
| 23 | PID_I | 1 | R/W | 0–254 | Integral gain |
| 24 | Min Start Force L | 1 | R/W | 0–1000 | Minimum drive force low byte |
| 25 | Min Start Force H | 1 | R/W | 0–1000 | Minimum drive force high byte |
| 26 | CW Dead Band | 1 | R/W | 0–254 | Clockwise dead band |
| 27 | CCW Dead Band | 1 | R/W | 0–254 | Counter-clockwise dead band |
| 31 | Position Offset L | 1 | R/W | ±2047 | Position offset low byte (sign-magnitude, bit 11) |
| 32 | Position Offset H | 1 | R/W | ±2047 | Position offset high byte |
| 33 | Mode | 1 | R/W | 0–2 | Operating mode (see mode table) |

### RAM (volatile, reset on power cycle)

| Addr | Name | Bytes | R/W | Range | Description |
|------|------|-------|-----|-------|-------------|
| 40 | Torque Enable | 1 | R/W | 0–1, 128 | 0=free, 1=position hold, 128=calibrate |
| 41 | ACC | 1 | R/W | 0–254 | Acceleration (0=max/instant, higher=smoother) |
| 42 | Goal Position L | 1 | R/W | ±30719 | Target position low byte (sign-magnitude, bit 15) |
| 43 | Goal Position H | 1 | R/W | ±30719 | Target position high byte |
| 44 | PWM Speed L | 1 | R/W | ±1000 | PWM speed (motor mode) low byte (sign-magnitude, bit 10) |
| 45 | PWM Speed H | 1 | R/W | ±1000 | PWM speed high byte |
| 46 | Goal Speed L | 1 | R/W | ±3400 | Max speed for position mode low byte (sign-magnitude, bit 15) |
| 47 | Goal Speed H | 1 | R/W | ±3400 | Max speed high byte |
| 48 | Max Torque L | 1 | R/W | 0–1000 | Torque limit low byte |
| 49 | Max Torque H | 1 | R/W | 0–1000 | Torque limit high byte |
| 56 | Present Position L | 1 | R | ±30719 | Current position low byte (sign-magnitude, bit 15) |
| 57 | Present Position H | 1 | R | ±30719 | Current position high byte |
| 58 | Present Speed L | 1 | R | ±3400 | Current speed low byte (sign-magnitude, bit 15) |
| 59 | Present Speed H | 1 | R | ±3400 | Current speed high byte |
| 60 | Present Load L | 1 | R | ±1023 | Current load low byte (sign-magnitude, bit 10) |
| 61 | Present Load H | 1 | R | ±1023 | Current load high byte |
| 62 | Present Voltage | 1 | R | — | Current voltage × 0.1 V |
| 63 | Present Temperature | 1 | R | — | Current temperature °C |
| 66 | Moving Status | 1 | R | 0–1 | 1 = servo is moving |
| 69 | Present Current L | 1 | R | 0–65535 | Current draw low byte (mA) |
| 70 | Present Current H | 1 | R | 0–65535 | Current draw high byte |

---

## Operating Modes (register 33)

| Value | Mode | Description |
|-------|------|-------------|
| 0 | POSITION_MODE | Standard servo position control |
| 1 | CLOSED_LOOP_MOTOR_MODE | Continuous rotation with speed feedback |
| 2 | OPEN_LOOP_MOTOR_MODE | Continuous rotation, PWM speed only |

---

## Baudrate Table (register 6)

| Value | Baudrate |
|-------|----------|
| 0 | 1,000,000 (default) |
| 1 | 500,000 |
| 2 | 250,000 |
| 4 | 115,200 |
| 5 | 76,800 |
| 6 | 57,600 |
| 7 | 38,400 |

---

## Position ↔ Angle Conversion

```
angle (°) = position × 300 / 4095
position  = angle × 4095 / 300
```

| Position | Angle |
|----------|-------|
| 0 | 0° |
| 2048 | ~150° (center) |
| 4095 | 300° |

---

## Error Byte (in reply packets)

| Bit | Mask | Name |
|-----|------|------|
| 0 | 0x01 | Input voltage error |
| 1 | 0x02 | Angle limit error |
| 2 | 0x04 | Overheating error |
| 3 | 0x08 | Range error |
| 4 | 0x10 | Checksum error |
| 5 | 0x20 | Instruction error |
| 6 | 0x40 | Overload error |

`0x00` = no error.

---

## Command Details and Examples

### PING — Check servo ID 1

```
TX: FF FF 01 02 01 FB
RX: FF FF 01 02 00 FC   (error=0x00 means OK)
```

---

### READ — Read present position from servo ID 1

`CMD=2`, params = [start_addr, length]

```
TX: FF FF 01 04 02 38 02 BF
                  |  └─ length = 2 bytes
                  └──── start addr = 56 (0x38) = Present Position L

RX: FF FF 01 04 00 PL PH CS
                     |  └─ Position H
                     └──── Position L
```

Decode: `raw = PL | (PH << 8)`, then apply sign-magnitude decode (bit 15).

---

### READ — Read position + speed (4 bytes from addr 56)

```
TX: FF FF 01 04 02 38 04 BD

RX: FF FF 01 06 00 PL PH SL SH CS
```

---

### READ — Read voltage + temperature (2 bytes from addr 62)

```
TX: FF FF 01 04 02 3E 02 B9

RX: FF FF 01 04 00 VL TL CS
                     |  └─ Temperature (°C)
                     └──── Voltage (× 0.1 V)
```

---

### READ — Read current (2 bytes from addr 69)

```
TX: FF FF 01 04 02 45 02 B2

RX: FF FF 01 04 00 CL CH CS
```

---

### READ — Read load (2 bytes from addr 60)

```
TX: FF FF 01 04 02 3C 02 BB

RX: FF FF 01 04 00 LL LH CS
```
Decode with sign-magnitude bit 10.

---

### READ — Read moving status (1 byte from addr 66)

```
TX: FF FF 01 04 02 42 01 B6

RX: FF FF 01 03 00 MS CS
                     └─ 0=stopped, 1=moving
```

---

### WRITE — Enable torque on servo ID 1

```
TX: FF FF 01 04 03 28 01 CF
                  |  └─ value = 1
                  └──── addr = 40 (0x28) = Torque Enable
```

### WRITE — Disable torque (free-drag)

```
TX: FF FF 01 04 03 28 00 D0
```

### WRITE — Calibrate position (set current position as center)

```
TX: FF FF 01 04 03 28 80 50
                        └─ 128 = calibrate command
```

---

### WRITE — Set goal position to 2048 (center, ~150°)

```
2048 = 0x0800  →  L=0x00, H=0x08

TX: FF FF 01 05 03 2A 00 08 C3
                  └──────── addr = 42 (0x2A) = Goal Position L
```

---

### WRITE — Set acceleration + position + speed together (7 bytes from addr 41)

This is `servo_write_pos_ex`: writes ACC, Goal_Pos (2B), 0x00 0x00, Goal_Speed (2B) in one packet.

```
Format: [ACC][POS_L][POS_H][0x00][0x00][SPD_L][SPD_H]

Example: acc=100, pos=2048, speed=1000
  acc=100=0x64
  pos=2048=0x0800 → L=0x00, H=0x08
  speed=1000=0x03E8 → L=0xE8, H=0x03

TX: FF FF 01 0A 03 29 64 00 08 00 00 E8 03 [CS]
                  └──── addr = 41 (0x29) = ACC
```

---

### WRITE — Set mode to position mode

```
TX: FF FF 01 04 03 21 00 D7
                  |  └─ 0 = POSITION_MODE
                  └──── addr = 33 (0x21) = Mode
```

### WRITE — Set mode to closed-loop motor mode

```
TX: FF FF 01 04 03 21 01 D6
```

---

### WRITE — Set PWM speed (motor mode, addr 44)

Range ±1000, sign-magnitude bit 10.

```
speed = 500 → encode = 500 = 0x01F4 → L=0xF4, H=0x01

TX: FF FF 01 05 03 2C F4 01 F8
```

---

### WRITE — Set max torque

```
torque = 800 = 0x0320 → L=0x20, H=0x03

TX: FF FF 01 05 03 30 20 03 A6
                  └──── addr = 48 (0x30) = Max Torque L
```

---

### WRITE — Set position offset (addr 31)

Range ±2047, sign-magnitude bit 11.

```
offset = -100 → encode: (-(-100)) | (1<<11) = 100 | 2048 = 2148 = 0x0864
→ L=0x64, H=0x08

TX: FF FF 01 05 03 1F 64 08 [CS]
                  └──── addr = 31 (0x1F) = Pos Offset L
```

---

### REG_WRITE + ACTION — Staged synchronised move

Stage a write on each servo without executing, then fire all at once with ACTION.

```
# Stage servo 1: pos=1000
TX: FF FF 01 09 04 29 00 E8 03 00 00 00 00 [CS]

# Stage servo 2: pos=2048
TX: FF FF 02 09 04 29 00 00 08 00 00 00 00 [CS]

# Execute all staged writes simultaneously (broadcast)
TX: FF FF FE 02 05 FA
```

---

### SYNC_WRITE — Move multiple servos simultaneously

`CMD=0x83`, params = [start_addr, data_len_per_servo, ID1, data..., ID2, data..., ...]

**Example: set acc+pos+speed for 3 servos (7 bytes each, starting at addr 41)**

```
servo 1: id=1, acc=100, pos=1000, speed=500
servo 2: id=2, acc=100, pos=2048, speed=500
servo 3: id=3, acc=100, pos=3000, speed=500

pos 1000 = 0x03E8 → L=E8, H=03
pos 2048 = 0x0800 → L=00, H=08
pos 3000 = 0x0BB8 → L=B8, H=0B
speed 500 = 0x01F4 → L=F4, H=01

TX: FF FF FE [LEN] 83 29 07
    01  64  E8 03  00 00  F4 01
    02  64  00 08  00 00  F4 01
    03  64  B8 0B  00 00  F4 01
    [CS]

LEN = 2 + (1 + 7) × 3 + 2 = 28 = 0x1C
```

---

### SYNC_READ — Read present position from multiple servos

`CMD=0x82`, params = [start_addr, byte_num, ID1, ID2, ...]

Each servo replies individually in order. Host must receive N reply packets.

**Example: read position (2 bytes) from servos 1, 2, 3**

```
TX: FF FF FE 07 82 38 02 01 02 03 [CS]
                     |  |  └─────── IDs
                     |  └────────── byte_num = 2
                     └───────────── start addr = 56 = Present Position L

RX (3 separate replies):
FF FF 01 04 00 PL1 PH1 CS1
FF FF 02 04 00 PL2 PH2 CS2
FF FF 03 04 00 PL3 PH3 CS3
```

**Example: read position + speed + load + voltage + temperature (8 bytes) from servos 1–6**

```
TX: FF FF FE 0A 82 38 08 01 02 03 04 05 06 [CS]

Each reply: [POS_L][POS_H][SPD_L][SPD_H][LOAD_L][LOAD_H][VOLTAGE][TEMPERATURE]
```

Decode:
- Position (bit 15 sign-magnitude): `MASK_SERVO(pos_raw, 15)`
- Speed (bit 15): `MASK_SERVO(speed_raw, 15)`
- Load (bit 10): `MASK_SERVO(load_raw, 10)`
- Voltage: raw × 0.1 V
- Temperature: raw °C

---

### RESET — Factory reset servo ID 1

```
TX: FF FF 01 02 06 F6
```

> Warning: resets ID back to 1, baudrate back to 1 Mbps, and all EEPROM registers to defaults.

---

## Checksum Examples

```
Packet: FF FF 01 04 02 38 02 [CS]
Sum of ID+LEN+CMD+PARAMS = 01+04+02+38+02 = 0x41
CS = (~0x41) & 0xFF = 0xBE
```

```
Packet: FF FF 01 02 01 [CS]   (PING)
Sum = 01+02+01 = 0x04
CS = (~0x04) & 0xFF = 0xFB
```

---

## Torque Enable Values Summary

| Value | Effect |
|-------|--------|
| 0 | Torque off — free drag mode |
| 1 | Torque on — position hold / servo mode |
| 128 | Calibrate — sets current physical position as zero offset |

---

## High-Level API Summary (from HX_30HM.c)

| Function | Registers | Notes |
|----------|-----------|-------|
| `servo_ping(id)` | — | PING |
| `servo_enable_torque(id)` | 40 ← 1 | WRITE |
| `servo_disable_torque(id)` | 40 ← 0 | WRITE |
| `servo_cali_pos(id)` | 40 ← 128 | WRITE, set current pos as center |
| `servo_select_mode(id, mode)` | 33 ← mode | WRITE |
| `servo_write_pos(id, pos)` | 42–43 | WRITE, sign-mag bit 15 |
| `servo_write_acc(id, acc)` | 41 | WRITE, 0–254 |
| `servo_write_speed(id, speed)` | 46–47 | WRITE, sign-mag bit 15, ±3400 |
| `servo_write_pos_ex(id, acc, speed, pos)` | 41–47 | WRITE 7 bytes: acc+pos+00 00+speed |
| `servo_write_pwm_speed(id, speed)` | 44–45 | WRITE, sign-mag bit 10, ±1000 |
| `servo_write_max_torque(id, torque)` | 48–49 | WRITE, 0–1000 |
| `servo_write_pos_offset(id, offset)` | 31–32 | WRITE, sign-mag bit 11, ±2047 |
| `servo_write_reg_pos_ex(id, acc, speed, pos)` | 41–47 | REG_WRITE (staged) |
| `servo_reg_action(id)` | — | ACTION (execute staged) |
| `servo_sync_write_pos_ex(data, n)` | 41 (7B each) | SYNC_WRITE acc+pos+speed for N servos |
| `servo_read_pos(id, *pos)` | 56–57 | READ 2B, sign-mag bit 15 |
| `servo_read_speed(id, *speed)` | 58–59 | READ 2B, sign-mag bit 15 |
| `servo_read_pos_speed(id, *pos, *speed)` | 56–59 | READ 4B |
| `servo_read_load(id, *load)` | 60–61 | READ 2B, sign-mag bit 10 |
| `servo_read_voltage(id, *vol)` | 62 | READ 1B, × 0.1 V |
| `servo_read_temperture(id, *temp)` | 63 | READ 1B, °C |
| `servo_read_current(id, *cur)` | 69–70 | READ 2B, mA |
| `servo_read_moving_status(id, *ms)` | 66 | READ 1B, 0=stopped 1=moving |
| `servo_read_pos_offset(id, *offset)` | 31–32 | READ 2B, sign-mag bit 11 |
| `servo_sync_read_cur_pos_ex(ids, n, data)` | 56 (8B each) | SYNC_READ pos+speed+load+volt+temp |
| `general_write(id, addr, data, len)` | any | Generic WRITE |
| `general_read(id, addr, data, len)` | any | Generic READ |
| `sync_write(addr, data, len, param_len)` | any | Generic SYNC_WRITE |
| `sync_read(addr, byte_num, ids, n, data)` | any | Generic SYNC_READ |

---

## NexArm ESP32 CommProtocol (host ↔ ESP32 layer)

The ESP32 wraps the above in its own framing for USB serial communication with the host PC. The ESP32 then translates these into native HX-30HM packets on the servo bus.

### ESP32 Frame Format

```
[0xFF] [0xFF] [0xFF] [LEN] [CMD] [PARAMS...] [CHECKSUM]
```

Checksum = `(~(0xFF + LEN + CMD + PARAMS...)) & 0xFF`

### ESP32 Command Table

| CMD | Params | Reply | Maps to HX-30HM | Description |
|-----|--------|-------|-----------------|-------------|
| 56 | `[ACC, SPD_L, SPD_H]` | No | REG_WRITE acc+speed on all 6 | Set motion_acc (0–254) + motion_speed (0–3400) |
| 68 | `[0x01]` / `[0x00]` | No | — | Enter (1) / exit (0) LeRobot bridge mode |
| 96 | — | Yes, 12B | SYNC_READ present pos × 6 | Read positions of all 6 joints (6 × int16 LE) |
| 97 | 12 bytes (6 × int16 LE) | No | SYNC_WRITE goal pos × 6 | Write positions to all 6 joints simultaneously |
| 98 | `[0x01]` / `[0x00]` | No | WRITE torque_enable × 6 | Enable (1) / disable (0) torque on all 6 servos |

### Position encoding in CMD 96/97

Positions are raw 12-bit values (0–4095), packed as little-endian int16:

```python
import struct
# encode 6 positions
args = b"".join(struct.pack("<h", p) for p in positions)   # 12 bytes

# decode reply
positions = [struct.unpack_from("<h", reply, i*2)[0] for i in range(6)]
```
