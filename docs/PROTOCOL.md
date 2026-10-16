# Goodix GXFP3200 ("Milan F") SPI protocol

Notes on the sensor protocol as implemented by the `goodixspi` driver. Everything
here was observed on an ASUS ZenBook 13 UX331F (chip id `0x002202a0`).

## Bus

- ACPI device `GXFP3200` under the LPSS SPI controller (`spi-GXFP3200:00`).
- SPI mode 0, 8 bits, chip select active low.
- ACPI advertises 4 MHz. Register access is done at 4 MHz. The frame read
  (CRC-protected) works at 8 MHz on the tested unit; 10 MHz and above fails.
- The ACPI node also declares a reset GPIO and an interrupt GPIO. The driver
  needs neither: it polls the status register.

## Register access

All registers and values are 16 bits, big endian.

```
write:  F0 RH RL 00 01 VH VL                 (00 01 = one 16-bit word)
read:   F0 RH RL                             (sets the address)
        F1 + read n bytes                    (one transfer, CS held)
single-byte commands: C0 (idle/ready), C2 (start image)
```

## Identification

Read 4 bytes at `0x0000`: `b0 b1 b2 b3`, so `id = b2<<24 | b3<<16 | b0<<8 | b1`.
The vendor driver accepts these `id >> 8` values: `0x2202` (Milan F, this
sensor), `0x2207`, `0x2208`, `0x220c`, `0x220f` and `0x2503`. Only `0x2202` is
supported here.

## Enable

```
w 0x124 = 0x0D00
w 0x204 = 0x0000
cmd C0, wait 10 ms
```

## OTP (32 bytes, factory calibration)

Read byte `i` (`a = i >> 2`, `b = i & 3`) through a register window:

```
w 0x102=0x0020; w 0x104=a;      w 0x102=0x0021; w 0x102=0x0020
w 0x104=b<<8;   w 0x102=0x0022; w 0x102=0x0020
r 0x106  -> low byte
```

Only control values `0x20`, `0x21` and `0x22` are written to `0x102`, which is
exactly what the vendor driver does. Do not experiment with other values: this
is one-time-programmable memory.

To validate the data, the driver first tries a byte sum and then CRC-8
(poly 0x07, init 0, xorout 0xFF) over 22, 23 or 25 bytes, compared with
`otp[0x1E]`. An all-zero OTP is also accepted.

Register `0x5C` gets an OTP-derived value: if `otp[0x16] != 0` and
`otp[0x16] + otp[0x17] == 0xFF`, then `0x5C = ((otp[0x16] >> 4) + 1) * 0x10`.

## Image capture

```
cmd C0; wait 1 ms; w 0x124 = 0x00FF
w 0x05C = 0x0080; w 0x120 = 0x0408; w 0x052 = 0x0008; w 0x050 = 0x0501
w 0x05C = <OTP value>              (only if non-zero)
cmd C2; wait 1 ms; w 0x054 = 0x0110
poll r 0x126 until (value & 0x1FF) == 0x008  (image ready); ack: w 0x124 = value
read 0x37B4 bytes at address 0xAAAA
```

Bits in status register `0x126` (each event is acknowledged by writing the same
value to `0x124`):

| value | event |
|---|---|
| `0x008` | image ready |
| `0x002` | finger detect (FDT) |
| `0x010` | ESD |
| `0x080`, `0x082` | "reverse" |
| `0x100` | reset |

## Frame format

- `0x37B4` bytes: `0x37B0` bytes of pixel data followed by a 4-byte CRC.
- CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, not reflected, no xorout)
  over the first `0x37B0` bytes. The stored value is
  `b[n+2]<<24 | b[n+3]<<16 | b[n]<<8 | b[n+1]`.
- 108 x 88 pixels of 12 bits. Every 6 bytes hold 4 pixels:
  `p0 = (b0&0xF)<<8 | b1`, `p1 = b3<<4 | b0>>4`,
  `p2 = (b5&0xF)<<8 | b2`, `p3 = b4<<4 | b5>>4`.
- A finger lowers the reading. The driver works with `background - frame`, where
  the background is the empty sensor.

## Matching

The sensor covers about 5.5 x 4.5 mm and yields only 3 to 8 minutiae, so the
NBIS/bozorth matcher in libfprint does not work. The driver matches on the host:

1. Preprocessing: crop a 3 px border, normalize contrast locally (11x11
   window), and mask out areas without finger contact.
2. Masked normalized cross-correlation (Padfield) between the probe and each
   enrolled image over every translation (via FFT) and rotations of
   -12, -6, 0, 6 and 12 degrees. The overlap must cover at least 35% of the
   probe.
3. Accept if any enrolled image reaches 0.88.

Enrollment stores 30 raw frames in the print. They are stored raw, not
preprocessed, so the matcher can change without re-enrolling.
