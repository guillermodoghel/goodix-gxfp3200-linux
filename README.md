# Goodix GXFP3200 fingerprint sensor on Linux

A [libfprint](https://gitlab.freedesktop.org/libfprint/libfprint) driver
(`goodixspi`) for the **Goodix GXFP3200** SPI fingerprint sensor. This is the
touchpad-corner reader of the **ASUS ZenBook 13 UX331F** (UX331FAL and
siblings). With it, the reader works with `fprintd`: lock screen, `sudo`,
login.

> Unofficial, community-made, and not affiliated with Goodix or ASUS.
> The protocol was worked out from the vendor's Windows driver, for interoperability.

## Does my laptop have this sensor?

```sh
ls /sys/bus/spi/devices/
```

If you see `spi-GXFP3200:00`, it does. Some UX331 units come with an ELAN sensor
instead (`spi-ELAN7001:00`). That one is supported upstream by libfprint's
`elanspi` driver, so this repo is not needed.

Only chip id `0x2202xx` ("Milan F") is supported. If the chip reports something
else, the driver refuses to open it.

## Security: read this first

The sensor is tiny (about 5.5 x 4.5 mm) and gives too few minutiae for
libfprint's standard matcher. This driver therefore uses its own
image-correlation matcher, tested on **one person's** fingers:

- the enrolled finger was accepted 10/10 in the final test;
- the person's other 9 fingers were rejected 37/37 at the default threshold
  (best impostor score 0.81, threshold 0.88);
- the false-accept rate, extrapolated from that small sample, is very roughly
  1 in 500 attempts. A commercial reader is around 1 in 50,000.

That is fine for convenience (lock screen, `sudo` with your password as a
fallback). Do not rely on it as your only factor for anything sensitive. Keep
password authentication enabled everywhere.

## Install

### Arch Linux

```sh
git clone https://github.com/guillermodoghel/goodix-gxfp3200-linux
cd goodix-gxfp3200-linux/packaging/arch
makepkg -si          # builds libfprint (pinned upstream commit) + this driver, replaces "libfprint"
sudo pacman -S fprintd
```

The package also installs `/etc/modprobe.d/goodixspi.conf`, which raises the
`spidev` buffer, and libfprint's udev rules, which bind the sensor to `spidev`
at boot. Reboot once, or reload by hand:

```sh
sudo modprobe -r spidev; sudo udevadm trigger --action=add /sys/bus/spi/devices/spi-GXFP3200:00
ls -l /dev/spidev*            # should show /dev/spidev1.0 (or similar)
```

### Other distributions

Build libfprint from source with the driver added:

```sh
git clone https://gitlab.freedesktop.org/libfprint/libfprint.git
git -C libfprint checkout 6f9479c3d55f847c1b3769f28ceb99227f9858cf   # tested commit
cp goodix-gxfp3200-linux/driver/* libfprint/libfprint/drivers/
cd libfprint && git apply ../goodix-gxfp3200-linux/patches/0001-meson-add-goodixspi-driver.patch
meson setup build -Ddoc=false -Dgtk-examples=false && meson compile -C build && sudo meson install -C build
sudo install -m644 ../goodix-gxfp3200-linux/system/goodixspi.conf /etc/modprobe.d/
```

This installs over your distribution's libfprint, so prefer packaging it the way
your distribution does.

## Enroll

```sh
fprintd-enroll -f right-index-finger
fprintd-verify
```

Enrollment takes **30 touches**. Recognition depends heavily on it:

- place the finger the way you will when unlocking, centered;
- between touches, vary it a little: slightly up, down, left, right, a few
  degrees of rotation;
- lift the finger completely between touches.

In testing, enrolling with extreme positions (tip, base, edges) dropped
recognition to about 50%. Enrolling with natural, near-center placements gave 10/10.

## Use it

- **hyprlock**: use its native fingerprint support (`auth { fingerprint:enabled = true }`)
  rather than adding `pam_fprintd` to its PAM file. If you use `pam_faillock`, note
  that hyprlock aborts the pending password conversation when the fingerprint
  unlocks, and `pam_faillock` counts that as a failed attempt. Three fingerprint
  unlocks within 15 minutes then lock your password for 10 minutes.
- **sudo / polkit / display manager**: add this line before the existing `auth` lines of
  `/etc/pam.d/sudo`, `/etc/pam.d/polkit-1` and your display manager's PAM file:
  ```
  auth  sufficient  pam_fprintd.so max-tries=2 timeout=10
  ```
  Keep a root shell open while you edit PAM files, and test before closing it.

## How it works

See [docs/PROTOCOL.md](docs/PROTOCOL.md). In short:

- the sensor is driven through `spidev` with register writes, and a 108 x 88
  12-bit frame is read with a CRC32;
- the driver detects the finger from the difference with the empty-sensor
  background and captures once the contact settles;
- matching is a masked normalized cross-correlation over translations and
  small rotations, computed with FFTs in parallel threads;
- verification takes about 1 s from touch to unlock on the tested laptop.

## Troubleshooting

```sh
cat /sys/bus/spi/devices/spi-GXFP3200:00/driver_override   # should say: spidev
cat /sys/module/spidev/parameters/bufsiz                    # should say: 65536
journalctl -u fprintd -b                                    # driver errors
```

To get debug output, stop the service and run `G_MESSAGES_DEBUG=all /usr/lib/fprintd`
as root.

## License

LGPL-2.1-or-later, like libfprint. See [LICENSE](LICENSE).
