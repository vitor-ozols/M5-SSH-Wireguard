# SSH Client for M5Cardputer-Adv + ILI9341

A full-featured SSH client for the [M5Stack Cardputer-Adv](https://docs.m5stack.com/en/core/Cardputer) (ESP32-S3) using an external 2.8" ILI9341 SPI display (320×240), with WireGuard VPN support, multi-profile management, and an ANSI terminal emulator.


---

## Features

- **SSH terminal** with ANSI/VT100 emulation — colours, cursor movement, scroll regions, alternate screen buffer (nano, htop, vim)
- **WireGuard VPN** — per-profile tunnel with seamless config switching via `ESP.restart()` and RTC memory auto-resume
- **Multiple SSH profiles** — stored on SD card, each with its own host, user, port, and optional WireGuard config
- **WireGuard config import** — drop standard `.conf` files onto the SD card, pick them from a menu
- **Two font sizes** — toggle in-session with `Fn+F` (53×25 or 26×12 characters on the external display)
- **Dual-display status** — the external ILI9341 shows menus/terminal while the built-in Cardputer LCD shows battery, WiFi, IP, mode, SSH target, session timer, WireGuard state, heap, and clock
- **WiFi manager** — scan, connect, save credentials; auto-connect on boot
- **Settings** — screen timeout, SSH idle timeout, brightness, keep-alive, password display mode
- **Remembered usernames** — recently used SSH usernames offered as quick picks

---

## Hardware

| | |
|---|---|
| **Board** | M5Stack Cardputer (ESP32-S3, 4 MB flash) |
| **Display** | External 2.8" ILI9341 SPI display, 320×240 |
| **Status display** | Built-in Cardputer LCD, used as a live status panel |
| **Storage** | microSD card (FAT32) |
| **Quit session** | `Fn+Q` or the **G0** side button |

### ILI9341 Wiring

Cardputer-Adv EXT connector to ILI9341:

| ILI9341 | Cardputer-Adv EXT | GPIO | Notes |
|---|---:|---:|---|
| VCC | PIN 15 | - | 3.3 V |
| GND | PIN 11 | - | Ground |
| CS | PIN 13 | G5 | LCD chip select |
| RST / RESET | PIN 1 | G3 | LCD reset |
| DC / RS | PIN 5 | G6 | Data/command |
| SDI / MOSI | PIN 9 | G14 | SPI MOSI |
| SCK / CLK | PIN 7 | G40 | SPI clock |
| LED / BLK | 3.3 V | - | Backlight always on |
| SDO / MISO | - | - | Not used by LCD |

The LCD and the Cardputer-Adv SD card share SPI3. SD card CS is GPIO 12.

---

## Requirements

### Arduino IDE Board Setup

1. Add the M5Stack board manager URL:
   ```
   https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
   ```
2. Install: **Tools → Board → M5Stack → M5Cardputer**
3. Set: **Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)**

### Library Versions

| Library | Version |
|---|---|
| M5Stack board manager | ≥ 3.2.6 (ESP-IDF 5.4) |
| M5Cardputer | ≥ 1.1.1 |
| M5Unified | ≥ 0.2.8 |
| M5GFX | ≥ 0.2.17 |
| WireGuard-ESP32-bis | ZIP from [issue #45](https://github.com/ciniml/WireGuard-ESP32-Arduino/issues/45) |
| LibSSH-ESP32 | ZIP from [github.com/ewpa/LibSSH-ESP32](https://github.com/ewpa/LibSSH-ESP32) |

Install WireGuard-ESP32-bis and LibSSH-ESP32 via **Sketch → Include Library → Add .ZIP Library**.

---

## SD Card Layout

```
/SSHAdv/
├── wifi.cfg        — saved WiFi credentials
├── users.cfg       — remembered SSH usernames
├── settings.cfg    — all app settings
├── 0.prof          — SSH profile 0
├── 1.prof          — SSH profile 1
└── wg/
    ├── home.conf   — WireGuard config (standard format)
    └── work.conf
```

WireGuard `.conf` files use the standard format exported by any WireGuard server or client.

---

## Flashing

The Cardputer-Adv requires manual download mode:

1. Switch the power switch **OFF**
2. Hold the **G0** button
3. Connect USB-C
4. Release G0
5. Flash from Arduino IDE as normal

---

## Navigation

### Menus

| Key | Action |
|---|---|
| `;` | Up |
| `.` | Down |
| `,` | Back |
| `/` | Forward / confirm |
| `Enter` | Select |

### SSH Terminal

| Key | Action |
|---|---|
| All keys | Type normally |
| `Fn + ; . , /` | Arrow keys (↑ ↓ ← →) |
| `Fn + Q` | Quit session |
| `Fn + F` | Toggle font size (53×25 ↔ 26×12) |
| `Ctrl + letter` | Send control character (`^C`, `^D`, `^Z` …) |
| `Ctrl + [` | Send ESC (for vim) |
| `Tab` | Tab / shell completion |
| **G0 button** | Quit session |

---

## WireGuard Notes

- Each SSH profile can optionally use a WireGuard tunnel
- Import `.conf` files to `/SSHAdv/wg/` on the SD card and select them in the profile editor
- **Reconnecting the same profile** reuses the existing tunnel instantly — no re-handshake
- **Switching to a different WG config** after an active session triggers a soft reboot (`~2s`) and auto-resumes into the new session via RTC memory
- **Switching WG config before any connection** does a clean teardown and restart without rebooting

---

## Terminal Emulation

Supported escape sequences:

- Cursor movement: `ESC[A/B/C/D/E/F/G/H`
- Erase: `ESC[J` (screen), `ESC[K` (line)
- Insert/delete: `ESC[L/M/P/@`
- Scroll region: `ESC[r`, `ESC[S/T`
- SGR colours: 16-colour ANSI, bold, reverse, 256-colour and truecolor approximation
- Alternate screen buffer: `ESC[?1049h/l` — nano, htop, vim work correctly
- Cursor show/hide: `ESC[?25h/l`
- Save/restore cursor: `ESC[s/u`, `ESC 7/8`
- OSC title sequences silently swallowed
- UTF-8 decoded; box-drawing characters mapped to ASCII equivalents

---

## Building & Running

1. Clone or download the sketch folder to `SSH/ssh_client_adv/`
2. Open `ssh_client_adv.ino` in Arduino IDE
3. Select board, partition scheme, and port as above
4. Upload
5. Insert a FAT32-formatted microSD card
6. On first boot the app creates `/SSHAdv/` automatically

### PlatformIO Build

This folder also contains a PlatformIO project:

```bash
pio run
```

The generated app binary is `.pio/build/m5stack-cardputer-adv/firmware.bin`.

---

## License

MIT
