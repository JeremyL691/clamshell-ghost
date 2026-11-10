# Clamshell Ghost ESP32-S3 Dual-Mode Microphone Firmware

Dedicated low-power USB UAC and Wi-Fi UDP microphone firmware for the **Waveshare ESP32-S3-Touch-AMOLED-1.75C** (`ESP32-S3`, `ES7210` dual-microphone ADC, `AXP2101` PMIC).

Designed to pair with the macOS `esp32-smart-dummy` daemon (`mac-smart-dummy/`) and `BlackHole 2ch` audio bridge so a MacBook in closed-clamshell mode always has an external microphone available over USB or Wi-Fi without opening the lid.

---

## Hardware Architecture & Power Management

### Target Board: Waveshare ESP32-S3-Touch-AMOLED-1.75C
| Subsystem | Controller / Pins | Firmware Configuration |
| :--- | :--- | :--- |
| **Dual Microphone ADC** | Everest `ES7210` over I2C (`SDA=GPIO15`, `SCL=GPIO14`, addr `0x40`) & I2S0 (`MCLK=GPIO16`, `BCLK=GPIO9`, `WS=GPIO45`, `DIN=GPIO10`) | 48 kHz, 16-bit stereo capture (`MIC1` + `MIC2`), mixed in DSP with `0.75x` headroom scaling (`(3 * (m1 + m2)) >> 2`) for +3 dB dual-mic SNR gain without clipping. Default hardware PGA gain step `12` (`34.5 dB`, adjustable `15.0 dB` – `37.5 dB` via host volume). |
| **PMIC & Battery Cut-Off** | X-Powers `AXP2101` (`SDA=GPIO15`, `SCL=GPIO14`, addr `0x34`) | Enables only `DC1` (`3.3V` MCU rail) and `ALDO1` (`3.3V` analog mic supply). Sets charger target voltage to `4.1V` (`200 mA` CC, `25 mA` termination). Polls `Reg 0x00` bit 5 (`VBUS good`) every 250 ms; if USB `VBUS` is absent for 1.5 seconds (6 ticks), writes `Reg 0x10` bit 0 to power off the PMIC completely so the LiPo battery never drains when the host powers down or is unplugged. |
| **Display & Speaker Shutdown** | AMOLED reset (`GPIO1`, `GPIO39`), Touch reset (`GPIO2`, `GPIO40`), Speaker PA enable (`GPIO46`) | All display/touch reset lines and the speaker amplifier enable pin are held low with pull-downs from boot, and all unused AXP2101 LDO rails are disabled for zero display/speaker power draw. |
| **ROM Download Escape Hatch** | `BOOT` button (`GPIO0`) | Because TinyUSB UAC takes over the native USB OTG pins (`GPIO19`/`GPIO20`), holding the `BOOT` button for 3 seconds (12 housekeeping ticks) sets `RTC_CNTL_FORCE_DOWNLOAD_BOOT` and restarts the chip into ROM USB-Serial-JTAG download mode for reflashing. |

---

## Dual-Mode Operation (USB UAC + Wi-Fi UDP)

The firmware runs both transports concurrently and transitions automatically based on whether macOS is actively reading from the USB Audio Class interface:

1. **USB Wired Mode (Active UAC Stream)**:
   - Enumerates as a driverless macOS USB Audio Class (UAC) 48 kHz 16-bit mono microphone (`Waveshare ESP32-S3 Dual Microphone`).
   - While the host is reading USB audio frames (`uac_mic_input_cb` called within the last 150 ms), `wifi_audio_task` suppresses duplicate I2S reads and sends lightweight 12-byte UDP heartbeat packets (`ESP32_PKT_HEARTBEAT`) every 500 ms with bit 1 of `flags` (`usb_uac_active`) set.
2. **Wi-Fi Wireless Mode (UDP 48 kHz Stream)**:
   - Whenever USB UAC is idle or only connected to a power brick, `wifi_audio_task` reads 10 ms stereo frames from the `ES7210`, mixes them to 480 mono 16-bit samples (`960` bytes), and transmits `972`-byte UDP audio datagrams (`ESP32_PKT_AUDIO_PCM16`) on UDP port `18796` with Wi-Fi power save disabled (`WIFI_PS_NONE`).
3. **Automatic Mac Discovery & Unicast Lock**:
   - Before receiving an acknowledgment from the Mac (or if no ACK has been received within the last 5 seconds), the firmware sends periodic UDP broadcast discovery heartbeats to `255.255.255.255:18796` every 2 seconds.
   - As soon as the Mac responds with a `MAC_ACK_CTRL` (`0x03`) packet on port `18796`, the ESP32 locks `s_mac_addr` onto the Mac's source IPv4 address for direct unicast streaming and applies any piggybacked PGA volume percentage (`sample_cnt` 1–100%).

---

## UDP Packet Protocol (Port `18796`)

Every UDP datagram begins with the packed 12-byte little-endian header `esp32_udp_hdr_t`:

```c
typedef struct __attribute__((packed)) {
    uint32_t magic;       /* 0x4D323345 ('E', '3', '2', 'M' in little-endian) */
    uint8_t  pkt_type;    /* 0x01 = AUDIO_PCM16, 0x02 = HEARTBEAT, 0x03 = MAC_ACK_CTRL */
    uint8_t  flags;       /* bit 0 (0x01): muted, bit 1 (0x02): usb_uac_active */
    uint16_t sample_cnt;  /* AUDIO_PCM16: 480 samples; HEARTBEAT: 0; MAC_ACK_CTRL: 1..100% volume */
    uint32_t seq;         /* Monotonically increasing packet sequence number */
} esp32_udp_hdr_t;
```

- **Audio Packet (`0x01`)**: `12`-byte header + `480 * sizeof(int16_t)` (`960` bytes) of 48 kHz 16-bit signed little-endian mono PCM = `972` bytes total per 10 ms frame.
- **Heartbeat Packet (`0x02`)**: `12`-byte header only (`sample_cnt = 0`).
- **Mac ACK / Control Packet (`0x03`, Mac -> ESP32)**: `12`-byte header sent by the Mac receiver to lock unicast routing and optionally set the hardware ES7210 PGA gain (`sample_cnt` in `1..100`).

---

## Wi-Fi Configuration

Wi-Fi station credentials are loaded at boot using a 3-tier priority order:

1. **Priority 1 — Custom NVS Namespace `"wifi"`**:
   Reads keys `"ssid"` / `"password"` (or `"ssid0"` / `"password0"`) from the `"wifi"` NVS namespace. This allows flashing firmware updates without overwriting credentials already stored in NVS on the board.
2. **Priority 2 — Standard ESP-IDF Wi-Fi NVS**:
   Falls back to credentials persisted by `esp_wifi_set_config()` in the default NVS partition.
3. **Priority 3 — Compile-Time Kconfig Fallback**:
   If NVS contains no SSID, uses `CONFIG_ESP32_WIFI_SSID` and `CONFIG_ESP32_WIFI_PASSWORD` configured via `idf.py menuconfig`.

To configure fallback credentials or an initial target IP before building:

```bash
idf.py menuconfig
# Navigate to: "Clamshell Ghost ESP32 Microphone Configuration"
#   -> Fallback Wi-Fi SSID
#   -> Fallback Wi-Fi Password
#   -> Initial Target Mac IPv4 Address (defaults to 255.255.255.255 for auto-discovery)
```

---

## Build & Flash

Requires **ESP-IDF v5.5+** targeting `esp32s3`.

```bash
# Set target (if starting from a clean checkout)
idf.py set-target esp32s3

# Build firmware
idf.py build

# Flash to board
# If the board is already running USB UAC firmware, hold the BOOT button for 3 seconds
# until it re-enumerates in ROM USB-Serial-JTAG download mode, then run:
idf.py -p /dev/cu.usbmodem* flash
```
