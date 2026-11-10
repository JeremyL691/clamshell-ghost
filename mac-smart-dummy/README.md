# macOS Smart Dummy Display, Clamshell Guard & Wireless Audio Daemon

`esp32-smart-dummy` is a single-file, zero-dependency Objective-C daemon (`esp32_smart_dummy.m`) for Apple Silicon Macs running macOS 11+. It turns a lid-closed MacBook into an instant-wake, Retina-grade headless workstation without requiring a physical HDMI/USB-C dummy plug, while automatically yielding hardware display pipes within `<50ms` whenever physical monitors are attached.

It supports two operating modes:
- **ESP32 Dock Mode (Default)**: Activates headless clamshell stay-awake and virtual display management whenever the companion ESP32-S3 dock is present over USB UAC2 (`0x303A:0x8001`) or Wi-Fi UDP (`:18796`), and bridges 48 kHz PCM16 wireless audio into macOS CoreAudio via `BlackHole 2ch`.
- **Standalone Mode (`--standalone` / `CLAMSHELL_STANDALONE=1`)**: Pure software Mac-only mode requiring no external hardware. Keeps the MacBook awake in closed-clamshell mode and mounts the HiDPI virtual display whenever connected to AC power and zero physical external monitors are online.

---

## Internal Architecture

### 1. `CGVirtualDisplay` HiDPI Creation & Mirror-Breaking (`EnsureNotMirrored`)

When the MacBook is running headless (`realExtCount == 0`) with stay-awake conditions satisfied, `CreateDummyDisplay` instantiates an in-memory virtual display through the private CoreGraphics `CGVirtualDisplay` API:
- **2x Retina Framebuffer**: Configures `CGVirtualDisplaySettings` with `hiDPI = 1` and exposes both the logical mode (`g_targetWidth x g_targetHeight`, default `1600x1000` 16:10 at 60 Hz) and the 2x backing pixel mode (`3200x2000` at 60 Hz).
- **Explicit HiDPI Mode Selection (`EnsureHiDPIMode1600x1000`)**: Queries `CGDisplayCopyAllDisplayModes` with `kCGDisplayShowDuplicateLowResolutionModes` enabled, locates the exact mode where logical dimensions match `g_targetWidth x g_targetHeight` and backing pixel dimensions match `(g_targetWidth * 2) x (g_targetHeight * 2)`, and locks it in permanently via `CGConfigureDisplayWithDisplayMode`.
- **Bidirectional Mirror-Breaking (`EnsureNotMirrored`)**: If `WindowServer` places the virtual display into a hardware mirror set during lid transitions, `EnsureNotMirrored` inspects `CGDisplayIsInMirrorSet` and clears both `CGDisplayMirrorsDisplay(dummyID)` and any reverse mirror references (`CGDisplayMirrorsDisplay(d) == dummyID`) inside a `CGBeginDisplayConfiguration` transaction.
- **Self-Healing Lifecycle**: Attaches a `terminationHandler` on `CGVirtualDisplayDescriptor` and verifies presence in `CGGetOnlineDisplayList` so that any `WindowServer` reset immediately remounts the virtual display.

### 2. `IOMobileFramebuffer` Apple Silicon DCP Detection & `<50ms` Auto-Yield

Apple Silicon MacBooks (such as M1/M2/M3 Pro) have a fixed number of hardware Display Coprocessor (`DCP`) external display pipes. Leaving a software `CGVirtualDisplay` mounted while plugging in multiple physical monitors can starve hardware display routing or cause layout collisions.

To guarantee zero interference with real monitors:
- **Kernel-Level DCP Inspection (`CountKernelExternalDisplays`)**: Walks the IOKit `IOMobileFramebuffer` registry entries, filters for external pipes (`external == kCFBooleanTrue`), and checks whether `DisplayAttributes` is non-empty. It also filters out the built-in Liquid Retina panel (`CGDisplayIsBuiltin`) and macOS's internal headless fallback placeholder (`0x756E6B6E:0x76697274`).
- **Dual-Path Hotplug Notifications**: Registers both CoreGraphics reconfiguration callbacks (`CGDisplayRegisterReconfigurationCallback`) and IOKit general interest notifications (`IOServiceAddInterestNotification`) directly on every external `IOMobileFramebuffer` service node.
- **`<50ms` Auto-Yield**: The moment `realExtCount >= 1` is detected, `DestroyDummyDisplay` releases the `CGVirtualDisplay` object within `20ms`, freeing all DCP pipes for physical displays. When the last physical monitor is unplugged (`realExtCount == 0`), the virtual HiDPI display is seamlessly remounted.

### 3. Triple-Layer Clamshell Sleep Prevention

When the last external monitor is unplugged from a lid-closed MacBook, `WindowServer` drops `DesktopMode` to `0` and `powerd` evaluates clamshell sleep within ~300 ms. Standard `caffeinate` assertions are ignored by `powerd` during a lid-closed display-unplug transition. `esp32-smart-dummy` prevents unwanted clamshell sleep through three coordinated layers that remain continuously armed on AC power even while physical monitors are attached:

1. **`IOPMrootDomain` Kernel UserClient (`Selector 12` / `kPMSetClamshellSleepState`)**: Opens a direct connection to `IOPMrootDomain` (`IOPMFindPowerManagement`) and invokes `IOConnectCallScalarMethod` with selector `kPMSetClamshellSleepState` (`12`) and scalar input `1` to instruct the kernel power management root domain to inhibit clamshell sleep. Reinforced immediately inside `kIOPMMessageClamshellStateChange` and display reconfiguration callbacks.
2. **`SensorsActive=YES` Power Assertions**: Creates `PreventUserIdleSystemSleep`, `PreventUserIdleDisplaySleep`, and `UserIsActive` assertions via `IOPMAssertionCreateWithProperties` with the undocumented `@"SensorsActive": @YES` property key, accompanied by periodic `IOPMAssertionDeclareUserActivity` display wake tickles (`TickleDisplayWake`). `powerd`'s internal `EvaluateClamshell` routine specifically inspects `SensorsActive` assertions to keep `ClamshellSleepDisabled=1` across lid-closed monitor unplugs.
3. **System Power Veto (`IORegisterForSystemPower` + `IOCancelPowerChange`)**: Registers a system power root port client and intercepts `kIOMessageCanSystemSleep`, vetoing idle/clamshell sleep transitions via `IOCancelPowerChange` whenever stay-awake mode is active.

When AC power is disconnected (`IsOnACPower() == NO`) or the trigger device disconnects, all assertions are released and `SetRootDomainClamshellSleepDisabled(NO)` restores normal macOS battery sleep behavior.

### 4. Lock-Free 48 kHz UDP PCM16 to `BlackHole 2ch` CoreAudio Bridge

In ESP32 Dock Mode, the daemon doubles as a low-latency wireless microphone receiver:
- **Interactive-QoS UDP Socket (`StartUdpListener`)**: Binds to `0.0.0.0:18796` on a `QOS_CLASS_USER_INTERACTIVE` GCD dispatch source, parses `esp32_udp_hdr_t` packets (`magic = 0x4D323345`), replies with `ESP32_PKT_MAC_ACK_CTRL` keepalives, and writes incoming 48 kHz 16-bit mono PCM samples into a 16,384-sample lock-free atomic ring buffer (`g_pcmRingBuf`) using C11 acquire/release semantics.
- **CoreAudio HAL Output Unit (`StartBlackHoleAudioBridge`)**: Locates `BlackHole 2ch` (`BlackHole2ch_UID`) via CoreAudio property queries, locks its nominal sample rate to `48000.0 Hz`, and attaches a real-time `kAudioUnitSubType_HALOutput` render callback (`BlackHoleRenderCallback`). The callback converts `int16_t` samples to stereo `Float32` frames, enforces a 20 ms prebuffer (`960` samples), and caps ring drift at 100 ms (`4800` samples -> `1920` target).
- **Automatic Default Input Switching & Fallback Restoration (`UpdateDefaultAudioInputDevice`)**: Automatically switches `kAudioHardwarePropertyDefaultInputDevice` to `ESP32-S3 Dual Microphone` when connected over USB, or to `BlackHole 2ch` when streaming over Wi-Fi UDP. Prior to switching, it records the user's previous non-virtual input device (`g_savedFallbackInputID`, preferring built-in microphones) and restores it automatically when the ESP32 goes offline.

---

## Build & Installation

### Prerequisites
- macOS 11.0+ on Apple Silicon (M1 / M2 / M3 / M4)
- Xcode Command Line Tools (`xcode-select --install`)
- Optional (for wireless audio bridge in ESP32 Dock Mode): `brew install --cask blackhole-2ch`

### Option A: ESP32 Dock Mode (Default)
Builds the binary, signs it ad-hoc, installs it to `~/.local/bin/esp32-smart-dummy`, generates `~/Library/LaunchAgents/com.waveshare.esp32-smart-dummy.plist`, and bootstraps the LaunchAgent:
```bash
make install
```

### Option B: Standalone Mac-Only Mode (No ESP32 Hardware Required)
Installs and bootstraps the LaunchAgent with `--standalone` enabled in `ProgramArguments` so your MacBook stays awake in closed-clamshell mode and mounts the HiDPI virtual display whenever on AC power with zero external monitors attached:
```bash
make install-standalone
```

### Uninstallation
Stops the LaunchAgent and removes both the plist and installed binary:
```bash
make uninstall
```

---

## Command-Line Reference

```text
Usage: esp32-smart-dummy [OPTIONS]

Options:
  --status              Print current power, display, audio bridge, and daemon state and exit
  --probe-wireless      Print wireless ESP32 UDP presence and IP address and exit
  --standalone          Run in standalone Mac-only mode (activate on AC power without ESP32 hardware;
                        can also be enabled via environment variable CLAMSHELL_STANDALONE=1)
  --resolution <WxH>    Set logical HiDPI resolution in range 800..3840 x 600..2400 (default: 1600x1000,
                        backed by a 2x Retina framebuffer, e.g. 3200x2000)
  --test-create         Force virtual display creation and stay-awake assertions for testing
  -h, --help            Print this usage summary and exit
```

### Example: Query Daemon Status
```bash
./esp32-smart-dummy --status
```
Output fields:
```ini
ESP32_MIC_PRESENT=1
ESP32_USB_PRESENT=0
ESP32_WIFI_PRESENT=1
BLACKHOLE_AUDIO_BRIDGE=1
AC_POWER=1
REAL_EXTERNAL_DISPLAYS=0
DUMMY_DISPLAYS=1
BUILTIN_DISPLAYS=0
STANDALONE_MODE=0
TARGET_RESOLUTION=1600x1000
```

### Runtime Signals
- `SIGUSR1`: Simulate unplugging all physical external monitors (`realExtCount = 0`) to test virtual display activation while monitors are still physically attached.
- `SIGUSR2`: Restore normal physical monitor counting.
- `SIGINT` / `SIGTERM`: Cleanly tear down the virtual display, restore the fallback CoreAudio input device, close UDP/IOKit handles, and exit.
