# macOS Smart Dummy Display, Clamshell Guard & Wireless Audio Daemon

`esp32-smart-dummy` is a single-file, zero-dependency Objective-C daemon (`esp32_smart_dummy.m`) for Apple Silicon Macs running macOS 11+. It turns a lid-closed MacBook into an instant-wake, Retina-grade headless workstation without requiring a physical HDMI/USB-C dummy plug, while automatically yielding hardware display pipes within `<20ms` when physical monitors are attached and remounting the virtual display in `<5ms` upon unplug.

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

### 2. Kernel-Priority `IOMobileFramebuffer` DCP Detection & `<5ms` Unplug Fast-Path

Apple Silicon MacBooks (such as M1/M2/M3/M4) utilize hardware Display Coprocessor (`DCP`) external display pipes. Leaving a software `CGVirtualDisplay` mounted while plugging in multiple physical monitors can starve hardware display routing or cause layout collisions. Conversely, when unplugging the last physical display in clamshell mode, any latency in detecting monitor removal risks triggering macOS lock screen before a virtual display can be instantiated.

To guarantee zero interference with real monitors and instantaneous transition to headless mode:
- **Direct Kernel-Level DCP Inspection (`CountKernelExternalDisplays`)**: Walks IOKit `IOMobileFramebuffer` registry entries, inspecting the hardware external pipes (`external == kCFBooleanTrue`) and validating active connection states via non-empty `DisplayAttributes`. Built-in Liquid Retina panels (`CGDisplayIsBuiltin`) and internal fallback placeholders (`0x756E6B6E:0x76697274`) are excluded.
- **Kernel-State Priority Over WindowServer**: Higher-level CoreGraphics APIs (`CGGetOnlineDisplayList`) can lag behind actual hardware disconnects while `WindowServer` recomputes display arrangements. By giving direct priority to `CountKernelExternalDisplays`, the daemon senses when physical display pipes drop to zero at the kernel hardware layer, triggering virtual display mounting in `<5ms`—well before `WindowServer` registers a zero-display state.
- **Dual-Path Hotplug Notifications**: Registers both CoreGraphics reconfiguration callbacks (`CGDisplayRegisterReconfigurationCallback`) and IOKit general interest notifications (`IOServiceAddInterestNotification`) directly on every external `IOMobileFramebuffer` service node.
- **Automatic Pipe Yielding & Seamless Remount**: When `realExtCount >= 1` is detected, `DestroyDummyDisplay` releases the `CGVirtualDisplay` object within `20ms`, freeing all DCP pipes for physical displays. When the last physical monitor is unplugged (`realExtCount == 0`), the virtual HiDPI display is mounted instantly.

### 3. Continuous Pre-Armed Power Assertions & Zero-Lock Unplug Protection

When the last external monitor is unplugged from a lid-closed MacBook, two critical race conditions in macOS can trigger a `loginwindow` session lock:
1. **`powerd` Instant Display Sleep Emission**: At the exact millisecond of physical disconnection, `powerd` evaluates active power assertions. If `PreventUserIdleDisplaySleep` with `SensorsActive=YES` is not *already held*, `powerd` immediately emits `Display is turned off`, instructing `loginwindow` to lock the user session before a replacement display can be initialized.
2. **`coreaudiod` Assertion Teardown Storm**: Unplugging an HDMI, DisplayPort, or Thunderbolt monitor causes `coreaudiod` to tear down display audio endpoints, flooding `powerd` with ~80 audio power assertion release calls. This congests `powerd`'s incoming XPC queue for 1.5–2.5 seconds. If a daemon attempts on-demand assertion acquisition (`IOPMAssertionCreateWithProperties`) during this hotplug window, the XPC call blocks in the queue past the critical deadline.

`esp32-smart-dummy` eliminates both failure modes by maintaining continuous pre-armed protection:
- **Continuous 4-Assertion Pre-Arming**: As long as stay-awake conditions are satisfied on AC power, the daemon holds all four required assertions continuously, even while physical monitors are attached:
  1. `PreventUserIdleSystemSleep` with `SensorsActive = YES` (sets `powerd` internal `0x08 AppliesOnLidClose` bit)
  2. `PreventUserIdleDisplaySleep` with `SensorsActive = YES`
  3. `UserIsActive` with `SensorsActive = YES`
  4. Periodic 4-second `IOPMAssertionDeclareUserActivity` display wake tickles (`TickleDisplayWake`)
- **Direct `IOPMrootDomain` Kernel UserClient (`Selector 12` / `kPMSetClamshellSleepState`)**: Calls scalar method `12` on the power management user client with scalar input `1` to directly lock kernel clamshell sleep disabled, reinforced across display and clamshell state events.
- **System Power Veto (`IORegisterForSystemPower` + `IOCancelPowerChange`)**: Intercepts `kIOMessageCanSystemSleep` power events and actively cancels unwanted sleep attempts.
- **Zero XPC Overhead During Hotplug**: Because all assertions are pre-armed, unplugging an external monitor requires zero blocking calls to `powerd`, bypassing `coreaudiod`'s assertion queue congestion entirely.

When AC power is disconnected (`IsOnACPower() == NO`) or the trigger device goes offline, all assertions are released and `SetRootDomainClamshellSleepDisabled(NO)` restores standard macOS battery clamshell sleep.

### 4. Display-First Execution Architecture & CoreAudio Decoupling

Hotplug evaluation in `EvaluateState()` is strictly staged to prevent auxiliary subsystem latencies from interfering with display recovery:
- **Synchronous 0ms Callback Dispatch**: `DisplayReconfigCallback` and `KernelFramebufferInterestCallback` execute `EvaluateState()` immediately on the main thread (0 ms delay), followed by a debounced 150 ms secondary verification (`ScheduleEvaluation`). Callbacks triggered by the daemon's own virtual display (`display == g_virtualDisplay.displayID`) are ignored to avoid feedback loops.
- **Re-entrancy Protection**: A strict guard (`s_inEvaluateState`) prevents concurrent or re-entrant evaluations while display transactions are in progress.
- **Display Critical Path First**: Phase 1 of `EvaluateState()` handles only display topology evaluation, power assertion maintenance, and `CGVirtualDisplay` mounting or teardown.
- **Decoupled Audio & Telemetry Maintenance**: Phase 2 operations—including CoreAudio HAL IPC (`UpdateDefaultAudioInputDevice`), `BlackHole 2ch` health verification, active wireless probing, and telemetry logging—are executed only after the display pipeline is fully settled.

### 5. Lock-Free 48 kHz UDP PCM16 to `BlackHole 2ch` CoreAudio Bridge

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
