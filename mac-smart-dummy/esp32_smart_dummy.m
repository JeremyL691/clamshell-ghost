#import <Foundation/Foundation.h>
#import <AppKit/AppKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <CoreAudio/CoreAudio.h>
#import <AudioToolbox/AudioToolbox.h>
#import <IOKit/IOKitLib.h>
#import <IOKit/IOMessage.h>
#import <IOKit/usb/IOUSBLib.h>
#import <IOKit/ps/IOPowerSources.h>
#import <IOKit/ps/IOPSKeys.h>
#import <IOKit/pwr_mgt/IOPM.h>
#import <IOKit/pwr_mgt/IOPMLib.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdatomic.h>
#include <os/lock.h>

#ifndef kIOPMMessageClamshellStateChange
#define kIOPMMessageClamshellStateChange ((uint32_t)0xE0000100)
#endif

/* Private CoreGraphics CGVirtualDisplay declarations (macOS 11+) */
@interface CGVirtualDisplayMode : NSObject
- (instancetype)initWithWidth:(unsigned int)width
                       height:(unsigned int)height
                  refreshRate:(double)refreshRate;
@end

@interface CGVirtualDisplaySettings : NSObject
@property(nonatomic, assign) unsigned int hiDPI;
@property(nonatomic, strong) NSArray<CGVirtualDisplayMode *> *modes;
@end

@interface CGVirtualDisplayDescriptor : NSObject
@property(nonatomic, assign) unsigned int vendorID;
@property(nonatomic, assign) unsigned int productID;
@property(nonatomic, assign) unsigned int serialNum;
@property(nonatomic, strong) NSString *name;
@property(nonatomic, assign) CGSize sizeInMillimeters;
@property(nonatomic, assign) unsigned int maxPixelsWide;
@property(nonatomic, assign) unsigned int maxPixelsHigh;
@property(nonatomic, assign) CGPoint redPrimary;
@property(nonatomic, assign) CGPoint greenPrimary;
@property(nonatomic, assign) CGPoint bluePrimary;
@property(nonatomic, assign) CGPoint whitePoint;
@property(nonatomic, strong) dispatch_queue_t queue;
@property(nonatomic, copy) void (^terminationHandler)(id, id);
- (dispatch_queue_t)dispatchQueue;
- (void)setDispatchQueue:(dispatch_queue_t)queue;
@end

@interface CGVirtualDisplay : NSObject
@property(nonatomic, readonly) unsigned int displayID;
- (instancetype)initWithDescriptor:(CGVirtualDisplayDescriptor *)descriptor;
- (BOOL)applySettings:(CGVirtualDisplaySettings *)settings;
@end

/* Constants identifying our ESP32 USB Microphone & Virtual Dummy Display */
#define ESP32_USB_VID               0x303A
#define ESP32_USB_PID               0x8001
#define DUMMY_DISPLAY_VID           0x303A
#define DUMMY_DISPLAY_PID           0x8001
#define DUMMY_DISPLAY_SERIAL        0x175C1600

/* Wireless Wi-Fi UDP 48kHz Audio & Heartbeat Protocol (Port 18796) */
#define ESP32_UDP_PORT              18796
#define ESP32_UDP_MAGIC             0x4D323345UL /* 'E','3','2','M' */
#define ESP32_PKT_AUDIO_PCM16       0x01
#define ESP32_PKT_HEARTBEAT         0x02
#define ESP32_PKT_MAC_ACK_CTRL      0x03

typedef struct __attribute__((packed)) {
    uint32_t magic;       /* 0x4D323345 */
    uint8_t  pkt_type;    /* 0x01=AUDIO, 0x02=HEARTBEAT, 0x03=MAC_ACK_CTRL */
    uint8_t  flags;       /* bit 0: muted, bit 1: usb_uac_active */
    uint16_t sample_cnt;  /* mono int16_t samples following header (e.g. 480) */
    uint32_t seq;         /* sequence number */
} esp32_udp_hdr_t;

/* Lock-free 48kHz PCM16 Ring Buffer (16384 samples = ~341ms @ 48kHz) */
#define PCM_RING_CAPACITY           16384U
#define PCM_RING_MASK               (PCM_RING_CAPACITY - 1U)
#define PCM_MAX_LATENCY_SAMPLES     4800U   /* 100ms @ 48kHz cap threshold */
#define PCM_TARGET_LATENCY_SAMPLES  1920U   /* 40ms @ 48kHz target latency */
#define PCM_PREBUFFER_SAMPLES       960U    /* 20ms @ 48kHz prebuffer */

#define STATE_FILE_PATH             "/tmp/esp32-smart-dummy.state"
#define STATE_FILE_TMP_PATH         "/tmp/esp32-smart-dummy.state.tmp"

/* macOS internal headless fallback virtual display ('unkn' / 'virt') */
#define FALLBACK_DISPLAY_VID        0x756E6B6E
#define FALLBACK_DISPLAY_PID        0x76697274

/* IOPMrootDomain UserClient selector for Clamshell Sleep Disable */
#define kPMSetClamshellSleepState   12

static CGVirtualDisplay *g_virtualDisplay = nil;
static CFAbsoluteTime g_dummyCreatedAt = 0;
static IOPMAssertionID g_sysSleepAssertion = kIOPMNullAssertionID;
static IOPMAssertionID g_dispSleepAssertion = kIOPMNullAssertionID;
static IOPMAssertionID g_userActAssertion = kIOPMNullAssertionID;
static IOPMAssertionID g_userTickleAssertion = kIOPMNullAssertionID;
static io_connect_t g_rootDomainConn = IO_OBJECT_NULL;
static io_object_t g_rootDomainNotifier = IO_OBJECT_NULL;
static io_connect_t g_sysPowerConn = IO_OBJECT_NULL;
static io_object_t g_sysPowerNotifier = IO_OBJECT_NULL;
static IONotificationPortRef g_sysPowerNotifyPort = NULL;
static io_object_t g_fbNotifiers[8];
static uint32_t g_fbNotifierCount = 0;
static BOOL g_clamshellStayAwakeActive = NO;

static BOOL g_forceCreateForTest = NO;
static BOOL g_simulateZeroRealMonitors = NO;
static BOOL g_isDaemonMode = NO;
static BOOL g_standaloneMode = NO;
static unsigned int g_targetWidth = 1600;
static unsigned int g_targetHeight = 1000;
static dispatch_source_t g_evalTimer = nil;
static dispatch_source_t g_periodicTimer = nil;
static NSMutableArray<dispatch_source_t> *g_signalSources = nil;

/* Wireless UDP & CoreAudio BlackHole 2ch Bridge state */
static int16_t g_pcmRingBuf[PCM_RING_CAPACITY];
static _Atomic uint32_t g_pcmWriteIdx = 0;
static _Atomic uint32_t g_pcmReadIdx = 0;
static _Atomic double g_lastWirelessRxTime = 0.0;
static _Atomic double g_lastAudioPktTime = 0.0;
static _Atomic uint64_t g_wirelessPktCount = 0;
static _Atomic uint8_t g_lastWirelessFlags = 0;
static char g_lastEsp32Ip[64] = {0};
static os_unfair_lock g_ipLock = OS_UNFAIR_LOCK_INIT;
static os_unfair_lock g_stateFileLock = OS_UNFAIR_LOCK_INIT;

static int g_udpSock = -1;
static dispatch_queue_t g_udpQueue = nil;
static dispatch_source_t g_udpReadSource = nil;
static CFAbsoluteTime g_lastUdpAckTime = 0.0;
static CFAbsoluteTime g_lastStateWriteFromUdp = 0.0;

static AudioComponentInstance g_blackHoleAudioUnit = NULL;
static _Atomic bool g_blackHoleBridgeActive = false;
static BOOL g_pcmPrebuffering = YES;

/* State tracking to avoid duplicate log spam */
static int g_lastRealExtCount = -1;
static int g_lastMicPresent = -1;
static int g_lastUsbMicPresent = -1;
static int g_lastWifiMicPresent = -1;
static int g_lastOnAC = -1;
static int g_lastDummyActive = -1;
static int g_lastKeepAwake = -1;

static void ScheduleEvaluation(NSString *trigger, uint64_t delayMs);

static void LogMsg(NSString *fmt, ...) {
    @autoreleasepool {
        va_list args;
        va_start(args, fmt);
        NSString *msg = [[NSString alloc] initWithFormat:fmt arguments:args];
        va_end(args);
        static NSDateFormatter *df = nil;
        static dispatch_once_t onceToken;
        dispatch_once(&onceToken, ^{
            df = [[NSDateFormatter alloc] init];
            [df setDateFormat:@"yyyy-MM-dd HH:mm:ss.SSS"];
        });
        fprintf(stdout, "[%s] [esp32-smart-dummy] %s\n",
                [[df stringFromDate:[NSDate date]] UTF8String],
                [msg UTF8String]);
        fflush(stdout);
    }
}

static void WriteDaemonStateFile(void) {
    if (!g_isDaemonMode) {
        return;
    }
    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    CFAbsoluteTime lastRx = atomic_load_explicit(&g_lastWirelessRxTime, memory_order_relaxed);
    uint64_t pktCnt = atomic_load_explicit(&g_wirelessPktCount, memory_order_relaxed);
    bool bhActive = atomic_load_explicit(&g_blackHoleBridgeActive, memory_order_relaxed);
    BOOL wifiPresent = (lastRx > 0.0 && (now - lastRx) < 5.0);

    char ipBuf[64] = {0};
    os_unfair_lock_lock(&g_ipLock);
    strncpy(ipBuf, g_lastEsp32Ip, sizeof(ipBuf) - 1);
    os_unfair_lock_unlock(&g_ipLock);

    char buf[512];
    int len = snprintf(buf, sizeof(buf),
                       "updated_at=%.3f\n"
                       "last_wireless_rx=%.3f\n"
                       "wireless_present=%d\n"
                       "blackhole_bridge=%d\n"
                       "wireless_pkt_count=%llu\n"
                       "esp32_ip=%s\n",
                       now,
                       lastRx,
                       wifiPresent ? 1 : 0,
                       bhActive ? 1 : 0,
                       (unsigned long long)pktCnt,
                       ipBuf[0] ? ipBuf : "none");
    if (len <= 0 || len >= (int)sizeof(buf)) {
        return;
    }

    os_unfair_lock_lock(&g_stateFileLock);
    int fd = open(STATE_FILE_TMP_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0) {
        ssize_t written = write(fd, buf, (size_t)len);
        (void)written;
        close(fd);
        rename(STATE_FILE_TMP_PATH, STATE_FILE_PATH);
    }
    os_unfair_lock_unlock(&g_stateFileLock);
}

static BOOL ReadDaemonStateFile(CFAbsoluteTime *outLastWirelessRx, BOOL *outBhBridge, char *outIp, size_t ipLen) {
    NSString *content = [NSString stringWithContentsOfFile:@(STATE_FILE_PATH)
                                                  encoding:NSUTF8StringEncoding
                                                     error:nil];
    if (!content) {
        return NO;
    }
    CFAbsoluteTime updatedAt = 0.0;
    CFAbsoluteTime lastRx = 0.0;
    int bhBridge = 0;
    for (NSString *line in [content componentsSeparatedByCharactersInSet:[NSCharacterSet newlineCharacterSet]]) {
        if ([line hasPrefix:@"updated_at="]) {
            updatedAt = [[line substringFromIndex:11] doubleValue];
        } else if ([line hasPrefix:@"last_wireless_rx="]) {
            lastRx = [[line substringFromIndex:17] doubleValue];
        } else if ([line hasPrefix:@"blackhole_bridge="]) {
            bhBridge = [[line substringFromIndex:17] intValue];
        } else if ([line hasPrefix:@"esp32_ip="] && outIp && ipLen > 0) {
            NSString *ip = [line substringFromIndex:9];
            strncpy(outIp, [ip UTF8String], ipLen - 1);
            outIp[ipLen - 1] = '\0';
        }
    }
    if (updatedAt <= 0.0 || (CFAbsoluteTimeGetCurrent() - updatedAt) > 10.0) {
        return NO;
    }
    if (outLastWirelessRx) *outLastWirelessRx = lastRx;
    if (outBhBridge) *outBhBridge = (bhBridge != 0);
    return YES;
}

/* Check if ESP32-S3 Dual Microphone (VID 0x303A, PID 0x8001) is plugged into USB */
static BOOL IsEsp32MicPresent(void) {
    CFMutableDictionaryRef matching = IOServiceMatching(kIOUSBDeviceClassName);
    if (!matching) {
        return NO;
    }
    io_iterator_t iter = 0;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iter) != KERN_SUCCESS) {
        return NO;
    }

    BOOL found = NO;
    io_service_t service;
    while ((service = IOIteratorNext(iter)) != 0) {
        CFNumberRef vidRef = IORegistryEntryCreateCFProperty(service, CFSTR(kUSBVendorID), kCFAllocatorDefault, 0);
        CFNumberRef pidRef = IORegistryEntryCreateCFProperty(service, CFSTR(kUSBProductID), kCFAllocatorDefault, 0);
        int vid = 0, pid = 0;
        if (vidRef) {
            CFNumberGetValue(vidRef, kCFNumberIntType, &vid);
            CFRelease(vidRef);
        }
        if (pidRef) {
            CFNumberGetValue(pidRef, kCFNumberIntType, &pid);
            CFRelease(pidRef);
        }
        IOObjectRelease(service);
        if (vid == ESP32_USB_VID && pid == ESP32_USB_PID) {
            found = YES;
            break;
        }
    }
    IOObjectRelease(iter);
    return found;
}

/* Check if ESP32-S3 Wireless UDP stream / heartbeat was received within the active window */
static BOOL IsEsp32WirelessPresent(void) {
    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    if (g_isDaemonMode) {
        CFAbsoluteTime lastRx = atomic_load_explicit(&g_lastWirelessRxTime, memory_order_relaxed);
        return (lastRx > 0.0 && (now - lastRx) < 12.0);
    }
    CFAbsoluteTime stateLastRx = 0.0;
    if (ReadDaemonStateFile(&stateLastRx, NULL, NULL, 0)) {
        return (stateLastRx > 0.0 && (now - stateLastRx) < 12.0);
    }
    return NO;
}

/* Extended 5-minute wireless holdover when headless (realExtCount == 0) so transient Wi-Fi jitter
 * or a 2-minute nightly router reboot never puts a headless clamshell Mac to sleep. */
static BOOL IsEsp32WirelessHoldActive(BOOL isHeadless) {
    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    CFAbsoluteTime lastRx = atomic_load_explicit(&g_lastWirelessRxTime, memory_order_relaxed);
    if (lastRx <= 0.0) {
        return NO;
    }
    double timeoutSec = isHeadless ? 300.0 : 12.0;
    return ((now - lastRx) < timeoutSec);
}

static void SendActiveWirelessProbeIfNeeded(void) {
    if (g_udpSock < 0) {
        return;
    }
    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    CFAbsoluteTime lastRx = atomic_load_explicit(&g_lastWirelessRxTime, memory_order_relaxed);
    if (lastRx <= 0.0 || (now - lastRx) < 2.0 || (now - lastRx) > 300.0) {
        return;
    }
    char targetIp[64] = {0};
    os_unfair_lock_lock(&g_ipLock);
    strncpy(targetIp, g_lastEsp32Ip, sizeof(targetIp) - 1);
    os_unfair_lock_unlock(&g_ipLock);
    if (targetIp[0] == '\0' || strcmp(targetIp, "0.0.0.0") == 0) {
        return;
    }
    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons(ESP32_UDP_PORT);
    if (inet_pton(AF_INET, targetIp, &dst.sin_addr) == 1) {
        esp32_udp_hdr_t probe = {
            .magic = ESP32_UDP_MAGIC,
            .pkt_type = ESP32_PKT_MAC_ACK_CTRL,
            .flags = 0,
            .sample_cnt = 0,
            .seq = 0
        };
        sendto(g_udpSock, &probe, sizeof(probe), 0, (const struct sockaddr *)&dst, sizeof(dst));
    }
}

/* CoreAudio Device Discovery & BlackHole 2ch Loopback Audio Bridge */
static UInt32 GetAudioDeviceChannelCount(AudioDeviceID devID, AudioObjectPropertyScope scope) {
    AudioObjectPropertyAddress addr = {
        kAudioDevicePropertyStreamConfiguration,
        scope,
        0
    };
    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(devID, &addr, 0, NULL, &dataSize) != noErr || dataSize == 0) {
        return 0;
    }
    AudioBufferList *bufList = (AudioBufferList *)malloc(dataSize);
    if (!bufList) {
        return 0;
    }
    UInt32 channels = 0;
    if (AudioObjectGetPropertyData(devID, &addr, 0, NULL, &dataSize, bufList) == noErr) {
        for (UInt32 i = 0; i < bufList->mNumberBuffers; i++) {
            channels += bufList->mBuffers[i].mNumberChannels;
        }
    }
    free(bufList);
    return channels;
}

static AudioDeviceID FindAudioDeviceByUID(CFStringRef targetUID) {
    if (!targetUID) {
        return kAudioObjectUnknown;
    }
    AudioObjectPropertyAddress propAddr = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &propAddr, 0, NULL, &dataSize) != noErr || dataSize == 0) {
        return kAudioObjectUnknown;
    }
    UInt32 deviceCount = dataSize / sizeof(AudioDeviceID);
    AudioDeviceID *devices = (AudioDeviceID *)calloc(deviceCount, sizeof(AudioDeviceID));
    if (!devices) {
        return kAudioObjectUnknown;
    }
    AudioDeviceID foundID = kAudioObjectUnknown;
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &propAddr, 0, NULL, &dataSize, devices) == noErr) {
        for (UInt32 i = 0; i < deviceCount; i++) {
            CFStringRef uid = NULL;
            UInt32 uidSize = sizeof(CFStringRef);
            AudioObjectPropertyAddress uidAddr = {
                kAudioDevicePropertyDeviceUID,
                kAudioObjectPropertyScopeGlobal,
                0
            };
            if (AudioObjectGetPropertyData(devices[i], &uidAddr, 0, NULL, &uidSize, &uid) == noErr && uid) {
                if (CFEqual(uid, targetUID)) {
                    foundID = devices[i];
                }
                CFRelease(uid);
                if (foundID != kAudioObjectUnknown) {
                    break;
                }
            }
        }
    }
    free(devices);
    return foundID;
}

static AudioDeviceID FindBlackHoleDeviceID(void) {
    AudioDeviceID byUID = FindAudioDeviceByUID(CFSTR("BlackHole2ch_UID"));
    if (byUID != kAudioObjectUnknown) {
        return byUID;
    }
    AudioObjectPropertyAddress propAddr = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &propAddr, 0, NULL, &dataSize) != noErr || dataSize == 0) {
        return kAudioObjectUnknown;
    }
    UInt32 deviceCount = dataSize / sizeof(AudioDeviceID);
    AudioDeviceID *devices = (AudioDeviceID *)calloc(deviceCount, sizeof(AudioDeviceID));
    if (!devices) {
        return kAudioObjectUnknown;
    }
    AudioDeviceID foundID = kAudioObjectUnknown;
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &propAddr, 0, NULL, &dataSize, devices) == noErr) {
        for (UInt32 i = 0; i < deviceCount; i++) {
            CFStringRef nameRef = NULL;
            UInt32 nameSize = sizeof(CFStringRef);
            AudioObjectPropertyAddress nameAddr = {
                kAudioObjectPropertyName,
                kAudioObjectPropertyScopeGlobal,
                0
            };
            if (AudioObjectGetPropertyData(devices[i], &nameAddr, 0, NULL, &nameSize, &nameRef) == noErr && nameRef) {
                NSString *name = (__bridge NSString *)nameRef;
                if ([name rangeOfString:@"BlackHole 2ch" options:NSCaseInsensitiveSearch].location != NSNotFound) {
                    foundID = devices[i];
                }
                CFRelease(nameRef);
                if (foundID != kAudioObjectUnknown) {
                    break;
                }
            }
        }
    }
    free(devices);
    return foundID;
}

static AudioDeviceID FindEsp32UsbAudioDeviceID(void) {
    AudioObjectPropertyAddress propAddr = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &propAddr, 0, NULL, &dataSize) != noErr || dataSize == 0) {
        return kAudioObjectUnknown;
    }
    UInt32 deviceCount = dataSize / sizeof(AudioDeviceID);
    AudioDeviceID *devices = (AudioDeviceID *)calloc(deviceCount, sizeof(AudioDeviceID));
    if (!devices) {
        return kAudioObjectUnknown;
    }
    AudioDeviceID foundID = kAudioObjectUnknown;
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &propAddr, 0, NULL, &dataSize, devices) == noErr) {
        for (UInt32 i = 0; i < deviceCount; i++) {
            if (GetAudioDeviceChannelCount(devices[i], kAudioObjectPropertyScopeInput) == 0) {
                continue;
            }
            CFStringRef uidRef = NULL;
            UInt32 uidSize = sizeof(CFStringRef);
            AudioObjectPropertyAddress uidAddr = {
                kAudioDevicePropertyDeviceUID,
                kAudioObjectPropertyScopeGlobal,
                0
            };
            BOOL matched = NO;
            if (AudioObjectGetPropertyData(devices[i], &uidAddr, 0, NULL, &uidSize, &uidRef) == noErr && uidRef) {
                NSString *uid = (__bridge NSString *)uidRef;
                if ([uid rangeOfString:@"WS175CMIC01" options:NSCaseInsensitiveSearch].location != NSNotFound ||
                    [uid rangeOfString:@"303A" options:NSCaseInsensitiveSearch].location != NSNotFound) {
                    matched = YES;
                }
                CFRelease(uidRef);
            }
            if (!matched) {
                CFStringRef nameRef = NULL;
                UInt32 nameSize = sizeof(CFStringRef);
                AudioObjectPropertyAddress nameAddr = {
                    kAudioObjectPropertyName,
                    kAudioObjectPropertyScopeGlobal,
                    0
                };
                if (AudioObjectGetPropertyData(devices[i], &nameAddr, 0, NULL, &nameSize, &nameRef) == noErr && nameRef) {
                    NSString *name = (__bridge NSString *)nameRef;
                    if ([name rangeOfString:@"ESP32-S3 Dual Microphone" options:NSCaseInsensitiveSearch].location != NSNotFound ||
                        [name rangeOfString:@"ESP32" options:NSCaseInsensitiveSearch].location != NSNotFound) {
                        matched = YES;
                    }
                    CFRelease(nameRef);
                }
            }
            if (!matched) {
                CFStringRef mfgRef = NULL;
                UInt32 mfgSize = sizeof(CFStringRef);
                AudioObjectPropertyAddress mfgAddr = {
                    kAudioObjectPropertyManufacturer,
                    kAudioObjectPropertyScopeGlobal,
                    0
                };
                if (AudioObjectGetPropertyData(devices[i], &mfgAddr, 0, NULL, &mfgSize, &mfgRef) == noErr && mfgRef) {
                    NSString *mfg = (__bridge NSString *)mfgRef;
                    if ([mfg rangeOfString:@"Waveshare" options:NSCaseInsensitiveSearch].location != NSNotFound ||
                        [mfg rangeOfString:@"Espressif" options:NSCaseInsensitiveSearch].location != NSNotFound) {
                        matched = YES;
                    }
                    CFRelease(mfgRef);
                }
            }
            if (matched) {
                foundID = devices[i];
                break;
            }
        }
    }
    free(devices);
    return foundID;
}

static OSStatus BlackHoleRenderCallback(void *inRefCon,
                                        AudioUnitRenderActionFlags *ioActionFlags,
                                        const AudioTimeStamp *inTimeStamp,
                                        UInt32 inBusNumber,
                                        UInt32 inNumberFrames,
                                        AudioBufferList *ioData) {
    (void)inRefCon;
    (void)ioActionFlags;
    (void)inTimeStamp;
    (void)inBusNumber;

    if (!ioData || ioData->mNumberBuffers == 0) {
        return noErr;
    }

    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    CFAbsoluteTime lastAudio = atomic_load_explicit(&g_lastAudioPktTime, memory_order_relaxed);
    uint32_t w = atomic_load_explicit(&g_pcmWriteIdx, memory_order_acquire);
    uint32_t r = atomic_load_explicit(&g_pcmReadIdx, memory_order_relaxed);
    uint32_t avail = w - r;

    if (avail > PCM_MAX_LATENCY_SAMPLES) {
        r = w - PCM_TARGET_LATENCY_SAMPLES;
        avail = PCM_TARGET_LATENCY_SAMPLES;
        atomic_store_explicit(&g_pcmReadIdx, r, memory_order_release);
    }

    BOOL streamRecent = (lastAudio > 0.0 && (now - lastAudio) < 0.5);
    if (!streamRecent) {
        g_pcmPrebuffering = YES;
    } else if (g_pcmPrebuffering) {
        uint32_t minStart = (inNumberFrames > PCM_PREBUFFER_SAMPLES) ? inNumberFrames : PCM_PREBUFFER_SAMPLES;
        if (avail >= minStart) {
            g_pcmPrebuffering = NO;
        }
    } else if (avail < inNumberFrames) {
        g_pcmPrebuffering = YES;
    }

    if (!streamRecent || g_pcmPrebuffering || avail < inNumberFrames) {
        for (UInt32 b = 0; b < ioData->mNumberBuffers; b++) {
            if (ioData->mBuffers[b].mData && ioData->mBuffers[b].mDataByteSize > 0) {
                memset(ioData->mBuffers[b].mData, 0, ioData->mBuffers[b].mDataByteSize);
            }
        }
        return noErr;
    }

    if (ioData->mNumberBuffers == 1 && ioData->mBuffers[0].mNumberChannels == 2) {
        float *out = (float *)ioData->mBuffers[0].mData;
        for (UInt32 i = 0; i < inNumberFrames; i++) {
            int16_t pcm = g_pcmRingBuf[(r + i) & PCM_RING_MASK];
            float s = (float)pcm / 32768.0f;
            out[i * 2]     = s;
            out[i * 2 + 1] = s;
        }
    } else if (ioData->mNumberBuffers >= 2) {
        float *outL = (float *)ioData->mBuffers[0].mData;
        float *outR = (float *)ioData->mBuffers[1].mData;
        for (UInt32 i = 0; i < inNumberFrames; i++) {
            int16_t pcm = g_pcmRingBuf[(r + i) & PCM_RING_MASK];
            float s = (float)pcm / 32768.0f;
            if (outL) outL[i] = s;
            if (outR) outR[i] = s;
        }
    } else {
        float *out = (float *)ioData->mBuffers[0].mData;
        UInt32 ch = ioData->mBuffers[0].mNumberChannels;
        for (UInt32 i = 0; i < inNumberFrames; i++) {
            int16_t pcm = g_pcmRingBuf[(r + i) & PCM_RING_MASK];
            float s = (float)pcm / 32768.0f;
            for (UInt32 c = 0; c < ch; c++) {
                out[i * ch + c] = s;
            }
        }
    }

    atomic_store_explicit(&g_pcmReadIdx, r + inNumberFrames, memory_order_release);
    return noErr;
}

static BOOL StartBlackHoleAudioBridge(void) {
    if (atomic_load_explicit(&g_blackHoleBridgeActive, memory_order_relaxed) && g_blackHoleAudioUnit != NULL) {
        return YES;
    }
    AudioDeviceID bhID = FindBlackHoleDeviceID();
    if (bhID == kAudioObjectUnknown) {
        return NO;
    }

    /* Ensure BlackHole 2ch nominal sample rate is 48000 Hz */
    Float64 nominalRate = 0.0;
    UInt32 rateSize = sizeof(Float64);
    AudioObjectPropertyAddress rateAddr = {
        kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    if (AudioObjectGetPropertyData(bhID, &rateAddr, 0, NULL, &rateSize, &nominalRate) == noErr && nominalRate != 48000.0) {
        Float64 targetRate = 48000.0;
        AudioObjectSetPropertyData(bhID, &rateAddr, 0, NULL, sizeof(Float64), &targetRate);
    }

    AudioComponentDescription desc = {
        .componentType = kAudioUnitType_Output,
        .componentSubType = kAudioUnitSubType_HALOutput,
        .componentManufacturer = kAudioUnitManufacturer_Apple,
        .componentFlags = 0,
        .componentFlagsMask = 0
    };
    AudioComponent comp = AudioComponentFindNext(NULL, &desc);
    if (!comp) {
        return NO;
    }
    AudioComponentInstance au = NULL;
    if (AudioComponentInstanceNew(comp, &au) != noErr || !au) {
        return NO;
    }

    UInt32 enableOutput = 1;
    if (AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &enableOutput, sizeof(enableOutput)) != noErr) {
        AudioComponentInstanceDispose(au);
        return NO;
    }
    UInt32 disableInput = 0;
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, 1, &disableInput, sizeof(disableInput));

    if (AudioUnitSetProperty(au, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &bhID, sizeof(bhID)) != noErr) {
        AudioComponentInstanceDispose(au);
        return NO;
    }

    AudioStreamBasicDescription streamFmt = {0};
    streamFmt.mSampleRate = 48000.0;
    streamFmt.mFormatID = kAudioFormatLinearPCM;
    streamFmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    streamFmt.mFramesPerPacket = 1;
    streamFmt.mChannelsPerFrame = 2;
    streamFmt.mBitsPerChannel = 32;
    streamFmt.mBytesPerPacket = 8;
    streamFmt.mBytesPerFrame = 8;
    if (AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &streamFmt, sizeof(streamFmt)) != noErr) {
        AudioComponentInstanceDispose(au);
        return NO;
    }

    AURenderCallbackStruct cb = {
        .inputProc = BlackHoleRenderCallback,
        .inputProcRefCon = NULL
    };
    if (AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &cb, sizeof(cb)) != noErr) {
        AudioComponentInstanceDispose(au);
        return NO;
    }

    if (AudioUnitInitialize(au) != noErr) {
        AudioComponentInstanceDispose(au);
        return NO;
    }
    if (AudioOutputUnitStart(au) != noErr) {
        AudioUnitUninitialize(au);
        AudioComponentInstanceDispose(au);
        return NO;
    }

    g_blackHoleAudioUnit = au;
    atomic_store_explicit(&g_blackHoleBridgeActive, true, memory_order_release);
    LogMsg(@"AUDIO BRIDGE: CoreAudio HALOutput 48kHz Stereo Float32 bound to 'BlackHole 2ch' (AudioDeviceID=%u)", (unsigned int)bhID);
    return YES;
}

static void StopBlackHoleAudioBridge(void) {
    atomic_store_explicit(&g_blackHoleBridgeActive, false, memory_order_release);
    if (g_blackHoleAudioUnit != NULL) {
        AudioOutputUnitStop(g_blackHoleAudioUnit);
        AudioUnitUninitialize(g_blackHoleAudioUnit);
        AudioComponentInstanceDispose(g_blackHoleAudioUnit);
        g_blackHoleAudioUnit = NULL;
    }
}

static AudioDeviceID g_savedFallbackInputID = kAudioObjectUnknown;

static UInt32 GetAudioDeviceTransportType(AudioDeviceID devID) {
    if (devID == kAudioObjectUnknown) {
        return 0;
    }
    UInt32 transport = 0;
    UInt32 tSize = sizeof(UInt32);
    AudioObjectPropertyAddress tAddr = {
        kAudioDevicePropertyTransportType,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    if (AudioObjectGetPropertyData(devID, &tAddr, 0, NULL, &tSize, &transport) == noErr) {
        return transport;
    }
    return 0;
}

/* Only allow Built-In (MacBook Pro Microphone) or physical USB microphones as fallback;
 * never select Continuity Camera / iPhone Microphone (Transport=Unknown/0) or Virtual devices. */
static BOOL IsAllowedFallbackInputDevice(AudioDeviceID devID, AudioDeviceID bhID, AudioDeviceID usbAudioID) {
    if (devID == kAudioObjectUnknown || devID == bhID || devID == usbAudioID) {
        return NO;
    }
    if (GetAudioDeviceChannelCount(devID, kAudioObjectPropertyScopeInput) == 0) {
        return NO;
    }
    UInt32 transport = GetAudioDeviceTransportType(devID);
    return (transport == kAudioDeviceTransportTypeBuiltIn ||
            transport == kAudioDeviceTransportTypeUSB);
}

static AudioDeviceID FindFallbackInputDeviceID(AudioDeviceID bhID, AudioDeviceID usbAudioID) {
    if (IsAllowedFallbackInputDevice(g_savedFallbackInputID, bhID, usbAudioID)) {
        return g_savedFallbackInputID;
    }

    AudioObjectPropertyAddress devsAddr = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &devsAddr, 0, NULL, &dataSize) != noErr || dataSize == 0) {
        return kAudioObjectUnknown;
    }
    UInt32 count = dataSize / sizeof(AudioDeviceID);
    AudioDeviceID *devices = (AudioDeviceID *)calloc(count, sizeof(AudioDeviceID));
    if (!devices) {
        return kAudioObjectUnknown;
    }
    AudioDeviceID builtinID = kAudioObjectUnknown;
    AudioDeviceID usbFallbackID = kAudioObjectUnknown;
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &devsAddr, 0, NULL, &dataSize, devices) == noErr) {
        for (UInt32 i = 0; i < count; i++) {
            AudioDeviceID d = devices[i];
            if (!IsAllowedFallbackInputDevice(d, bhID, usbAudioID)) {
                continue;
            }
            UInt32 transport = GetAudioDeviceTransportType(d);
            if (transport == kAudioDeviceTransportTypeBuiltIn) {
                builtinID = d;
                break;
            }
            if (transport == kAudioDeviceTransportTypeUSB && usbFallbackID == kAudioObjectUnknown) {
                usbFallbackID = d;
            }
        }
    }
    free(devices);
    return (builtinID != kAudioObjectUnknown) ? builtinID : usbFallbackID;
}

static void UpdateDefaultAudioInputDevice(BOOL usbMic, BOOL wirelessMic) {
    AudioObjectPropertyAddress defInAddr = {
        kAudioHardwarePropertyDefaultInputDevice,
        kAudioObjectPropertyScopeGlobal,
        0
    };
    AudioDeviceID curInID = kAudioObjectUnknown;
    UInt32 propSize = sizeof(AudioDeviceID);
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &defInAddr, 0, NULL, &propSize, &curInID);

    AudioDeviceID bhID = FindBlackHoleDeviceID();
    AudioDeviceID usbAudioID = FindEsp32UsbAudioDeviceID();

    if (IsAllowedFallbackInputDevice(curInID, bhID, usbAudioID)) {
        g_savedFallbackInputID = curInID;
    }

    if (!usbMic && wirelessMic) {
        if (bhID != kAudioObjectUnknown && curInID != bhID) {
            if (AudioObjectSetPropertyData(kAudioObjectSystemObject, &defInAddr, 0, NULL, sizeof(AudioDeviceID), &bhID) == noErr) {
                LogMsg(@"Switched macOS Default Input Device to 'BlackHole 2ch' (ESP32 Wireless 48kHz Audio)");
            }
        }
    } else if (usbMic) {
        if (usbAudioID != kAudioObjectUnknown && curInID != usbAudioID) {
            if (AudioObjectSetPropertyData(kAudioObjectSystemObject, &defInAddr, 0, NULL, sizeof(AudioDeviceID), &usbAudioID) == noErr) {
                LogMsg(@"Switched macOS Default Input Device to 'ESP32-S3 Dual Microphone' (USB Wired 48kHz Audio)");
            }
        }
    } else {
        if (bhID != kAudioObjectUnknown && curInID == bhID) {
            AudioDeviceID fallbackID = FindFallbackInputDeviceID(bhID, usbAudioID);
            if (fallbackID != kAudioObjectUnknown && fallbackID != bhID) {
                if (AudioObjectSetPropertyData(kAudioObjectSystemObject, &defInAddr, 0, NULL, sizeof(AudioDeviceID), &fallbackID) == noErr) {
                    LogMsg(@"Restored macOS Default Input Device from 'BlackHole 2ch' to fallback microphone (AudioDeviceID=%u)", (unsigned int)fallbackID);
                }
            }
        }
    }
}

/* High-Priority GCD UDP 48kHz Audio & Heartbeat Listener (Port 18796) */
static BOOL StartUdpListener(void) {
    if (g_udpSock >= 0) {
        return YES;
    }
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LogMsg(@"ERROR: Failed to create UDP socket on port %d", ESP32_UDP_PORT);
        return NO;
    }
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));
    int rcvBuf = 256 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvBuf, sizeof(rcvBuf));

    int fl = fcntl(sock, F_GETFL, 0);
    if (fl >= 0) {
        fcntl(sock, F_SETFL, fl | O_NONBLOCK);
    }

    struct sockaddr_in bindAddr;
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.sin_len = sizeof(bindAddr);
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bindAddr.sin_port = htons(ESP32_UDP_PORT);

    if (bind(sock, (struct sockaddr *)&bindAddr, sizeof(bindAddr)) < 0) {
        LogMsg(@"ERROR: Failed to bind UDP socket to 0.0.0.0:%d", ESP32_UDP_PORT);
        close(sock);
        return NO;
    }

    g_udpSock = sock;
    dispatch_queue_attr_t qAttr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, 0);
    g_udpQueue = dispatch_queue_create("com.waveshare.esp32-smart-dummy.udp", qAttr);
    g_udpReadSource = dispatch_source_create(DISPATCH_SOURCE_TYPE_READ, (uintptr_t)g_udpSock, 0, g_udpQueue);
    if (!g_udpReadSource) {
        close(g_udpSock);
        g_udpSock = -1;
        return NO;
    }

    dispatch_source_set_event_handler(g_udpReadSource, ^{
        uint8_t pktBuf[2048];
        struct sockaddr_in senderAddr;
        socklen_t senderLen = sizeof(senderAddr);

        while (1) {
            senderLen = sizeof(senderAddr);
            ssize_t n = recvfrom(g_udpSock, pktBuf, sizeof(pktBuf), 0, (struct sockaddr *)&senderAddr, &senderLen);
            if (n < 0) {
                break;
            }
            if ((size_t)n < sizeof(esp32_udp_hdr_t)) {
                continue;
            }
            const esp32_udp_hdr_t *hdr = (const esp32_udp_hdr_t *)pktBuf;
            if (hdr->magic != ESP32_UDP_MAGIC) {
                continue;
            }

            CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
            CFAbsoluteTime prevRx = atomic_exchange_explicit(&g_lastWirelessRxTime, now, memory_order_relaxed);
            BOOL wasWirelessPresent = (prevRx > 0.0 && (now - prevRx) < 5.0);
            atomic_fetch_add_explicit(&g_wirelessPktCount, 1, memory_order_relaxed);
            atomic_store_explicit(&g_lastWirelessFlags, hdr->flags, memory_order_relaxed);

            char ipStr[INET_ADDRSTRLEN] = {0};
            if (inet_ntop(AF_INET, &senderAddr.sin_addr, ipStr, sizeof(ipStr))) {
                os_unfair_lock_lock(&g_ipLock);
                strncpy(g_lastEsp32Ip, ipStr, sizeof(g_lastEsp32Ip) - 1);
                g_lastEsp32Ip[sizeof(g_lastEsp32Ip) - 1] = '\0';
                os_unfair_lock_unlock(&g_ipLock);
            }

            if (!wasWirelessPresent) {
                WriteDaemonStateFile();
                g_lastStateWriteFromUdp = now;
                ScheduleEvaluation(@"WirelessConnect", 10);
            } else if ((now - g_lastStateWriteFromUdp) >= 0.5) {
                WriteDaemonStateFile();
                g_lastStateWriteFromUdp = now;
            }

            if (hdr->pkt_type == ESP32_PKT_HEARTBEAT || (now - g_lastUdpAckTime) >= 1.0) {
                esp32_udp_hdr_t ack = {
                    .magic = ESP32_UDP_MAGIC,
                    .pkt_type = ESP32_PKT_MAC_ACK_CTRL,
                    .flags = 0,
                    .sample_cnt = 0,
                    .seq = hdr->seq
                };
                sendto(g_udpSock, &ack, sizeof(ack), 0, (const struct sockaddr *)&senderAddr, senderLen);
                g_lastUdpAckTime = now;
            }

            if (hdr->pkt_type == ESP32_PKT_AUDIO_PCM16 && hdr->sample_cnt > 0) {
                size_t expectedBytes = sizeof(esp32_udp_hdr_t) + (size_t)hdr->sample_cnt * sizeof(int16_t);
                if ((size_t)n >= expectedBytes) {
                    const int16_t *samples = (const int16_t *)(pktBuf + sizeof(esp32_udp_hdr_t));
                    BOOL isMuted = ((hdr->flags & 0x01) != 0);
                    uint32_t cnt = hdr->sample_cnt;
                    uint32_t w = atomic_load_explicit(&g_pcmWriteIdx, memory_order_relaxed);
                    for (uint32_t i = 0; i < cnt; i++) {
                        g_pcmRingBuf[(w + i) & PCM_RING_MASK] = isMuted ? 0 : samples[i];
                    }
                    w += cnt;
                    atomic_store_explicit(&g_pcmWriteIdx, w, memory_order_release);
                    if (!atomic_load_explicit(&g_blackHoleBridgeActive, memory_order_relaxed)) {
                        uint32_t r = atomic_load_explicit(&g_pcmReadIdx, memory_order_relaxed);
                        if ((uint32_t)(w - r) > PCM_MAX_LATENCY_SAMPLES) {
                            atomic_store_explicit(&g_pcmReadIdx, w - PCM_TARGET_LATENCY_SAMPLES, memory_order_release);
                        }
                    }
                    atomic_store_explicit(&g_lastAudioPktTime, now, memory_order_relaxed);
                }
            }
        }
    });

    dispatch_source_set_cancel_handler(g_udpReadSource, ^{
        if (g_udpSock >= 0) {
            close(g_udpSock);
            g_udpSock = -1;
        }
    });

    dispatch_resume(g_udpReadSource);
    LogMsg(@"UDP LISTENER: Bound to 0.0.0.0:%d (QOS_CLASS_USER_INTERACTIVE)", ESP32_UDP_PORT);
    return YES;
}

static void StopUdpListener(void) {
    if (g_udpReadSource) {
        dispatch_source_cancel(g_udpReadSource);
        g_udpReadSource = nil;
    } else if (g_udpSock >= 0) {
        close(g_udpSock);
        g_udpSock = -1;
    }
}

/* Check if MacBook is connected to AC Power */
static BOOL IsOnACPower(void) {
    CFTypeRef blob = IOPSCopyPowerSourcesInfo();
    if (!blob) {
        return YES;
    }
    CFStringRef type = IOPSGetProvidingPowerSourceType(blob);
    BOOL ac = (type && CFEqual(type, CFSTR(kIOPMACPowerKey)));
    CFRelease(blob);
    return ac;
}

/* Query Apple Silicon DCP kernel driver (IOMobileFramebuffer) for connected physical external monitors */
static BOOL CountKernelExternalDisplays(uint32_t *outKernelExtCount) {
    CFMutableDictionaryRef matching = IOServiceMatching("IOMobileFramebuffer");
    if (!matching) {
        return NO;
    }
    io_iterator_t iter = 0;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iter) != KERN_SUCCESS) {
        return NO;
    }

    uint32_t extPipesFound = 0;
    uint32_t connectedCount = 0;
    io_service_t service;
    while ((service = IOIteratorNext(iter)) != 0) {
        CFTypeRef extRef = IORegistryEntryCreateCFProperty(service, CFSTR("external"), kCFAllocatorDefault, 0);
        BOOL isExternal = (extRef && CFGetTypeID(extRef) == CFBooleanGetTypeID() && CFBooleanGetValue((CFBooleanRef)extRef));
        if (extRef) {
            CFRelease(extRef);
        }
        if (isExternal) {
            extPipesFound++;
            CFTypeRef attrRef = IORegistryEntryCreateCFProperty(service, CFSTR("DisplayAttributes"), kCFAllocatorDefault, 0);
            if (attrRef) {
                if (CFGetTypeID(attrRef) == CFDictionaryGetTypeID() && CFDictionaryGetCount((CFDictionaryRef)attrRef) > 0) {
                    connectedCount++;
                }
                CFRelease(attrRef);
            }
        }
        IOObjectRelease(service);
    }
    IOObjectRelease(iter);

    if (extPipesFound == 0) {
        return NO;
    }
    if (outKernelExtCount) {
        *outKernelExtCount = connectedCount;
    }
    return YES;
}

/* Count online physical external displays (excluding built-in LCD, macOS fallback virtual display, and our Dummy Display) */
static BOOL CountRealExternalDisplays(uint32_t *outRealExtCount,
                                      uint32_t *outBuiltinCount,
                                      uint32_t *outDummyCount) {
    CGDirectDisplayID displays[16];
    uint32_t count = 0;
    if (CGGetOnlineDisplayList(16, displays, &count) != kCGErrorSuccess) {
        return NO;
    }

    uint32_t cgRealExt = 0;
    uint32_t builtin = 0;
    uint32_t dummy = 0;
    unsigned int myVirtualID = g_virtualDisplay ? g_virtualDisplay.displayID : 0;

    for (uint32_t i = 0; i < count; i++) {
        CGDirectDisplayID d = displays[i];
        if (CGDisplayIsBuiltin(d)) {
            builtin++;
            continue;
        }
        uint32_t vid = CGDisplayVendorNumber(d);
        uint32_t pid = CGDisplayModelNumber(d);
        uint32_t ser = CGDisplaySerialNumber(d);
        if ((myVirtualID != 0 && d == myVirtualID) ||
            (vid == DUMMY_DISPLAY_VID && pid == DUMMY_DISPLAY_PID && ser == DUMMY_DISPLAY_SERIAL)) {
            dummy++;
            continue;
        }
        /* Ignore macOS WindowServer internal fallback virtual display ('unkn'/'virt') when headless */
        if (vid == FALLBACK_DISPLAY_VID && pid == FALLBACK_DISPLAY_PID) {
            continue;
        }
        cgRealExt++;
    }

    uint32_t kernelExt = 0;
    uint32_t realExt = cgRealExt;
    if (CountKernelExternalDisplays(&kernelExt)) {
        realExt = kernelExt;
    }

    if (outRealExtCount) *outRealExtCount = realExt;
    if (outBuiltinCount) *outBuiltinCount = builtin;
    if (outDummyCount) *outDummyCount = dummy;
    return YES;
}

/* Direct IOPMrootDomain UserClient call to disable/enable Clamshell Sleep (Selector 12) */
static void SetRootDomainClamshellSleepDisabled(BOOL disableSleep) {
    for (int attempt = 0; attempt < 2; attempt++) {
        if (g_rootDomainConn == IO_OBJECT_NULL) {
            io_service_t rootDomain = IOPMFindPowerManagement(kIOMainPortDefault);
            if (rootDomain != IO_OBJECT_NULL) {
                IOServiceOpen(rootDomain, mach_task_self(), 0, &g_rootDomainConn);
                IOObjectRelease(rootDomain);
            }
        }
        if (g_rootDomainConn == IO_OBJECT_NULL) {
            break;
        }
        uint64_t inScalar = disableSleep ? 1ULL : 0ULL;
        uint32_t outCount = 0;
        kern_return_t kr = IOConnectCallScalarMethod(g_rootDomainConn,
                                                     kPMSetClamshellSleepState,
                                                     &inScalar,
                                                     1,
                                                     NULL,
                                                     &outCount);
        if (kr == KERN_SUCCESS) {
            break;
        }
        IOServiceClose(g_rootDomainConn);
        g_rootDomainConn = IO_OBJECT_NULL;
    }
}

static void TickleDisplayWake(BOOL force) {
    static CFAbsoluteTime lastTickleTime = 0;
    CFAbsoluteTime now = CFAbsoluteTimeGetCurrent();
    if (force || g_userTickleAssertion == kIOPMNullAssertionID || (now - lastTickleTime) >= 30.0) {
        IOReturn rc = IOPMAssertionDeclareUserActivity(CFSTR("ESP32 Smart Dummy Clamshell Display Wake"),
                                                       kIOPMUserActiveLocal,
                                                       &g_userTickleAssertion);
        if (rc != kIOReturnSuccess && g_userTickleAssertion != kIOPMNullAssertionID) {
            g_userTickleAssertion = kIOPMNullAssertionID;
            rc = IOPMAssertionDeclareUserActivity(CFSTR("ESP32 Smart Dummy Clamshell Display Wake"),
                                                  kIOPMUserActiveLocal,
                                                  &g_userTickleAssertion);
        }
        if (rc == kIOReturnSuccess) {
            lastTickleTime = now;
        }
    }
}

/* Acquire Clamshell-proof powerd assertions (with SensorsActive=YES so powerd EvaluateClamshell counts them on lid-closed unplug) */
static void AcquireStayAwakeAssertions(BOOL forceTickle) {
    BOOL newlyArmed = !g_clamshellStayAwakeActive;
    if (g_sysSleepAssertion == kIOPMNullAssertionID) {
        NSDictionary *sysProps = @{
            (__bridge NSString *)kIOPMAssertionTypeKey: (__bridge NSString *)kIOPMAssertionTypePreventUserIdleSystemSleep,
            (__bridge NSString *)kIOPMAssertionNameKey: @"ESP32 Smart Dummy Clamshell Keep-Alive",
            (__bridge NSString *)kIOPMAssertionLevelKey: @(kIOPMAssertionLevelOn),
            @"SensorsActive": @YES,
        };
        IOPMAssertionCreateWithProperties((__bridge CFDictionaryRef)sysProps, &g_sysSleepAssertion);
    }
    if (g_dispSleepAssertion == kIOPMNullAssertionID) {
        NSDictionary *dispProps = @{
            (__bridge NSString *)kIOPMAssertionTypeKey: (__bridge NSString *)kIOPMAssertionTypePreventUserIdleDisplaySleep,
            (__bridge NSString *)kIOPMAssertionNameKey: @"ESP32 Smart Dummy Display Keep-Alive",
            (__bridge NSString *)kIOPMAssertionLevelKey: @(kIOPMAssertionLevelOn),
            @"SensorsActive": @YES,
        };
        IOPMAssertionCreateWithProperties((__bridge CFDictionaryRef)dispProps, &g_dispSleepAssertion);
    }
    if (g_userActAssertion == kIOPMNullAssertionID) {
        NSDictionary *userProps = @{
            (__bridge NSString *)kIOPMAssertionTypeKey: @"UserIsActive",
            (__bridge NSString *)kIOPMAssertionNameKey: @"ESP32 Smart Dummy Clamshell UserActive",
            (__bridge NSString *)kIOPMAssertionLevelKey: @(kIOPMAssertionLevelOn),
            @"SensorsActive": @YES,
        };
        IOPMAssertionCreateWithProperties((__bridge CFDictionaryRef)userProps, &g_userActAssertion);
    }
    SetRootDomainClamshellSleepDisabled(YES);
    TickleDisplayWake(forceTickle || newlyArmed);
    g_clamshellStayAwakeActive = YES;
}

static void ReleaseStayAwakeAssertions(BOOL allowClamshellSleepNow) {
    BOOL wasActive = g_clamshellStayAwakeActive;
    g_clamshellStayAwakeActive = NO;
    if (g_sysSleepAssertion != kIOPMNullAssertionID) {
        IOPMAssertionRelease(g_sysSleepAssertion);
        g_sysSleepAssertion = kIOPMNullAssertionID;
    }
    if (g_dispSleepAssertion != kIOPMNullAssertionID) {
        IOPMAssertionRelease(g_dispSleepAssertion);
        g_dispSleepAssertion = kIOPMNullAssertionID;
    }
    if (g_userActAssertion != kIOPMNullAssertionID) {
        IOPMAssertionRelease(g_userActAssertion);
        g_userActAssertion = kIOPMNullAssertionID;
    }
    if (g_userTickleAssertion != kIOPMNullAssertionID) {
        IOPMAssertionRelease(g_userTickleAssertion);
        g_userTickleAssertion = kIOPMNullAssertionID;
    }
    if (wasActive && allowClamshellSleepNow) {
        SetRootDomainClamshellSleepDisabled(NO);
    }
}

/* Break mirroring in both directions if WindowServer ever puts the dummy display in a mirror set */
static void EnsureNotMirrored(CGDirectDisplayID dummyID) {
    if (dummyID == 0 || !CGDisplayIsInMirrorSet(dummyID)) {
        return;
    }
    CGDirectDisplayID displays[16];
    uint32_t count = 0;
    if (CGGetOnlineDisplayList(16, displays, &count) != kCGErrorSuccess) {
        return;
    }
    CGDisplayConfigRef configRef = NULL;
    if (CGBeginDisplayConfiguration(&configRef) != kCGErrorSuccess) {
        return;
    }
    BOOL changed = NO;
    if (CGDisplayMirrorsDisplay(dummyID) != kCGNullDirectDisplay) {
        if (CGConfigureDisplayMirrorOfDisplay(configRef, dummyID, kCGNullDirectDisplay) == kCGErrorSuccess) {
            changed = YES;
        }
    }
    for (uint32_t i = 0; i < count; i++) {
        CGDirectDisplayID d = displays[i];
        if (d != dummyID && CGDisplayMirrorsDisplay(d) == dummyID) {
            if (CGConfigureDisplayMirrorOfDisplay(configRef, d, kCGNullDirectDisplay) == kCGErrorSuccess) {
                changed = YES;
            }
        }
    }
    if (changed) {
        CGCompleteDisplayConfiguration(configRef, kCGConfigurePermanently);
    } else {
        CGCancelDisplayConfiguration(configRef);
    }
}

/* Explicitly select the target HiDPI (2x backing pixels) display mode */
static void EnsureHiDPIMode1600x1000(CGDirectDisplayID dummyID) {
    if (dummyID == 0) {
        return;
    }
    unsigned int targetW = g_targetWidth;
    unsigned int targetH = g_targetHeight;
    unsigned int backingW = targetW * 2;
    unsigned int backingH = targetH * 2;
    NSDictionary *opts = @{ (__bridge NSString *)kCGDisplayShowDuplicateLowResolutionModes: @YES };
    CFArrayRef modes = CGDisplayCopyAllDisplayModes(dummyID, (__bridge CFDictionaryRef)opts);
    if (!modes) {
        return;
    }
    CGDisplayModeRef bestMode = NULL;
    CFIndex count = CFArrayGetCount(modes);
    for (CFIndex i = 0; i < count; i++) {
        CGDisplayModeRef m = (CGDisplayModeRef)CFArrayGetValueAtIndex(modes, i);
        size_t w = CGDisplayModeGetWidth(m);
        size_t h = CGDisplayModeGetHeight(m);
        size_t pw = CGDisplayModeGetPixelWidth(m);
        size_t ph = CGDisplayModeGetPixelHeight(m);
        if (w == targetW && h == targetH && pw == backingW && ph == backingH) {
            bestMode = m;
            break;
        }
    }
    if (bestMode) {
        CGDisplayModeRef cur = CGDisplayCopyDisplayMode(dummyID);
        BOOL alreadySet = (cur &&
                           CGDisplayModeGetWidth(cur) == targetW &&
                           CGDisplayModeGetHeight(cur) == targetH &&
                           CGDisplayModeGetPixelWidth(cur) == backingW &&
                           CGDisplayModeGetPixelHeight(cur) == backingH);
        if (cur) {
            CGDisplayModeRelease(cur);
        }
        if (!alreadySet) {
            CGDisplayConfigRef configRef = NULL;
            if (CGBeginDisplayConfiguration(&configRef) == kCGErrorSuccess) {
                if (CGConfigureDisplayWithDisplayMode(configRef, dummyID, bestMode, NULL) == kCGErrorSuccess) {
                    CGCompleteDisplayConfiguration(configRef, kCGConfigurePermanently);
                } else {
                    CGCancelDisplayConfiguration(configRef);
                }
            }
        }
    }
    CFRelease(modes);
}

static void EvaluateState(NSString *trigger);

static void CreateDummyDisplay(void) {
    if (g_virtualDisplay != nil) {
        EnsureNotMirrored(g_virtualDisplay.displayID);
        EnsureHiDPIMode1600x1000(g_virtualDisplay.displayID);
        return;
    }
    unsigned int targetW = g_targetWidth;
    unsigned int targetH = g_targetHeight;
    unsigned int backingW = targetW * 2;
    unsigned int backingH = targetH * 2;

    CGVirtualDisplayDescriptor *desc = [[CGVirtualDisplayDescriptor alloc] init];
    desc.name = [NSString stringWithFormat:@"ESP32 Dummy %ux%u HiDPI", targetW, targetH];
    desc.vendorID = DUMMY_DISPLAY_VID;
    desc.productID = DUMMY_DISPLAY_PID;
    desc.serialNum = DUMMY_DISPLAY_SERIAL;
    desc.maxPixelsWide = backingW;
    desc.maxPixelsHigh = backingH;
    desc.sizeInMillimeters = CGSizeMake(480, 300);
    desc.redPrimary = CGPointMake(0.6400, 0.3300);
    desc.greenPrimary = CGPointMake(0.3000, 0.6000);
    desc.bluePrimary = CGPointMake(0.1500, 0.0600);
    desc.whitePoint = CGPointMake(0.3127, 0.3290);
    static dispatch_queue_t vdisplayQueue = nil;
    if (!vdisplayQueue) {
        vdisplayQueue = dispatch_queue_create("com.waveshare.esp32-smart-dummy.vdisplay", DISPATCH_QUEUE_SERIAL);
    }
    if ([desc respondsToSelector:@selector(setDispatchQueue:)]) {
        [desc setDispatchQueue:vdisplayQueue];
    } else if ([desc respondsToSelector:@selector(setQueue:)]) {
        desc.queue = vdisplayQueue;
    }
    desc.terminationHandler = ^(id a, id b) {
        (void)a;
        (void)b;
        dispatch_async(dispatch_get_main_queue(), ^{
            @autoreleasepool {
                if (g_virtualDisplay != nil) {
                    LogMsg(@"WARNING: CGVirtualDisplay terminationHandler fired — scheduling immediate remount");
                    g_virtualDisplay = nil;
                    g_dummyCreatedAt = 0;
                    EvaluateState(@"VirtualDisplayTerminated");
                }
            }
        });
    };

    CGVirtualDisplay *display = [[CGVirtualDisplay alloc] initWithDescriptor:desc];
    if (!display) {
        LogMsg(@"ERROR: Failed to instantiate CGVirtualDisplay");
        return;
    }

    CGVirtualDisplayMode *modeLogical = [[CGVirtualDisplayMode alloc] initWithWidth:targetW height:targetH refreshRate:60.0];
    CGVirtualDisplayMode *modeBacking = [[CGVirtualDisplayMode alloc] initWithWidth:backingW height:backingH refreshRate:60.0];
    CGVirtualDisplaySettings *settings = [[CGVirtualDisplaySettings alloc] init];
    settings.hiDPI = 1;
    settings.modes = @[modeLogical, modeBacking];

    if (![display applySettings:settings]) {
        LogMsg(@"ERROR: Failed to apply %ux%u HiDPI (%ux%u)@60Hz settings to CGVirtualDisplay",
               targetW, targetH, backingW, backingH);
        return;
    }

    g_virtualDisplay = display;
    g_dummyCreatedAt = CFAbsoluteTimeGetCurrent();
    EnsureNotMirrored(g_virtualDisplay.displayID);
    EnsureHiDPIMode1600x1000(g_virtualDisplay.displayID);
    AcquireStayAwakeAssertions(YES);

    /* Ensure powerd & WindowServer register display activity and HiDPI mode after virtual screen attaches */
    CGVirtualDisplay *createdDisplay = display;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 400 * NSEC_PER_MSEC), dispatch_get_main_queue(), ^{
        @autoreleasepool {
            if (g_virtualDisplay != nil && g_virtualDisplay == createdDisplay) {
                EnsureNotMirrored(g_virtualDisplay.displayID);
                EnsureHiDPIMode1600x1000(g_virtualDisplay.displayID);
                SetRootDomainClamshellSleepDisabled(YES);
                TickleDisplayWake(YES);
            }
        }
    });

    LogMsg(@"ACTIVE: Mounted 'ESP32 Dummy %ux%u HiDPI' (%ux%u backing -> %ux%u@60Hz HiDPI, displayID=%u) + Clamshell Stay-Awake ON",
           targetW, targetH, backingW, backingH, targetW, targetH, g_virtualDisplay.displayID);
}

static void DestroyDummyDisplay(NSString *reason) {
    if (g_virtualDisplay != nil) {
        unsigned int oldID = g_virtualDisplay.displayID;
        g_virtualDisplay = nil;
        g_dummyCreatedAt = 0;
        LogMsg(@"YIELDED: Destroyed 'ESP32 Dummy %ux%u HiDPI' (displayID=%u) — Reason: %@",
               g_targetWidth, g_targetHeight, oldID, reason);
    }
}

static void EvaluateState(NSString *trigger) {
    @autoreleasepool {
        uint32_t realExtCount = 0;
        uint32_t builtinCount = 0;
        uint32_t dummyCount = 0;
        if (!CountRealExternalDisplays(&realExtCount, &builtinCount, &dummyCount)) {
            return;
        }
        if (g_simulateZeroRealMonitors) {
            realExtCount = 0;
        }
        BOOL usbMicPresent = IsEsp32MicPresent();
        BOOL wirelessMicPresent = IsEsp32WirelessPresent();
        BOOL wirelessHoldPresent = IsEsp32WirelessHoldActive(realExtCount == 0);
        BOOL micPresent = (g_standaloneMode || usbMicPresent || wirelessMicPresent || wirelessHoldPresent);
        BOOL onAC = IsOnACPower();

        SendActiveWirelessProbeIfNeeded();

        if (!atomic_load_explicit(&g_blackHoleBridgeActive, memory_order_relaxed)) {
            StartBlackHoleAudioBridge();
        }
        UpdateDefaultAudioInputDevice(usbMicPresent, wirelessMicPresent);
        WriteDaemonStateFile();

        if (g_forceCreateForTest) {
            LogMsg(@"[%@] State check (TEST MODE): realExt=%u, builtin=%u, micPresent=%d (standalone=%d, usb=%d, wifi=%d), onAC=%d",
                   trigger, realExtCount, builtinCount, micPresent, g_standaloneMode, usbMicPresent, wirelessMicPresent, onAC);
            AcquireStayAwakeAssertions(YES);
            CreateDummyDisplay();
            return;
        }

        /* Core Decoupled Policy:
         * A) Clamshell Stay-Awake Assertions (wantKeepAwake = micPresent && onAC):
         *    Must be held CONTINUOUSLY whenever the ESP32 Microphone is connected (USB or Wi-Fi UDP) or standalone mode
         *    is enabled and MacBook is on AC power, even while 1 or 2 real physical monitors are connected!
         *    Why: When the user unplugs the last physical monitor while the lid is closed, WindowServer immediately
         *    sets DesktopMode=0 and powerd runs EvaluateClamshell within ~300ms. Because our SensorsActive=YES
         *    assertions are ALREADY active in powerd, EvaluateClamshell sees assertions >= 1 and keeps
         *    ClamshellSleepDisabled=1 instead of putting the Mac into Clamshell Sleep!
         *
         * B) HiDPI (2x backing) Virtual Dummy Display (wantDummy = wantKeepAwake && realExtCount == 0):
         *    - When realExtCount >= 1: Destroy the virtual display immediately so M3 Pro's 2 external display
         *      pipes are 100% available for real physical monitors regardless of plug order.
         *    - When realExtCount == 0 (and micPresent && onAC): Mount the HiDPI virtual display so
         *      UURemote / Screen Sharing has an active HiDPI framebuffer while in headless clamshell mode.
         */
        BOOL wantKeepAwake = (micPresent && onAC);
        BOOL wantDummy = (wantKeepAwake && realExtCount == 0);
        BOOL stateChanged = ((int)realExtCount != g_lastRealExtCount ||
                             (int)micPresent != g_lastMicPresent ||
                             (int)usbMicPresent != g_lastUsbMicPresent ||
                             (int)wirelessMicPresent != g_lastWifiMicPresent ||
                             (int)onAC != g_lastOnAC ||
                             (int)wantDummy != g_lastDummyActive ||
                             (int)wantKeepAwake != g_lastKeepAwake);

        g_lastRealExtCount = (int)realExtCount;
        g_lastMicPresent = (int)micPresent;
        g_lastUsbMicPresent = (int)usbMicPresent;
        g_lastWifiMicPresent = (int)wirelessMicPresent;
        g_lastOnAC = (int)onAC;
        g_lastDummyActive = (int)wantDummy;
        g_lastKeepAwake = (int)wantKeepAwake;

        if (wantKeepAwake) {
            AcquireStayAwakeAssertions(NO);
            if (realExtCount >= 1) {
                if (g_virtualDisplay != nil) {
                    DestroyDummyDisplay([NSString stringWithFormat:@"%u real external display(s) connected", realExtCount]);
                } else if (stateChanged) {
                    LogMsg(@"[%@] Standby: %u real external display(s) online, active=YES (standalone=%d, USB=%d, WiFi=%d), AC=YES (Dummy Display OFF to yield display pipe; Clamshell Stay-Awake ARMED)",
                           trigger, realExtCount, g_standaloneMode, usbMicPresent, wirelessMicPresent);
                }
            } else {
                if (g_virtualDisplay != nil && dummyCount == 0 && (CFAbsoluteTimeGetCurrent() - g_dummyCreatedAt) > 5.0) {
                    LogMsg(@"[%@] WARNING: Virtual display (displayID=%u) missing from online display list after grace period -> recreating",
                           trigger, g_virtualDisplay.displayID);
                    g_virtualDisplay = nil;
                    g_dummyCreatedAt = 0;
                }
                if (g_virtualDisplay == nil) {
                    LogMsg(@"[%@] 0 real external displays + active trigger (standalone=%d, USB=%d, WiFi=%d) + AC power -> Activating %ux%u HiDPI Dummy Display",
                           trigger, g_standaloneMode, usbMicPresent, wirelessMicPresent, g_targetWidth, g_targetHeight);
                    CreateDummyDisplay();
                } else {
                    EnsureNotMirrored(g_virtualDisplay.displayID);
                    EnsureHiDPIMode1600x1000(g_virtualDisplay.displayID);
                }
            }
        } else {
            NSString *why = !micPresent ? @"ESP32 microphone offline (USB=0, WiFi=0)" : @"MacBook on battery power";
            if (g_virtualDisplay != nil) {
                DestroyDummyDisplay(why);
            }
            /* Re-enable normal Clamshell Sleep if no real monitors are keeping DesktopMode active */
            ReleaseStayAwakeAssertions(realExtCount == 0);
            if (stateChanged) {
                LogMsg(@"[%@] Disarmed: realExt=%u, %@ (Dummy Display OFF, Clamshell Stay-Awake OFF)",
                       trigger, realExtCount, why);
            }
        }
    }
}

static void ScheduleEvaluation(NSString *trigger, uint64_t delayMs) {
    dispatch_async(dispatch_get_main_queue(), ^{
        @autoreleasepool {
            if (g_evalTimer) {
                dispatch_source_cancel(g_evalTimer);
                g_evalTimer = nil;
            }
            g_evalTimer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, dispatch_get_main_queue());
            dispatch_source_set_timer(g_evalTimer,
                                      dispatch_time(DISPATCH_TIME_NOW, delayMs * NSEC_PER_MSEC),
                                      DISPATCH_TIME_FOREVER,
                                      10 * NSEC_PER_MSEC);
            dispatch_source_set_event_handler(g_evalTimer, ^{
                EvaluateState(trigger);
            });
            dispatch_resume(g_evalTimer);
        }
    });
}

static void DisplayReconfigCallback(CGDirectDisplayID display, CGDisplayChangeSummaryFlags flags, void *userInfo) {
    @autoreleasepool {
        (void)display;
        (void)userInfo;
        if (g_clamshellStayAwakeActive) {
            SetRootDomainClamshellSleepDisabled(YES);
        }
        if (flags & kCGDisplayBeginConfigurationFlag) {
            return;
        }
        if (flags & (kCGDisplayAddFlag | kCGDisplayRemoveFlag | kCGDisplayEnabledFlag | kCGDisplayDisabledFlag)) {
            ScheduleEvaluation(@"DisplayHotplug", 20);
        }
    }
}

static void RootDomainInterestCallback(void *refcon, io_service_t service, uint32_t messageType, void *messageArgument) {
    @autoreleasepool {
        (void)refcon;
        (void)service;
        (void)messageArgument;
        if (messageType == kIOPMMessageClamshellStateChange) {
            if (g_clamshellStayAwakeActive) {
                SetRootDomainClamshellSleepDisabled(YES);
                TickleDisplayWake(YES);
            }
            ScheduleEvaluation(@"ClamshellStateChange", 20);
        }
    }
}

static void KernelFramebufferInterestCallback(void *refcon, io_service_t service, uint32_t messageType, void *messageArgument) {
    @autoreleasepool {
        (void)refcon;
        (void)service;
        (void)messageType;
        (void)messageArgument;
        if (g_clamshellStayAwakeActive) {
            SetRootDomainClamshellSleepDisabled(YES);
        }
        ScheduleEvaluation(@"KernelFramebufferEvent", 20);
    }
}

static void SystemPowerCallback(void *refCon, io_service_t service, uint32_t messageType, void *messageArgument) {
    @autoreleasepool {
        (void)refCon;
        (void)service;
        switch (messageType) {
            case kIOMessageCanSystemSleep:
                if (g_clamshellStayAwakeActive) {
                    SetRootDomainClamshellSleepDisabled(YES);
                    TickleDisplayWake(YES);
                    IOCancelPowerChange(g_sysPowerConn, (long)messageArgument);
                } else {
                    IOAllowPowerChange(g_sysPowerConn, (long)messageArgument);
                }
                break;
            case kIOMessageSystemWillSleep:
                IOAllowPowerChange(g_sysPowerConn, (long)messageArgument);
                break;
            case kIOMessageSystemWillPowerOn:
            case kIOMessageSystemHasPoweredOn:
                if (g_clamshellStayAwakeActive) {
                    SetRootDomainClamshellSleepDisabled(YES);
                    TickleDisplayWake(YES);
                }
                ScheduleEvaluation(@"SystemWake", 50);
                break;
            default:
                break;
        }
    }
}

static void DrainUsbIterator(io_iterator_t iterator) {
    io_service_t service;
    while ((service = IOIteratorNext(iterator)) != 0) {
        IOObjectRelease(service);
    }
}

static void UsbDeviceNotificationCallback(void *refCon, io_iterator_t iterator) {
    @autoreleasepool {
        (void)refCon;
        io_service_t service;
        BOOL changed = NO;
        while ((service = IOIteratorNext(iterator)) != 0) {
            IOObjectRelease(service);
            changed = YES;
        }
        if (changed) {
            ScheduleEvaluation(@"USBHotplug", 50);
        }
    }
}

static void PowerSourceCallback(void *context) {
    @autoreleasepool {
        (void)context;
        ScheduleEvaluation(@"PowerSourceChange", 50);
    }
}

static void RegisterSignalHandlers(void) {
    g_signalSources = [NSMutableArray array];
    int signals[] = { SIGINT, SIGTERM, SIGUSR1, SIGUSR2 };
    for (size_t i = 0; i < sizeof(signals) / sizeof(signals[0]); i++) {
        int sig = signals[i];
        signal(sig, SIG_IGN);
        dispatch_source_t src = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL,
                                                       (uintptr_t)sig,
                                                       0,
                                                       dispatch_get_main_queue());
        if (!src) {
            continue;
        }
        dispatch_source_set_event_handler(src, ^{
            @autoreleasepool {
                if (sig == SIGUSR1) {
                    g_simulateZeroRealMonitors = YES;
                    EvaluateState(@"SimulateUnplugMonitors(SIGUSR1)");
                } else if (sig == SIGUSR2) {
                    g_simulateZeroRealMonitors = NO;
                    EvaluateState(@"SimulatePlugMonitors(SIGUSR2)");
                } else {
                    DestroyDummyDisplay(@"Daemon shutting down");
                    if (!IsEsp32WirelessPresent()) {
                        UpdateDefaultAudioInputDevice(NO, NO);
                    }
                    StopBlackHoleAudioBridge();
                    StopUdpListener();
                    uint32_t realExt = 0;
                    CountRealExternalDisplays(&realExt, NULL, NULL);
                    if (realExt > 0 || !IsOnACPower()) {
                        unlink(STATE_FILE_PATH);
                        ReleaseStayAwakeAssertions(realExt == 0);
                    }
                    if (g_rootDomainConn != IO_OBJECT_NULL) {
                        IOServiceClose(g_rootDomainConn);
                        g_rootDomainConn = IO_OBJECT_NULL;
                    }
                    if (g_sysPowerConn != IO_OBJECT_NULL) {
                        IODeregisterForSystemPower(&g_sysPowerNotifier);
                        IOServiceClose(g_sysPowerConn);
                        IONotificationPortDestroy(g_sysPowerNotifyPort);
                        g_sysPowerConn = IO_OBJECT_NULL;
                    }
                    exit(0);
                }
            }
        });
        dispatch_resume(src);
        [g_signalSources addObject:src];
    }
}

static BOOL ParseResolutionArg(const char *arg, unsigned int *outW, unsigned int *outH) {
    if (!arg || !outW || !outH) {
        return NO;
    }
    char *end1 = NULL;
    unsigned long w = strtoul(arg, &end1, 10);
    if (!end1 || end1 == arg || (*end1 != 'x' && *end1 != 'X')) {
        return NO;
    }
    const char *hStart = end1 + 1;
    char *end2 = NULL;
    unsigned long h = strtoul(hStart, &end2, 10);
    if (!end2 || end2 == hStart || *end2 != '\0') {
        return NO;
    }
    if (w < 800 || w > 3840 || h < 600 || h > 2400) {
        return NO;
    }
    *outW = (unsigned int)w;
    *outH = (unsigned int)h;
    return YES;
}

static void PrintUsage(const char *progName) {
    printf("Usage: %s [OPTIONS]\n\n"
           "macOS Headless Clamshell HiDPI Virtual Display, Sleep-Prevention & Wireless Audio Daemon.\n\n"
           "Options:\n"
           "  --status              Print current power, display, audio bridge, and daemon state and exit\n"
           "  --probe-wireless      Print wireless ESP32 UDP presence and IP address and exit\n"
           "  --standalone          Run in standalone Mac-only mode (activate on AC power without ESP32 hardware;\n"
           "                        can also be enabled via environment variable CLAMSHELL_STANDALONE=1)\n"
           "  --resolution <WxH>    Set logical HiDPI resolution in range 800..3840 x 600..2400 (default: 1600x1000,\n"
           "                        backed by a 2x Retina framebuffer, e.g. 3200x2000)\n"
           "  --test-create         Force virtual display creation and stay-awake assertions for testing\n"
           "  -h, --help            Print this usage summary and exit\n",
           progName ? progName : "esp32-smart-dummy");
}

int main(int argc, const char * argv[]) {
    @autoreleasepool {
        const char *envStandalone = getenv("CLAMSHELL_STANDALONE");
        if (envStandalone && (strcmp(envStandalone, "1") == 0 ||
                              strcasecmp(envStandalone, "true") == 0 ||
                              strcasecmp(envStandalone, "yes") == 0)) {
            g_standaloneMode = YES;
        }

        BOOL oneShotStatus = NO;
        BOOL probeWireless = NO;
        for (int i = 1; i < argc; i++) {
            if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
                PrintUsage(argv[0]);
                return 0;
            } else if (strcmp(argv[i], "--standalone") == 0) {
                g_standaloneMode = YES;
            } else if (strcmp(argv[i], "--resolution") == 0) {
                if (i + 1 >= argc) {
                    fprintf(stderr, "Error: --resolution requires a <W>x<H> argument (800..3840 x 600..2400)\n");
                    return 1;
                }
                const char *resArg = argv[++i];
                unsigned int w = 0, h = 0;
                if (!ParseResolutionArg(resArg, &w, &h)) {
                    fprintf(stderr, "Error: Invalid resolution '%s' (expected <W>x<H> in range 800..3840 x 600..2400)\n", resArg);
                    return 1;
                }
                g_targetWidth = w;
                g_targetHeight = h;
            } else if (strncmp(argv[i], "--resolution=", 13) == 0) {
                const char *resArg = argv[i] + 13;
                unsigned int w = 0, h = 0;
                if (!ParseResolutionArg(resArg, &w, &h)) {
                    fprintf(stderr, "Error: Invalid resolution '%s' (expected <W>x<H> in range 800..3840 x 600..2400)\n", resArg);
                    return 1;
                }
                g_targetWidth = w;
                g_targetHeight = h;
            } else if (strcmp(argv[i], "--test-create") == 0) {
                g_forceCreateForTest = YES;
            } else if (strcmp(argv[i], "--status") == 0) {
                oneShotStatus = YES;
            } else if (strcmp(argv[i], "--probe-wireless") == 0) {
                probeWireless = YES;
            }
        }

        if (probeWireless) {
            char ipBuf[64] = "none";
            CFAbsoluteTime lastRx = 0.0;
            BOOL wifi = NO;
            if (ReadDaemonStateFile(&lastRx, NULL, ipBuf, sizeof(ipBuf))) {
                wifi = (lastRx > 0.0 && (CFAbsoluteTimeGetCurrent() - lastRx) < 5.0);
            }
            printf("ESP32_WIFI_PRESENT=%d\nESP32_IP=%s\n", wifi ? 1 : 0, ipBuf);
            return 0;
        }

        if (oneShotStatus) {
            uint32_t realExt = 0, builtin = 0, dummy = 0;
            CountRealExternalDisplays(&realExt, &builtin, &dummy);
            BOOL usbMic = IsEsp32MicPresent();
            CFAbsoluteTime lastRx = 0.0;
            BOOL bhBridge = NO;
            if (ReadDaemonStateFile(&lastRx, &bhBridge, NULL, 0)) {
                /* State file from running daemon is valid */
            } else {
                bhBridge = (FindBlackHoleDeviceID() != kAudioObjectUnknown);
            }
            BOOL wifiMic = (lastRx > 0.0 && (CFAbsoluteTimeGetCurrent() - lastRx) < 5.0);
            BOOL mic = (g_standaloneMode || usbMic || wifiMic);
            BOOL ac = IsOnACPower();
            printf("ESP32_MIC_PRESENT=%d\n"
                   "ESP32_USB_PRESENT=%d\n"
                   "ESP32_WIFI_PRESENT=%d\n"
                   "BLACKHOLE_AUDIO_BRIDGE=%d\n"
                   "AC_POWER=%d\n"
                   "REAL_EXTERNAL_DISPLAYS=%u\n"
                   "DUMMY_DISPLAYS=%u\n"
                   "BUILTIN_DISPLAYS=%u\n"
                   "STANDALONE_MODE=%d\n"
                   "TARGET_RESOLUTION=%ux%u\n",
                   mic ? 1 : 0,
                   usbMic ? 1 : 0,
                   wifiMic ? 1 : 0,
                   bhBridge ? 1 : 0,
                   ac ? 1 : 0,
                   realExt,
                   dummy,
                   builtin,
                   g_standaloneMode ? 1 : 0,
                   g_targetWidth,
                   g_targetHeight);
            return 0;
        }

        g_isDaemonMode = YES;

        /* Seed wireless state from previous daemon instance if updated within the last 60s
         * so a daemon restart while headless in pure Wireless mode has 0ms gap. */
        {
            CFAbsoluteTime prevRx = 0.0;
            char prevIp[64] = {0};
            if (ReadDaemonStateFile(&prevRx, NULL, prevIp, sizeof(prevIp))) {
                if (prevRx > 0.0 && (CFAbsoluteTimeGetCurrent() - prevRx) < 60.0) {
                    atomic_store_explicit(&g_lastWirelessRxTime, prevRx, memory_order_relaxed);
                    if (prevIp[0] != '\0') {
                        os_unfair_lock_lock(&g_ipLock);
                        strncpy(g_lastEsp32Ip, prevIp, sizeof(g_lastEsp32Ip) - 1);
                        os_unfair_lock_unlock(&g_ipLock);
                    }
                }
            }
        }

        /* Initialize shared NSApplication in Prohibited (background-only, no Dock icon) mode
         * so SkyLight delivers CGDisplayRegisterReconfigurationCallback events immediately on hotplug. */
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyProhibited];

        RegisterSignalHandlers();

        /* 0. Start CoreAudio BlackHole 2ch Loopback Bridge & Wireless UDP Listener (Port 18796) */
        StartBlackHoleAudioBridge();
        StartUdpListener();

        /* 1. Register CoreGraphics display hotplug callback */
        CGDisplayRegisterReconfigurationCallback(DisplayReconfigCallback, NULL);

        /* 2. Register IOKit USB attach/detach notifications + IOPMrootDomain clamshell + IOMobileFramebuffer notifications */
        IONotificationPortRef notifyPort = IONotificationPortCreate(kIOMainPortDefault);
        if (notifyPort) {
            CFRunLoopSourceRef rlSource = IONotificationPortGetRunLoopSource(notifyPort);
            CFRunLoopAddSource(CFRunLoopGetMain(), rlSource, kCFRunLoopDefaultMode);

            CFMutableDictionaryRef matchAdd = IOServiceMatching(kIOUSBDeviceClassName);
            if (matchAdd) {
                CFNumberRef vidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &(int){ESP32_USB_VID});
                CFNumberRef pidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &(int){ESP32_USB_PID});
                CFDictionarySetValue(matchAdd, CFSTR(kUSBVendorID), vidNum);
                CFDictionarySetValue(matchAdd, CFSTR(kUSBProductID), pidNum);
                CFRetain(matchAdd); /* One copy for Match, one for Terminate */

                io_iterator_t addIter = 0;
                if (IOServiceAddMatchingNotification(notifyPort, kIOFirstMatchNotification, matchAdd,
                                                     UsbDeviceNotificationCallback, NULL, &addIter) == KERN_SUCCESS) {
                    DrainUsbIterator(addIter);
                }
                io_iterator_t remIter = 0;
                if (IOServiceAddMatchingNotification(notifyPort, kIOTerminatedNotification, matchAdd,
                                                     UsbDeviceNotificationCallback, NULL, &remIter) == KERN_SUCCESS) {
                    DrainUsbIterator(remIter);
                }
                CFRelease(vidNum);
                CFRelease(pidNum);
            }

            io_service_t rootDomain = IOPMFindPowerManagement(kIOMainPortDefault);
            if (rootDomain != IO_OBJECT_NULL) {
                IOServiceAddInterestNotification(notifyPort,
                                                 rootDomain,
                                                 kIOGeneralInterest,
                                                 RootDomainInterestCallback,
                                                 NULL,
                                                 &g_rootDomainNotifier);
                IOObjectRelease(rootDomain);
            }

            io_iterator_t fbIter = 0;
            CFMutableDictionaryRef fbMatch = IOServiceMatching("IOMobileFramebuffer");
            if (fbMatch && IOServiceGetMatchingServices(kIOMainPortDefault, fbMatch, &fbIter) == KERN_SUCCESS) {
                io_service_t fbService;
                while ((fbService = IOIteratorNext(fbIter)) != 0) {
                    CFBooleanRef isExt = IORegistryEntryCreateCFProperty(fbService, CFSTR("external"), kCFAllocatorDefault, 0);
                    if (isExt == kCFBooleanTrue && g_fbNotifierCount < 8) {
                        IOServiceAddInterestNotification(notifyPort,
                                                         fbService,
                                                         kIOGeneralInterest,
                                                         KernelFramebufferInterestCallback,
                                                         NULL,
                                                         &g_fbNotifiers[g_fbNotifierCount++]);
                    }
                    if (isExt) {
                        CFRelease(isExt);
                    }
                    IOObjectRelease(fbService);
                }
                IOObjectRelease(fbIter);
            }
        }

        /* 2b. Register System Sleep/Wake Power Client (IORegisterForSystemPower) */
        g_sysPowerConn = IORegisterForSystemPower(NULL, &g_sysPowerNotifyPort, SystemPowerCallback, &g_sysPowerNotifier);
        if (g_sysPowerConn != IO_OBJECT_NULL && g_sysPowerNotifyPort) {
            CFRunLoopAddSource(CFRunLoopGetMain(),
                               IONotificationPortGetRunLoopSource(g_sysPowerNotifyPort),
                               kCFRunLoopDefaultMode);
        }

        /* 3. Register IOKit Power Source (AC / Battery) change notification */
        CFRunLoopSourceRef psSource = IOPSNotificationCreateRunLoopSource(PowerSourceCallback, NULL);
        if (psSource) {
            CFRunLoopAddSource(CFRunLoopGetMain(), psSource, kCFRunLoopDefaultMode);
        }

        /* 4. Periodic 2s fallback check and Clamshell state reinforcement */
        g_periodicTimer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, dispatch_get_main_queue());
        dispatch_source_set_timer(g_periodicTimer,
                                  dispatch_time(DISPATCH_TIME_NOW, 2 * NSEC_PER_SEC),
                                  2 * NSEC_PER_SEC,
                                  200 * NSEC_PER_MSEC);
        dispatch_source_set_event_handler(g_periodicTimer, ^{
            EvaluateState(@"PeriodicCheck");
        });
        dispatch_resume(g_periodicTimer);

        LogMsg(@"Daemon started (Target: %ux%u@60Hz HiDPI Virtual Dummy Display + Clamshell Stay-Awake, Standalone=%d, Auto-Yield on Real Monitor)",
               g_targetWidth, g_targetHeight, g_standaloneMode);
        EvaluateState(@"Startup");

        [NSApp run];
    }
    return 0;
}
