// Mac-only passive Pro Controller motion transport. All callbacks and routing run on the main run loop, alongside front_mac.m's player slots.
// Workaround for macOS's zero-only Pro GCMotion: the OS driver cannot be fixed here, but its underlying reports contain the IMU samples (#296).
// A separate buffer / decoder belongs to each retained IOHIDDevice. Disconnect or ambiguity clears only the raw source; GameController keeps its buttons and motion.
#pragma once
#import <IOKit/hid/IOHIDManager.h>
#import <GameController/GCSyntheticDeviceKeys.h>
#include "motion_pro.h"
#include "dm_input.h"
#include <stdlib.h>

typedef struct pro_device { IOHIDDeviceRef hid; uint32_t pid; uint8_t report[512]; pro_motion motion; struct pro_device *next; } pro_device;
static IOHIDManagerRef g_pro_manager;
static pro_device *g_pro_devices, *g_pro_bound;
static int g_pro_player = -1, g_pro_active = 1, g_pro_controllers, g_pro_slot = -1;
static int g_pro_seen_devices = -1, g_pro_seen_controllers = -1;
static void (*g_pro_changed)(void);
static uint8_t g_pro_counter;              // output-report packet counter (4 bits)
static uint64_t g_pro_imu_tried;           // when the IMU-on subcommand was last sent
static int g_pro_imu_retries;

// Custom (local patch, not upstream): Joy-Cons speak the same 0x30 report as a Pro Controller, so the passive reader
// works for them too. A paired set is two HID devices (0x2006 left, 0x2007 right) behind ONE GCController, which the
// Pro rule below rejects. Pick a single IMU source per controller instead: the right Joy-Con (0x2007), whose IMU is the
// one a single-IMU game expects, else a left Joy-Con alone, else the one Pro.
static pro_device *pro_pick(void) {
    pro_device *right = NULL, *left = NULL, *pro = NULL;
    int n = 0;
    for (pro_device *d = g_pro_devices; d; d = d->next) {
        n++;
        if (!right && d->pid == 0x2007) right = d;
        else if (!left && d->pid == 0x2006) left = d;
        else if (!pro && d->pid == 0x2009) pro = d;
    }
    if (g_pro_controllers != 1) return NULL;                 // as before: one controller to bind the samples to
    if (right || left) return right ? right : left;          // one Joy-Con pair: its right half drives the IMU
    return n == 1 ? pro : NULL;                              // otherwise the Pro rule, unchanged
}
static void pro_route(void) {
    int n = 0; for (pro_device *d = g_pro_devices; d; d = d->next) n++;
    // Custom: pro_pick() decides first. It is the only rule that understands a Joy-Con pair (two HID devices
    // behind one controller), and pro_motion_player() rejects exactly that case, so calling it first skipped
    // the Joy-Con path entirely. It is still the fallback for the pure-Pro and ambiguous cases.
    pro_device *d = pro_pick();
    int p = d ? g_pro_slot : pro_motion_player(n, g_pro_controllers, g_pro_slot);
    LOG("[hid] route: %d HID device(s), %d controller(s), slot %d, pick %s, player %d", n, g_pro_controllers, g_pro_slot, d ? "yes" : "no", p);
    if (p < 0) d = NULL;
    if (p < 0 && n && g_pro_controllers && (n != g_pro_seen_devices || g_pro_controllers != g_pro_seen_controllers))
        LOG("[hid] Pro Controller raw motion: %d HID devices / %d controllers; cannot pair them uniquely, keeping GameController motion", n, g_pro_controllers);
    g_pro_seen_devices = n; g_pro_seen_controllers = g_pro_controllers;
    if (d == g_pro_bound && p == g_pro_player) return;
    if (g_pro_player >= 0) dm_motion_clear_hid(g_pro_player);
    g_pro_bound = d; g_pro_player = p;
    if (d) { d->motion = (pro_motion){0}; d->motion.right = (d->pid == 0x2007); g_pro_imu_retries = 0; g_pro_imu_tried = 0; LOG("[hid] Pro Controller raw motion: player %d (passive HID 0x30, pid %04x)", p + 1, d->pid); }
}
static void dm_motion_pro_players(int controllers, int slot) { g_pro_controllers = controllers; g_pro_slot = slot; pro_route(); }

// Custom (local patch, not upstream): a Joy-Con's IMU is OFF until told otherwise, and a pad a Switch has used can keep it
// off, so every 0x30 report carries six zero axes (measured: 680 reports, zero non-zero samples). The console sends
// subcommand 0x40 {0x01} to switch it on; nxnative never did. Output report 0x01 = counter, neutral rumble, subcommand,
// argument. Resent every 2 s while the samples stay zero, at most 5 times, as MacNativeGames does (a pad drops
// subcommands sent back to back).
static void pro_imu_on(pro_device *d) {
    if (!d || d->pid == 0x2009) return;                    // a Pro's IMU is already on (it streams real samples)
    if (g_pro_imu_retries >= 5) return;
    uint64_t now = now_ns();
    if (g_pro_imu_tried && now - g_pro_imu_tried < 2000000000ull) return;
    g_pro_imu_tried = now; g_pro_imu_retries++;
    uint8_t out[49] = {0};
    out[0] = 0x01;                                         // output report ID
    out[1] = ++g_pro_counter & 0x0f;                       // packet counter: 4 bits
    out[2] = 0x00; out[3] = 0x01; out[4] = 0x40; out[5] = 0x40;   // neutral rumble, left side
    out[6] = 0x00; out[7] = 0x01; out[8] = 0x40; out[9] = 0x40;   // neutral rumble, right side
    out[10] = 0x40; out[11] = 0x01;                        // subcommand 0x40, IMU on
    IOReturn e = IOHIDDeviceSetReport(d->hid, kIOHIDReportTypeOutput, out[0], out, sizeof out);
    LOG("[hid] Joy-Con raw motion: IMU-on subcommand sent (%d of 5)%s", g_pro_imu_retries,
        e == kIOReturnSuccess ? "" : " — write failed");
}
static void dm_motion_pro_active(int active) {
    g_pro_active = active;
    if (g_pro_bound) { int r = g_pro_bound->motion.right; g_pro_bound->motion = (pro_motion){0}; g_pro_bound->motion.right = r; }
    if (!active && g_pro_player >= 0) dm_motion_clear_hid(g_pro_player);
}
static void pro_report(void *ctx, IOReturn result, void *sender, IOHIDReportType type, uint32_t report_id, uint8_t *r, CFIndex len) {
    (void)sender;
    pro_device *d = ctx;
    if (result != kIOReturnSuccess || type != kIOHIDReportTypeInput || report_id != 0x30 || len < 49 || !g_pro_active || d != g_pro_bound) return;
    pro_motion_sample samples[3];
    int64_t before = d->motion.live;
    int n = pro_motion_decode(&d->motion, r, (size_t)len, now_ns(), samples);
    if (!before && !d->motion.live) { pro_imu_on(d); return; }   // six zero axes: the Joy-Con's IMU is off; ask for it
    if (!before && d->motion.live) g_pro_imu_retries = 0;        // real samples now: stop resending
    for (int i = 0; i < n; i++) dm_motion_set_hid(g_pro_player, samples[i].acc, samples[i].rot, samples[i].ns);
}
static void pro_found(void *ctx, IOReturn result, void *sender, IOHIDDeviceRef hid) {
    (void)ctx; (void)sender;
    uint32_t found_pid = ((NSNumber *)IOHIDDeviceGetProperty(hid, CFSTR(kIOHIDProductIDKey))).unsignedIntValue;
    if (result != kIOReturnSuccess) { LOG("[hid] raw motion: match callback result 0x%x for pid %04x", result, found_pid); return; }
    if (IOHIDDeviceGetProperty(hid, CFSTR(kIOHIDGCSyntheticDeviceKey)) == kCFBooleanTrue) { LOG("[hid] raw motion: pid %04x is synthetic, skipped", found_pid); return; }
    LOG("[hid] raw motion: matched pid %04x", found_pid);
    for (pro_device *d = g_pro_devices; d; d = d->next) if (d->hid == hid) return;
    pro_device *d = calloc(1, sizeof *d); if (!d) { LOG("[RED] Pro Controller raw motion: cannot allocate device state"); return; }
    d->hid = (IOHIDDeviceRef)CFRetain(hid);
    d->pid = ((NSNumber *)IOHIDDeviceGetProperty(hid, CFSTR(kIOHIDProductIDKey))).unsignedIntValue;
    d->next = g_pro_devices; g_pro_devices = d;
    IOHIDDeviceRegisterInputReportCallback(hid, d->report, sizeof d->report, pro_report, d);
    if (g_pro_changed) g_pro_changed();
}
static void pro_removed(void *ctx, IOReturn result, void *sender, IOHIDDeviceRef hid) {
    (void)ctx; (void)result; (void)sender;
    pro_device **link = &g_pro_devices;
    while (*link && (*link)->hid != hid) link = &(*link)->next;
    if (!*link) return;
    pro_device *d = *link; *link = d->next;
    // Unregister while its buffer / context still exist. The main run loop serializes removal with report callbacks.
    IOHIDDeviceRegisterInputReportCallback(hid, d->report, sizeof d->report, NULL, NULL);
    if (g_pro_changed) g_pro_changed(); else pro_route();
    CFRelease(d->hid); free(d);
}
static void dm_motion_pro_start(void (*changed)(void)) {
    const char *e = getenv("DM_MOTION"); if (e && !atoi(e)) return;
    e = getenv("DM_MOTION_HID"); if (e && !atoi(e)) return;
    g_pro_changed = changed;
    g_pro_manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone); if (!g_pro_manager) return;
    IOHIDManagerSetDeviceMatchingMultiple(g_pro_manager, (__bridge CFArrayRef)@[
        @{@kIOHIDVendorIDKey: @0x057E, @kIOHIDProductIDKey: @0x2009},   // Pro Controller (and pads that present as one, e.g. the IINE)
        @{@kIOHIDVendorIDKey: @0x057E, @kIOHIDProductIDKey: @0x2006},   // Joy-Con (L)
        @{@kIOHIDVendorIDKey: @0x057E, @kIOHIDProductIDKey: @0x2007},   // Joy-Con (R)
    ]);
    IOHIDManagerRegisterDeviceMatchingCallback(g_pro_manager, pro_found, NULL);
    IOHIDManagerRegisterDeviceRemovalCallback(g_pro_manager, pro_removed, NULL);
    IOHIDManagerScheduleWithRunLoop(g_pro_manager, CFRunLoopGetMain(), kCFRunLoopCommonModes);
    IOReturn r = IOHIDManagerOpen(g_pro_manager, kIOHIDOptionsTypeNone);   // nonexclusive, no output / feature reports
    LOG("[hid] raw motion manager: open %s (0x%x), matching 057e:2006/2007/2009", r == kIOReturnSuccess ? "ok" : "FAILED", r);
    if (r != kIOReturnSuccess) {
        LOG("[RED] Pro Controller raw motion: cannot open HID (0x%x); keeping GameController motion", r);
        IOHIDManagerUnscheduleFromRunLoop(g_pro_manager, CFRunLoopGetMain(), kCFRunLoopCommonModes);
        CFRelease(g_pro_manager); g_pro_manager = NULL;
    }
}
