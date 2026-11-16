# Fixed-AP Wi-Fi scan guard for the X1 robot

Validated on 192.168.10.117, Realtek 8852be v1.19.17, 2026-11-16.
This is a machine-specific deployment, not a general Wi-Fi roaming policy.

The driver was continuously in `STA SCAN ASOC`, scanning 3 channels at 100 ms
each before 400 ms on the associated channel. Kernel VR UDP timestamps showed
repeated 300–350 ms gaps followed by queued packets. The DAgger 300 ms watchdog
correctly faulted after input stopped; its threshold must not be increased.

Measured comparison while the robot remained faulted:

| Condition | Packets | Gaps >300 ms | Maximum gap |
| --- | ---: | ---: | ---: |
| Original power save on, 20 s | 1195 | 6 | 343.26 ms |
| Power save off, 40 s capture window | 1554 | 8 | 350.46 ms |
| Power save off, active scan aborted/fenced, 60 s | 3601 | 0 | 114.71 ms |

The power-save-only window contained fewer packets than a continuous 60 Hz
stream; comparison is by observed gaps, not a claimed equal-count loss rate.
No scans were intentionally triggered for this test.

## Installed configuration

`hc-teleop-wifi-realtime` is installed root-owned mode 0755 as
`/etc/NetworkManager/dispatcher.d/90-hc-teleop-wifi-realtime`, with relative
symlinks from both `pre-up.d/` and `pre-down.d/`. The current WLAN connection
UUID is `5aef0153-ba17-48c3-830a-f8d49afd10f9`, with powersave explicitly 2.
The existing global 99-disable setting was overridden by the later-sorted
default-wifi-powersave-on.conf; the explicit connection property avoids this.

For the selected connected fixed-AP profile only, the hook disables power save,
sets the driver's boolean `scan_deny=1`, then requests a bounded 1000 ms scan
abort. It changes no address, route, domain, AP/channel or robot safety setting.
Pre-up, pre-down, down and another profile's up release the scan fence. There
is no NetworkManager restart, link cycle or automatic robot resume.

Connected background AP discovery/roaming scans are intentionally suppressed.
Disconnect before selecting another AP, or release scan_deny manually with
robot output closed. Automatic roaming while still associated is not guaranteed;
do not deploy this policy on a mobile/multi-AP roaming setup without redesign.

The proc ABI was checked against the deployed 8852be.ko disassembly:
proc_set_scan_deny parses an integer and stores bool(nonzero);
proc_set_scan_abort parses an integer and invokes rtw_scan_abort(timeout_ms).
Do not assume the same ABI on another driver/version.

## Revert (with robot output closed)

Write 0 to `/proc/net/rtl8852be/wlan0/scan_deny`; remove the installed hook and
the two symlinks. Restore the selected NetworkManager profile's powersave to 0
(its previous default). Apply the new network policy through the normal
connection lifecycle. Do not silently re-enable power saving during teleop.

Software fixes in the same release remove the remaining observation-only ROS
clock wait from the send lock and prevent marker-only finger projections from
creating fictitious arm-rejection records. Actual input, authorization and
feedback guards remain mandatory. Physical DAgger takeover acceptance must
still be performed by the operator after explicit recovery.
