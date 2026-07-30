# Tripper app — puck integration notes

Changes the [Tripper iOS app](https://github.com/kbirand/Tripper) needs in order
to talk to the puck correctly. Written after the Light/Full build split and the
regen-level decode landed.

Byte layouts are authoritative in [`README.md`](../README.md) → BLE protocol;
this document covers only what the **app** has to do differently, and why.

## Packet reference

**Status packet — 14 bytes, notified at 1 Hz**, little-endian:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `ver` (0x01) |
| 1 | u8 | `fix` |
| 2 | u8 | `sats` |
| 3 | u8 | **`battPct`** — see §1 |
| 4–5 | u16 | `hdop_c` |
| 6–9 | u32 | `uptime_s` |
| 10–11 | i16 | `temp_x10` |
| 12 | u8 | `marker` |
| 13 | u8 | **`caps`** — see §3, was `reserved` |

**Telemetry packet — 70 bytes at 5 Hz.** The byte this document cares about is
**offset 50, `canFlags`**.

---

## 1 · `battPct` is a constant, not a measurement

The app renders `battPct == 0xFF` as `USB` in the settings list. The puck **has
no battery** — that was decided when it moved to bike power (no LiPo, no charge
circuit, no fuel gauge) — so both firmwares assign the byte unconditionally:

```c
s.battPct = 0xFF;   // tripper_puck.ino  and  tripper_light.ino
```

No code path writes anything else, on either build, on any power source.

**Why it misleads rather than merely being useless.** In production the puck runs
off the **5 V BEC from the bike's traction pack**. USB-C is only connected for
flashing, and never at the same time as the BEC — so `USB`, the single state the
row can display, is the one state that should never be true while riding.

**Fix, preferred:** delete the row. It cannot carry information.

**Fix, if a power row is wanted:** label it from the sentinel's *meaning*, not
its value.

```swift
// battPct is a sentinel, not a measurement.
// 0xFF = externally powered (BEC in production, USB while flashing).
let powerLabel = status.battPct == 0xFF ? "External" : "\(status.battPct)%"
```

Keep the `!= 0xFF` branch only as future-proofing should a battery ever appear.

---

## 2 · Regen level — decoded, shipping, not yet displayed

Regen rides in **`canFlags`** at telemetry **offset 50**:

| Bits | Field |
|---|---|
| 0 | CAN live |
| 1 | kickstand down |
| 3:2 | ride mode — 1 Eco, 2 Sport |
| **6:4** | **regen level, 1–4** |

```swift
let canFlags = telemetry[50]
let canLive  = canFlags & 0x01 != 0
let regen    = Int((canFlags >> 4) & 0x07)   // 1...4, 0 = unknown
```

Behaviour verified on the bench across two independent captures and 10 button
presses, including a descending sweep:

- Values are **only ever 1–4**.
- The selector **reverses at each end** instead of wrapping — 4→3→2→1→2→3→4 —
  so it never rolls `4→1`, and **`0` never appears on a live bus**.
- Therefore `regen == 0` unambiguously means *not known yet* and is safe to
  render as `—`.

Worth recording into the ride log, not just showing live: regen level changes
how the bike decelerates, which is real context when replaying a ride corner by
corner.

### Gate every CAN field on `canLive`

This applies to the rows that already exist (`Bike battery`, `Ride mode`,
`Kickstand`) as much as to regen. The firmware **zeroes the whole CAN block**
whenever no frame has arrived in the last 2 s, so an ungated read of a sleeping
bike shows 0 % battery and Eco mode rather than showing nothing:

```swift
guard canLive else { /* render "—", or hide the bike section */ }
```

A parked bike and a disconnected transceiver are indistinguishable by design —
`canFlags` bit 0 is the only thing that separates "real zero" from "no data".

---

## 3 · A Light puck has no GPS — read `caps` first

There are now two hardware builds. The **Light build has no GPS at all** and the
phone must remain the position source. But a Light puck currently looks
identical to a Full puck that hasn't got a fix — both report `flags` bits 0–1
clear, `lat/lon/alt = 0`, `sats = 0`, `hdop = 9999`, `gpsTimeMs = 0xFFFFFFFF`.

That sameness is deliberate: it is why no app branch is needed to *parse* either
build. But it means `GPS: no fix` reads as a fault when it is a permanent
property of the hardware — and the current settings copy promises something a
Light puck cannot do:

> *"While it's live, its 5 Hz GPS and bike-mounted motion sensor are recorded
> instead of the phone's"*

If the app ever stops recording phone GPS on the strength of that promise, a
Light build logs **no position at all**.

### Capability bits — status byte 13

| Bit | Meaning | Light | Full |
|---|---|---|---|
| `0x01` | has GPS | — | ✅ |
| `0x02` | has OLED | — | ✅ |
| `0x04` | has buttons | — | ✅ |
| `0x08` | has CAN | ✅ | ✅ |

Light sends `0x08`; Full sends `0x0F`.

```swift
struct PuckCaps {
    let raw: UInt8
    var hasGPS:     Bool { raw & 0x01 != 0 }
    var hasOLED:    Bool { raw & 0x02 != 0 }
    var hasButtons: Bool { raw & 0x04 != 0 }
    var hasCAN:     Bool { raw & 0x08 != 0 }

    // IMPORTANT: 0 means firmware older than this field, NOT "no capabilities".
    // Every firmware predating it was the Full build, so treat 0 as Full.
    static func parse(_ b: UInt8) -> PuckCaps { PuckCaps(raw: b == 0 ? 0x0F : b) }
}
```

### What to change

1. **Position source** — when `!caps.hasGPS`, keep recording phone GPS and never
   hand over. This is the one that risks losing ride data.
2. **GPS row** — show `Not fitted` instead of `no fix`, or hide the row.
3. **Settings copy** — vary it per build, e.g. *"Its bike-mounted motion sensor
   and the bike's own CAN data are recorded; position comes from your phone."*
4. **`Identify puck`** — `caps.hasOLED` is false on Light, so nothing visibly
   happens. Hide the button, or say it has no indicator on this build.

---

## 4 · Marker semantics differ between builds

Control opcode `0x01` is **not** symmetric:

| Build | Meaning of `0x01` |
|---|---|
| Full | An **ack** — the D2 button already counted the marker; this flashes MARK on the OLED |
| Light | **Originates** the marker and increments the counter — there is no button |

If the app suppresses `0x01` believing it to be cosmetic, markers become
impossible on a Light puck. Send it unconditionally; both builds accept all four
opcodes, so the app never has to withhold a control write.
