# Open issues

## Lean and slope are still wrong, and the cause is not yet known

**Status:** open, investigation logged in
[`docs/lean-investigation.md`](docs/lean-investigation.md). Read that before
touching attitude code.

Five hypotheses have been tested and eliminated — mount/axes/zero, gyro drift,
the frame algebra, the bike's own acceleration, vibration level — and two
explanations were offered and withdrawn. The gyro, the mount and the barometer
are all confirmed healthy; the absolute angle out of the BNO055's fusion is
not. Firmware v0x04 puts the raw accelerometer on the wire so the next ride can
tell "the chip is fed bad data" from "the chip mishandles good data", which
this one could not. The file lists exactly what to check when that ride lands.

## Connector topology for the handlebar run is out of date in the docs

**Status:** decided in discussion, not yet reflected in the documentation.

The docs currently describe the rear-to-handlebar link as **two panel-mount M12
receptacles** — one in each enclosure. That is wrong on its own terms: two panel
receptacles mate face to face, so there is no cable between them. A panel-only
pair would need two additional cable-end plugs to become a working harness, and
no M12 8-pin cable-end plug could be found in stock domestically.

The part actually sourced is a [Motorobit M12 8-pin
set](https://www.motorobit.com/m12-8-pinli-konnektor-seti) (≈ ₺691, IP67, 4–6 mm
cable entry). **Both halves of that set are cable-end**, not panel — so the real
topology is an in-line coupler:

```
[handlebar box] ══cable══► (female) ╪ (male) ◄══pigtail══ [rear box]
        ↑                                              ↑
      gland                                          gland
```

That is **2 glands + 1 in-line connector pair**, not 1 panel + 1 plug.

### What needs changing

| File | What is stale |
|---|---|
| `README.md` | "Connectors" section — the M12 row and the gender-follows-the-live-side paragraph (that rule applies to the M8 bike run only; the handlebar run is dead on both sides once the bike connector is pulled) |
| `hardware/build-guide.html` | "Connectors — two harnesses, deliberately different" section, the shopping-list M12 row, and build step 7 ("Terminate the handlebar run in the M12 8-pin connector") |
| `wiring-schema.jpg` | Callout ⑤ still reads as a panel connector at the enclosure boundary |

### Consequences to carry into the rewrite

- **Cable choice narrows.** The 4–6 mm gland rules out full S/FTP Cat6 (~6.0–6.2 mm).
  Use **F/UTP stranded Cat5e/Cat6 patch, ~5 mm**. Stranded, not solid — solid
  copper work-hardens and breaks under vibration.
- **The shield cannot cross the connector.** The body is plastic and all 8
  conductors are in use, so there is no path for a 360° shield termination.
  Bond the shield to GND **in the handlebar box only** and cut it back at the
  connector. This reverses the "shield at the rear end only" rule currently in
  the docs — what matters is that it is grounded at exactly one end.
- **Mechanical:** an in-line connector left hanging fatigues its own cable
  entries. Secure it to the frame with a P-clip.
- Pair assignment is unaffected: `3V3+GND / SDA+btn1 / SCL+btn2 / GPS TX+RX`.
  SDA and SCL must stay in separate pairs.

### Not affected

The M8 bike connector (ATTEND 219A-04MSR panel male + SIGNAL ASM08AF04001 cable
female) is unchanged, and so is the gender rule for it: the cable side runs back
to the BEC and stays live, so it gets the female sockets.

`hardware/tripper-puck.fzz` is also unaffected — it models each connector as a
mated pass-through, which is true regardless of whether the body is panel-mount
or in-line.
