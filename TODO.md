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

## Bike control: mode works on the bike, regen is unverified

**Status:** **mode control works on the bike.** Built, flashed over WiFi and
confirmed 2026-08-10 with the bike on a stand: the app sets the mode and the
machine obeys. Regen is built and shipped alongside it but is **still
unverified** — see the spin-down test below. The evidence lives in
[`tools/talaria.dbc`](tools/talaria.dbc) (`0x490`, `0x402`) and the CAN sections
of both READMEs; read those before touching this.

`0x490` is a **command from the dash to the controller**, not the echo it was
labelled as for months. The puck now answers each of the dash's frames with the
overridden value. Measured on the bike over ~1000 transmits: `tx` climbs by
exactly 5/s (one reply per dash frame, no waste), `txerr=0`, `boff=0`, driver
never leaving `run`. `rxerr` blips to 4–5 and self-clears, which is what two
nodes sharing an arbitration ID look like.

### The bike's display cannot be made to agree — this is permanent

The dash renders its own internal state and ignores the bus entirely. It ignores
the **controller's** frame too, not just ours: through the 30 s injection test
`0x202` reported Eco for the full duration while the screen stayed on Sport. No
CAN message can update that display, from the puck or from anything else.

So while an override is held the bike carries two disagreeing instruments, and
the **app is the honest one** — its mode comes from `0x202`, the controller's own
report. Do not "fix" this; there is nothing to fix it with. The mitigation is
already in the firmware: pressing the handlebar button releases the override and
realigns the display, so the disagreement can never get stuck.

### Regen needs a turning wheel, not a ride — test it before building

An earlier version of this entry said regen could not be confirmed until the
firmware shipped, and made the build order depend on it. That was circular and
wrong twice over. **The rider can already change regen with the handlebar
button**, and the injected-regen test runs from the Mac over the USB-CAN-A,
which is the same rig that proved the mode override. Neither waits on firmware.

It also does not need a ride. It needs a *turning wheel*:

> Bike on a paddock or centre stand, rear wheel free, kickstand up. Throttle to
> ~20 km/h indicated, release, and log `0x303` through the spin-down. Repeat at
> regen 1 and regen 4.

Two runs with the handlebar button answer **"does regen level change the
deceleration at all?"**. Two more with `can_sniff.py` injecting a different
level than the dash is showing answer **"does the controller honour an injected
level?"** — which is the only question that gates the feature.

An unloaded wheel is a poor model of road braking; it has almost no inertia and
the numbers will not resemble real deceleration. That does not matter. The test
asks whether the injected level *changes* the spin-down, not whether it changes
it realistically, so a difference is the entire result.

**If every level spins down identically**, regen is probably triggered by the
brake lever rather than by throttle-off. Retry pulling the *front* lever only,
so the rear mechanical brake does not mask the rear motor's regen.

### The ride that settles the rest

What is genuinely left for a ride, once the bench test above has run:

1. **Odometer** — currently **provisional**. It rests on a single static match:
   the bike's display read 400 km and `0x402[2:4]` read exactly 400. This bus
   carries round constants (1200 sits in both `0x101[4:6]` and `0x302[6:8]`),
   so one match against a round number is suggestive, not proof. 1–2 km of
   riding must move the field.
2. **Regen under load** — only worth doing *after* the bench spin-down has
   shown the controller honours an injected level. The bench answers whether
   the command lands; a road coast-down at regen 1 versus 4 answers whether it
   feels like anything with the bike's own mass behind it. Needs a recorder on
   the moving bike, so it wants the puck flashed first.
3. **Mode override under load** — confirmed on a stand (Eco capped the bike at
   44 km/h, Sport at 80), but a wheel in the air is not the road. Worth one
   check that it holds under real load.

### What is already proven, so nobody re-derives it

- **Reactive injection is the technique.** The controller obeys whichever
  `0x490` it heard last and never latches. Free-running at 20 Hz against a dash
  sending 5 Hz still lost 24% of samples — the current limit oscillating
  between 750 and 1100 several times a second. Replying the *instant* the dash
  speaks held **297/297** of the controller's own frames over 30 s, at a third
  of the traffic.
- **Release is fail-safe and free.** The bike reverts to the dash's choice
  within 100 ms of the last injected frame, so a dropped BLE link, a crashed
  app and a wedged puck all land in the same safe place.
- **The dash never listens.** It renders its own state, so an overridden bike
  disagrees with its own screen for as long as the override lasts. The app
  shows the truth for free — mode reaches the packet from `0x202`, the
  controller's frame — but **regen reaches it from `0x490`**, the dash's frame,
  which the puck cannot hear itself transmit. Under a regen override the app
  must display what it commanded, not what it overheard.
- **The puck stops being read-only.** `TWAI_MODE_LISTEN_ONLY` becomes
  `TWAI_MODE_NORMAL` in both `.ino` files. "It never transmits, so it cannot
  disturb the bike" stops being true, and both READMEs claim it in those words.
