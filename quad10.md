# First Airframe: Mark4 V2 10 inch Quad

**Decision (2026-11-18):** the first real aircraft is a small quad on the **Mark4 V2 10 inch** carbon frame. It is the "small trainer" of [`drone.md`](drone.md#recommended-order). The larger options in [`octocopter.md`](octocopter.md) stay as later builds that reuse the same board and firmware.

Chosen listing: <https://rcdrone.top/products/mark4-v2-10-inch-fpv-racing-freestyle-drone-frame-kit>

Prices were looked up on 2026-11-18. They are single listings, not quotes, and exclude shipping and import duty. I could not open the chosen listing itself, so the frame figures come from other sellers of the same frame; check them against the page before ordering.

## Why this one

- **Crashes are cheap.** A broken propeller is a few dollars and an arm is part of a $50–70 frame.
- **It is close to what the simulation already models.** The arm length is about 0.21 m against the 0.2 m in [`Stage2-a.md`](Stage2-a.md), and the estimated inertia is nearly the same (see [Rough numbers](#rough-numbers)).
- **It is a plain X quad,** so the four-motor mixer in Stage 2a is used as written.
- **Four motors need four PWM channels,** which is one SAME70 PWM instance (`PWM0`).

## The frame

| | |
|---|---|
| Wheelbase (motor to motor, diagonal) | 427 mm in most listings |
| Arm thickness | 7–7.5 mm carbon |
| Weight | about 230 g |
| Motor mounting holes | 16×16 mm and 19×19 mm |
| Stack mounting holes | 30.5×30.5 mm (M3) and 20×20 mm (M2) |
| Swept circle with 10 inch propellers turning | about 0.68 m |

Listings disagree a little (424, 427 and 473 mm for the wheelbase; 227 and 235 g for the weight). Measure the real frame and use those numbers in the simulation.

## Parts list

### The aircraft

| Item | Qty | Each | Total | Notes |
|---|---|---|---|---|
| Mark4 V2 10 inch frame | 1 | $49–73 | $49–73 | from two other listings of the same frame |
| 3115 900KV motor | 4 | $22–28 | $88–112 | 19×19 mm mount, 5 mm shaft, 105–125 g; the FlashHobby Mars M3115 is $28 |
| 60 A 4-in-1 ESC, 3–6S | 1 | $47–61 | $47–61 | SpeedyBee BLS 60A, 30.5×30.5 mm, BLHeli_S |
| 10×5 three-blade propellers | 2 pairs | $6–18 | $12–36 | Gemfan 1050-3 or HQProp 10x5x3; one CW and one CCW per pair |
| 4S 5000 mAh LiPo | 1 | $29–55 | $29–55 | CNHL; about 450 g. For first flights, see [Battery](#battery-4s-first) |
| ICM-42688-P breakout | 1 | $30–90 | $30–90 | see `octocopter.md` for the listings |
| 5 V UBEC | 1 | $10–15 | $10–15 | rough figure |
| Battery lead, capacitor, wire, standoffs, adapter plate, landing legs | 1 | $20–30 | $20–30 | rough figure; see [Landing legs](#landing-legs) |
| SAME70 Xplained Ultra board | 1 | about $190 | $190 | take this off if it is already on the desk |
| **Aircraft subtotal** | | | **$475–660** | |

A 6S 5000 mAh pack for later is $60–65 (CNHL; about 810 g).

### Needed whatever the aircraft

| Item | Cost | Notes |
|---|---|---|
| Radio transmitter and receiver | $100–250 | rough figure; needs a switch for the motor kill |
| Charger | $50–100 | rough figure from memory; a 5000 mAh pack needs far less than the octocopter's |
| Spare propellers, 4 more pairs | $25–70 | they are the part that breaks |
| LiPo bag, arming plug or smoke stopper | $20–30 | rough figure |
| **Subtotal** | **$195–450** | |

### Total

**About $670–1100** with the Xplained board, or **$480–920** without it.

### The bundle

rcdrone.top also sells this frame as a $299 kit with four 3115 900KV motors, a 60 A 4-in-1 ESC, 1050 propellers and an F722 flight controller. Bought separately, the same parts without the flight controller come to about $200–280. The flight controller is not needed, because the SAME70 replaces it. The bundle is only worth it if the separate parts are out of stock.

### Things to check before ordering

- **Battery connector.** The CNHL 5000 mAh packs found had XT90 or EC5 plugs, not XT60. The ESC's battery lead must match, or be changed.
- **The ESC has no 5 V output** as far as the listings show, which is why the UBEC is on the list. The Xplained board accepts 5–14 V on its power input, and a 4S pack is 16.8 V when full, so it cannot be fed straight from the battery.
- **Motor wires reach the ESC.** On a 10 inch frame the arms are long; some motors come with 25 cm leads and some need extending.

## Driving the ESC from the SAME70

A 4-in-1 ESC is four ESCs on one board. It has a small connector with battery voltage, ground, a current-sense output and four signal inputs, one per motor.

- **Signal.** BLHeli_S firmware accepts an ordinary 1000–2000 µs PWM pulse as well as the faster OneShot125 and digital DShot protocols. The listings only mention DShot, so this is from general knowledge of BLHeli_S; confirm with one motor on the bench.
- **Start with plain PWM** at 400 Hz from `PWM0`. It is the simplest thing to generate and to model in the emulator. OneShot125 (the same idea with 125–250 µs pulses) is an easy later step. DShot is a serial bit stream and would need more firmware and a new emulator model.
- **Throttle calibration.** With plain PWM the ESC has to learn the pulse widths that mean zero and full throttle. Do this once, propellers off.
- **Signal level.** The SAME70's 3.3 V output is what flight controllers use, so this should work directly.
- **Motor direction** is set by which two of a motor's three wires are swapped, or in the ESC's configuration tool. Check each against the Stage 2a table, propellers off.

## Battery: 4S first

On 6S this aircraft is very overpowered for a first self-written controller.

| | 4S | 6S |
|---|---|---|
| Take-off weight | about 1.5 kg | about 1.9 kg |
| Max thrust per motor | about 1.8 kg (my estimate) | about 4 kg (one vendor's claim) |
| Total thrust ÷ weight | about 5 | about 8.5 |
| Hover, as a share of full thrust | about 21% | about 12% |
| Hover command, if thrust goes as command squared | about 0.46 | about 0.34 |

- **Start on 4S.** The motors are rated 3–6S. Less thrust means a mistake in the gains turns into a slower, smaller motion.
- **Add a throttle cap in firmware** as well, as planned for the larger builds.
- **ESC current.** One retailer's own test shows a 3115 900KV pulling 84 A at full throttle on 6S with a 10×5×3 propeller, above the ESC's 60 A rating. On 4S, or with a cap, this does not arise.
- **The 4S thrust figure is an estimate** (thrust scaled by voltage squared from the 6S claim). The seesaw rig will measure the real one.

## Mounting the Xplained board

The frame's stack holes are 30.5 mm apart, made for a flight controller about 36 mm square. The Xplained Ultra is several times that size and has its own hole pattern, so it cannot bolt onto the stack.

This is an inconvenience, not a blocker:

- **Use an adapter plate.** A flat plate of 2–3 mm plywood, G10 or a 3D print bolts to the frame's top-plate standoffs, and the board bolts to it on its own standoffs. Measure the board and the top plate first; the board will probably overhang the sides.
- **The weight is not a problem.** The board and plate add perhaps 100–150 g to an aircraft with several kilograms of spare thrust.
- **Put the gyro, not the board, on the stack position.** The gyro breakout is small. Fix it firmly at the centre of the frame, where the flight controller would go, and run its wires up to the board. Where the board sits then matters little to the control.
- **Use locking or soldered connections.** Loose jumper wires on pin headers will shake off.

The part that does matter is exposure. A board on top of the frame is the first thing to hit the ground when the quad flips, and at $190 it is the most expensive single part on the aircraft.

- **Protect it:** a hoop or a second plate above the board, standing taller than it.
- **Or hang it under the bottom plate.** The frame then shields it in a flip, and the battery goes on top. The quad lands on its belly, so this needs landing legs taller than the board and its connectors, and a thin guard plate under the board against stones and wet grass. Of the two positions this is probably the safer one for the board.
- **Keep it off the aircraft for as long as possible.** On the seesaw rig the board sits on the bench.
- **Later option:** a small custom board, or a compact SAME70 module, that does fit the stack. Not needed to start.

### Landing legs

With the board under the bottom plate, the quad needs legs taller than the board, its standoffs and its connectors. That is probably 30–40 mm; measure once the board is on its plate.

Ready-made legs for this frame are too short:

- **Foam pads.** Mark4-type kits come with small EVA pads that stick under the arms. They are a few millimetres thick.
- **Bolt-on skids.** Small plastic skids that bolt under the motors are sold, for example a set of four for the 16×19 mm motor pattern from WREKD. The listing gives no height and does not confirm a Mark4 fit, and skids of this kind are usually low.

No tall leg sold for this frame turned up, so the legs are made or adapted:

| Option | What it is | Notes |
|---|---|---|
| Long standoffs | four M3 nylon or aluminium standoffs, 40–50 mm, at the corners of the adapter plate, with rubber feet | a few dollars; the height is easy to change; rigid, so a hard landing goes straight into the plate |
| 3D-printed legs | flexible TPU legs that clamp around the arms | absorb landings better; print them or order from a print service |
| Legs from a larger frame | tall plastic legs sold for 450–500 mm camera quads | made for different arms, so they need drilling |

Start with the standoffs. Put the feet as far apart as the plate allows, so the quad does not tip over on landing.

## Rough numbers

Estimates for the simulation's parameter struct, on 4S. Replace them with measurements.

| Parameter | Simulation now | This quad (estimate) |
|---|---|---|
| Mass | not used yet | 1.5 kg |
| Arm length `L` | 0.2 m | 0.21 m |
| `d = L / √2` | 0.141 m | 0.151 m |
| Max thrust per motor (`thrust_coeff`) | 6 N | about 18 N on 4S, about 40 N on 6S |
| `Ixx`, `Iyy` | 0.015 kg·m² | about 0.015 kg·m² |
| `Izz` | 0.025 kg·m² | about 0.03 kg·m² |
| Motor time constant | 30–50 ms | perhaps 40–60 ms |

Weights behind the mass: frame 0.23 kg, four motors 0.46 kg, propellers and ESC 0.1 kg, 4S battery 0.45 kg, board, plate, gyro and wiring 0.25 kg.

The geometry and inertia are already close. The thrust is three to seven times the simulated value, so the torque per unit of command is that much larger, and the Stage 1 gains will be far too strong. Retune in the simulation with these numbers.

## Order of work

The same staged order as in [`octocopter.md`](octocopter.md#tuning-it-without-damaging-it), with smaller hardware:

1. **Host simulation** with the numbers above (Stages 2 and 3 of `drone.md`).
2. **Firmware on the emulator** (Stages 4 and 5).
3. **Bench test, propellers off.** ESC calibration, motor order and direction, tilt response by hand.
4. **Two-motor seesaw rig.** The design in [`octocopter.md`](octocopter.md#the-seesaw-rig), scaled down: a 50 cm bar with the motors 21 cm from the pivot, a pivot 20 cm above the base, and stops 10 cm out and 6 cm below the bar. Measure thrust and motor lag, then tune the rate loop.
5. **Tethered low hover** on 4S with a throttle cap.
6. **Free hover.**

A 10 inch propeller on these motors still cuts badly. The firmware protections and bench rules in `octocopter.md` apply unchanged: arming, kill switch, signal-loss stop, tilt cut-off, throttle cap, loop watchdog, and propellers off whenever the board is on USB.

## Sources

- [Mark4 V2 10 inch frame kit, rcdrone.top (the chosen listing)](https://rcdrone.top/products/mark4-v2-10-inch-fpv-racing-freestyle-drone-frame-kit)
- [Mark4 V2 10 inch frame with F722, 60A ESC and 3115 motors, rcdrone.top](https://rcdrone.top/products/mark4-v2-10inch-427mm-fpv-racing-freestyle-drone-frame-kit-f722-f7-flight-control-60a-4in1-esc-3115-900kv-motor-1050-propeller)
- [RJXHobby Mark4 V2 10 inch frame, rcdrone.top](https://rcdrone.top/products/rjxhobby-mark4-v2-10-inch-fpv-frame-kit)
- [Mark4 V2 10 inch frame, Zbotic](https://zbotic.in/product/mark4-v2-10-inch-carbon-fiber-fpv-frame-424mm-wheelbase/)
- [FlashHobby Mars M3115 900KV, rcdrone.top](https://rcdrone.top/products/flashhobby-mars-m3115-900kv-3-6s-brushless-motor)
- [GTSKYTENRC Tracker 3115 900KV, rcdrone.top](https://rcdrone.top/products/gtskytenrc-tracker-3115-900kv-brushless-motor)
- [Hobbywing XRotor FPV 3115 900KV, ReadyMadeRC](https://www.readymaderc.com/products/details/86677-hobbywing-xrotor-fpv-3115-900kv-motor)
- [10 inch motor, propeller and ESC matching, Unmanned Tech](https://www.unmannedtechshop.co.uk/blogs/knowledge-base/10-inch-fpv-build-motor-prop-esc-matching)
- [SpeedyBee BLS 60A 4-in-1 ESC, GetFPV](https://www.getfpv.com/speedybee-60a-3-6s-blheli-s-4-in-1-esc-30x30.html)
- [SpeedyBee BLS 60A 4-in-1 ESC, NextFPV](https://nextfpv.com/products/speedybee-bls-60a-30x30-4-in-1-esc)
- [Gemfan 1050-3 propeller pair, Pyrodrone](https://pyrodrone.com/collections/all/products/gemfan-1050-3-10-reinforced-carbon-nylon-tri-blade-cinelifter-macro-quad-propellers-1cw-1ccw)
- [HQProp MacroQuad 10x5x3 set of 4, iFlight](https://shop.iflight.com/hqprop-macroquad-prop-10x5x3-cw-ccw-4pcs-2-pairs-pro859?tag=10inch)
- [CNHL 4S 5000 mAh, chinahobbyline.com](https://chinahobbyline.com/products/cnhl-g-plus-5000mah-14-8v-4s-100c-lipo-battery-with-ec5-plug)
- [CNHL Black Series 6S 5000 mAh, chinahobbyline.com](https://chinahobbyline.com/products/cnhl-black-series-5000mah-22-2v-6s-65c-lipo-battery-with-ec5-plug)
- [Ovonic 4S 5000 mAh, rcdrone.top](https://rcdrone.top/products/ovonic-5000mah-lipo-battery-1)
- [WREKD landing skids, 16×19 bolt pattern](https://wrekd.com/products/landing-skids-16-x-19-bolt-pattern-set-of-4)
- [GEPRC Mark4 10 inch frame kit with EVA landing pads, GetFPV](https://www.getfpv.com/commercial-industry-drones/geprc-mark4-10-frame-kit.html)
- [SAM E70 Xplained Ultra, RS](https://uk.rs-online.com/web/p/arduino-compatible-boards-kits/1884854)
