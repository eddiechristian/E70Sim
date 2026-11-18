# Octocopter Option: Tarot T15

This looks at building on the **Tarot T15 folding octocopter frame (TL15T00)** in place of a quad, with the same plan as [`drone.md`](drone.md): the ICM-42688-P gyro, my own flight code on the SAME70, and off-the-shelf ESCs.

Prices were looked up on 2026-11-18. They are single listings, not quotes, and exclude shipping and import duty. Check each one before buying.

## Will it work?

Yes. Nothing in the plan depends on having four motors.

- **The ESCs take a plain PWM signal.** A hobby ESC only wants a pulse of about 1000–2000 µs per motor. It does not care what produced it, so my PID and mixer can drive it directly. No Tarot or DJI flight controller is needed.
- **The SAME70 has exactly eight PWM channels.** The SVD lists `PWM0` (`0x40020000`) and `PWM1` (`0x4005C000`), with four channels each. Check that all eight come out on pins the board exposes.
- **The gyro is unchanged.** One ICM-42688-P, same driver, same estimator.
- **The control code is unchanged** apart from the mixer, which becomes an eight-row table (see [The mixer](#the-mixer)).

Three things to get right:

- **The frame is a kit of carbon and aluminium only.** TL15T00 has no motors, ESCs or propellers. Those are bought separately.
- **The ESCs are "OPTO", so they have no 5 V output.** The SAME70 board and receiver need their own 5 V regulator (a UBEC) from the battery.
- **Signal level.** The SAME70 puts out 3.3 V PWM. Hobbywing ESCs normally accept that, but confirm it on the bench with one ESC before wiring eight.

One open point: I could not confirm the XRotor Pro's accepted PWM frequency from Hobbywing's page. From memory it is 50–500 Hz. Read the manual in the box; 400 Hz is the usual choice and gives the ESC a fresh command every 2.5 ms.

## The frame

| | |
|---|---|
| Part number | TL15T00 |
| Layout | flat octocopter, eight folding arms |
| Motor-to-motor diameter | 1020 mm (1070 mm overall) |
| Arm tubes | 25 mm carbon |
| Frame weight | 1.95 kg |
| Recommended motor | Tarot 4114 320KV |
| Recommended ESC | 35–40 A (Hobbywing XRotor 40A OPTO) |
| Recommended battery | 6S, 10000–15000 mAh |
| Weight with accessories | 8 kg (one listing; probably the intended flying weight) |

**Availability is the first problem.** The only retailer found with a price, rcdrone.top, lists it at $479 and marks it sold out. No Amazon, eBay or AliExpress listing turned up. If it cannot be found, the Tarot T18 (TL18T00, 1270 mm, 18 inch props) is the same design one size up, and some spare parts are shared with the T15. See [The T18 option](#the-t18-option-lifting-more).

## Parts list

### The aircraft

| Item | Qty | Each | Total | Notes |
|---|---|---|---|---|
| Tarot T15 frame, TL15T00 | 1 | $479 | $479 | sold out where found |
| Tarot 4114 320KV motor, TL100B08 | 8 | $50–60 | $400–480 | £44.90 inc. VAT at Flying Tech, in stock |
| Hobbywing XRotor Pro 40A ESC | 8 | $25–40 | $200–320 | 3–6S, 40 A continuous, no BEC; sold singly at about $40 or as a dual pack at about $50 |
| Tarot 1555 folding carbon propellers | 2 sets of 4 | $59 | $118 | each set is 2 CW + 2 CCW |
| 6S 16000 mAh LiPo | 1 | $170–336 | $170–336 | CNHL 15C $170; Tattu 30C $336 |
| ICM-42688-P breakout | 1 | $30–90 | $30–90 | see below |
| Power distribution board, XT90 connectors, 12 AWG wire | 1 | $40–60 | $40–60 | must carry about 100 A in total |
| 5 V UBEC | 1 | $10–15 | $10–15 | powers the SAME70 board and receiver |
| SAME70 Xplained Ultra board, DM320113 | 1 | about $190 | $190 | $188.60 at Future Electronics; other distributors are dearer and DigiKey quotes a 12 week lead time |
| **Aircraft subtotal** | | | **$1640–2090** | |

If the Xplained board is already on the desk, take $190 off.

**The Xplained board as a flight controller.** It works, and it is the board the emulator and firmware already target, so the same binary runs in both places. Three things to plan for:

- **Which PWM pins reach the headers.** The chip has eight PWM channels, but the board routes only some pins to its Xplained Pro and Arduino headers, and some are shared with on-board parts. Check the pin tables in the user's guide (DS70005389B, section 5) and list eight usable `PWMH` pins before ordering anything. If fewer than eight are free, the timer counters (TC) can generate the missing ones.
- **Vibration and wiring.** It is a large development board with pin headers. Jumper wires shake loose on a flying frame, and a lost gyro or ESC wire is a crash. Use a soldered or locking-connector adapter board, and mount the gyro breakout rigidly near the centre of the frame, not dangling on wires.
- **Power.** Feed it from the UBEC, not from USB, in flight.

**The gyro breakout is oddly expensive at the moment.** The bare chip is about $2, but the boards found were $88 (Elecrow, which says the price has spiked on supply), $53 (TDK's own Pmod board at DigiKey, two weeks' lead time) and $30 (Mikroe MIKROE-4237 at DigiKey; I believe this is the "6DOF IMU 14 Click" with the ICM-42688-P on it, but the listing did not say, so confirm before ordering). The BMI270 remains the fallback.

### Needed whatever the aircraft

| Item | Cost | Notes |
|---|---|---|
| Radio transmitter and receiver | $100–250 | needs a switch that can be assigned as a motor kill |
| Charger and power supply | $100–200 | a 16000 mAh 6S pack needs 300 W or more to charge in about an hour |
| Spare propellers, one set of 4 | $59 | |
| LiPo bag, XT90-S anti-spark arming plug, tether line and anchors | $30–50 | |
| **Subtotal** | **$290–560** | |

### Total

**About $1950–2650** with the Xplained board, or **$1750–2450** without it, before shipping and duty. That is at or over the $2000 budget in `drone.md`, with one battery and one spare set of propellers.

Two spare motors and two spare ESCs ($150–200) are worth adding. A hard landing on a folding-arm frame usually costs an arm, a motor and a propeller.

## Rough numbers

All estimates until the parts are weighed.

| | Estimate |
|---|---|
| Frame | 1.95 kg |
| 8 motors (about 150 g each) | 1.2 kg |
| 8 ESCs (45–50 g each) | 0.4 kg |
| 8 propellers and holders | 0.3 kg |
| Battery, 6S 16000 mAh | 1.9–2.0 kg |
| Wiring, power board, electronics | 0.5 kg |
| **Take-off weight, no payload** | **about 6.3 kg** |

- **Thrust:** a 4114 320KV with a 15 inch propeller on 6S gives very roughly 2.5 kg at full throttle. That figure is from memory; use the manufacturer's thrust table. Eight give about 20 kg, a little over three times the weight.
- **Hover:** about 0.8 kg per motor, around 40% throttle.
- **Payload:** 2–3 kg keeps the thrust at twice the weight or better. That matches the "mid-size lifter" row in `drone.md`.
- **Arm length `L`:** 0.51 m, against 0.2 m in the simulation's current quad.
- **Inertia:** roughly `Ixx = Iyy ≈ 0.25` kg·m² and `Izz ≈ 0.5` kg·m², from the motor masses at the arm ends. The simulated quad uses 0.015 and 0.025, so this is about 15–20 times harder to turn.
- **Motor time constant:** perhaps 60–100 ms with 15 inch propellers, against the 30–50 ms assumed so far.

## The T18 option: lifting more

The T15 cannot be upgraded to lift much more, because it is already at its largest propeller, and propeller size is what sets the lift.

- **Propeller clearance.** Eight motors on a 1020 mm circle sit about 390 mm apart. A 15 inch propeller is 381 mm across, so neighbours clear by about 9 mm. A 16 inch propeller (406 mm) would collide.
- **Bigger motors alone gain little.** On the same propeller a stronger motor only helps by spinning it faster, which costs a lot of battery for a small gain in thrust.
- **The frame is rated for about 8 kg.**

The **Tarot T18 (TL18T00)** is the same folding design one size up, and is the way to more lift.

### T15 and T18 compared

| | T15 | T18 |
|---|---|---|
| Motor-to-motor diameter | 1020 mm | 1270 mm (1320 mm overall) |
| Motor spacing | 390 mm | 486 mm |
| Propellers | 15 inch | 18 inch (457 mm, clears by 29 mm) |
| Arm tubes | 25 mm carbon | 25 mm carbon |
| Frame weight | 1.95 kg | 2.0 kg (one listing) |
| Motor | Tarot 4114 320KV | Tarot 5008 340KV |
| Max thrust per motor | about 2.5 kg (from memory) | 3.7 kg with an 1855 propeller (retailer listing) |
| Total thrust | about 20 kg | about 30 kg |
| Listed weight with accessories | 8 kg | 11 kg |
| Take-off weight, no payload | about 6.3 kg | about 6.9 kg |
| Rough payload | 2–3 kg | about 4 kg |

The payload figures keep the aircraft inside the listed weight. The T18's thrust would carry more than 4 kg, but the 11 kg figure is the only rating found for the frame, so treat it as the limit until Tarot's own documents say otherwise.

### T18 parts and prices

| Item | Qty | Each | Total | Notes |
|---|---|---|---|---|
| Tarot T18 frame, TL18T00 | 1 | $509–712 | $509–712 | $509 at rcdrone.top, sold out; $712 at QWinOut, stock not confirmed; GetFPV lists it as discontinued |
| Tarot 5008 340KV motor, TL96020 | 8 | $55–65 | $440–520 | £49.90 inc. VAT at Flying Tech, in stock; 171 g each |
| Hobbywing XRotor Pro 40A ESC | 8 | $25–40 | $200–320 | the motor's maximum current is 40 A, so there is no headroom; see below |
| 18 inch folding propellers | 4 pairs | $30–70 | $120–280 | no priced listing found for the Tarot 1855 itself; the range is from Tarot 1865 and 1765 pairs |
| 6S 16000 mAh LiPo | 1 | $170–336 | $170–336 | as for the T15 |
| ICM-42688-P breakout | 1 | $30–90 | $30–90 | as for the T15 |
| Power distribution board, XT90 connectors, 10 AWG wire | 1 | $50–80 | $50–80 | heavier than the T15's |
| 5 V UBEC | 1 | $10–15 | $10–15 | |
| SAME70 Xplained Ultra board | 1 | about $190 | $190 | |
| **Aircraft subtotal** | | | **$1720–2540** | |

With the radio, charger, a spare pair or two of propellers and the safety gear ($290–640), the total is **about $2000–3200**, against $1950–2650 for the T15. Like the T15 frame, the T18 frame was not found in stock at a confirmed price.

**ESC headroom.** The 5008 draws up to 40 A at full throttle, which is the XRotor Pro 40A's continuous rating. In a hover each motor draws far less, but a 50 A or 60 A ESC is the safer match. Those were not priced here.

### T18 rough numbers

| | Estimate |
|---|---|
| Frame | 2.0 kg |
| 8 motors (171 g each) | 1.4 kg |
| 8 ESCs | 0.4 kg |
| 8 propellers and holders | 0.4 kg |
| Battery, 6S 16000 mAh | 2.0 kg |
| Wiring, power board, electronics | 0.7 kg |
| **Take-off weight, no payload** | **about 6.9 kg** |

- **Arm length `L`:** 0.635 m.
- **Inertia:** roughly `Ixx = Iyy ≈ 0.5` kg·m² and `Izz ≈ 1.0` kg·m², about twice the T15's.
- **Motor time constant:** a little slower than the T15's; perhaps 80–120 ms.
- **A second battery** in parallel adds 2 kg and takes 2 kg off the payload.

### Choosing between them

Decide the payload before buying motors, because the motors and propellers are the parts that do not transfer.

- **Carries over from a T15 to a T18:** the SAME70 board, gyro, firmware, radio, charger, 6S batteries and the seesaw rig.
- **Does not carry over:** the frame, the eight motors and the propellers, about $1000 of the build.
- **The seesaw rig's measurements belong to one motor and propeller.** Build the rig with the pair that will fly. With 18 inch propellers, move the stops in to 20 cm from the centre, 11 cm below the bar.

Neither frame reaches the 20 lb (9 kg) payload in `drone.md`. That needs 12S motors, propellers around 30 inches and a different frame: see [The quad option](#the-quad-option-30-inch-propellers).

## The quad option: 30 inch propellers

A quad can lift more than either octocopter, because four motors sit further apart than eight and leave room for much larger propellers. This is the "bare-bones 20 lb lifter" of `drone.md`, worked out with real parts.

- **Lift comes from propeller area.** Four 30 inch propellers have about twice the area of the T15's eight 15 inch ones.
- **Room for them.** A 30 inch propeller is 762 mm across. On a quad the motors are 0.707 × the frame diameter apart, so the frame needs a motor-to-motor diameter of at least 1080 mm; 1200 mm leaves about 87 mm between blade tips.
- **With the same motors a quad lifts half as much.** The gain comes only from the bigger propellers and the bigger motors that turn them.

### Propulsion: Hobbywing XRotor X8

At this size the motor, ESC and propeller are sold as one unit that clamps onto the end of an arm tube. The Hobbywing X8 is the common one for 30 inch propellers:

| | |
|---|---|
| Propeller | 3090 folding (30 inch) |
| Battery | 12S |
| Max thrust per unit | 15.3 kg |
| Recommended load per unit | 5–7 kg |
| Weight per unit | 1040 g, complete |
| Arm tube | 35 mm, or 30 mm with the adapter in the box; a "D40" version fits 40/35 mm |
| Price | $186–230 each; the two cheaper listings were sold out, Hobbywing's own store is $229.99 |

Four units give about 61 kg of thrust. The recommended load puts the take-off weight at 20–28 kg.

It takes a throttle signal like any ESC, so my own PID and mixer still drive it. I could not confirm the accepted PWM frequency for this exact model; read its manual before choosing a rate.

### Frames found

Two carbon frames turned up that are built for 30 inch propellers. Neither was in stock where a price was shown.

| | EFT E410P | RJX1200D |
|---|---|---|
| Type | four-arm folding agricultural frame | four-arm "umbrella" folding frame |
| Built for | Hobbywing X8 and 30 inch propellers | 30 inch propellers |
| Motor-to-motor diameter | not found; check the EFT manual | 1200 mm |
| Folded | 635 × 666 × 490 mm | 600 × 600 × 590 mm |
| Frame weight | 6 kg (one listing; may include the 10 L tank and landing gear) | about 3.0 kg |
| Price | $790 at rcdrone.top, sold out; about $490–600 at Indian retailers | $2159, sold out; sold without motor mounts |
| Arm tube diameter | not found | not found |

- **The E410P is the budget choice.** It is a crop-spraying frame, so it comes with a tank mount and landing gear that this project does not need, and it is heavy. It is designed around the X8, which removes the question of whether the motor fits the arm.
- **The RJX1200D is half the weight and nearly three times the price.**
- **Check the arm tube diameter against the X8** (30, 35 or 40 mm) before buying either.

### Parts and prices, with the E410P

| Item | Qty | Each | Total | Notes |
|---|---|---|---|---|
| EFT E410P frame | 1 | $790 | $790 | sold out where priced in USD |
| Hobbywing X8 unit with 3090 propeller | 4 (2 CW, 2 CCW) | $186–230 | $745–920 | |
| 12S 16000 mAh LiPo | 1 | $475–860 | $475–860 | Tattu and Herewin listings; several sold out |
| ICM-42688-P breakout | 1 | $30–90 | $30–90 | |
| Power distribution, AS150 connectors, 8 AWG wire | 1 | $50–80 | $50–80 | rough figure |
| 5 V regulator rated for 12S input | 1 | $15–30 | $15–30 | rough figure; an ordinary 6S UBEC will not survive 50 V |
| SAME70 Xplained Ultra board | 1 | about $190 | $190 | |
| **Aircraft subtotal** | | | **$2300–2960** | |

Also needed: radio ($100–250), a 12S charger ($150–400, rough figure from memory), a spare pair of propellers (about $70) and safety gear ($30–50).

**Total: about $2650–3750** with the E410P, or about $4000–5100 with the RJX1200D.

### Rough numbers

| | E410P | RJX1200D |
|---|---|---|
| Frame | 6.0 kg | 3.0 kg |
| 4 X8 units | 4.2 kg | 4.2 kg |
| Battery, 12S 16000 mAh (from memory) | 4.7 kg | 4.7 kg |
| Wiring and electronics | 0.7 kg | 0.7 kg |
| **Take-off weight, no payload** | **about 15.6 kg** | **about 12.6 kg** |
| With a 9 kg (20 lb) payload | about 24.6 kg | about 21.6 kg |

- **A 20 lb payload fits** inside the X8's recommended load on either frame.
- **The E410P version sits at the FAA's 55 lb (25 kg) limit** for small unmanned aircraft with that payload. Check the current rules.
- **Swept circle:** about 1.96 m (6.4 ft) on a 1200 mm frame.
- **Inertia:** very roughly `Ixx = Iyy ≈ 0.9` kg·m² and `Izz ≈ 1.8` kg·m², because a kilogram of motor sits at the end of each arm.
- **Motor time constant:** 30 inch propellers are slow to change speed; perhaps 100–200 ms.

### The three options side by side

| | T15 octocopter | T18 octocopter | 30 inch quad (E410P) |
|---|---|---|---|
| Propellers | 8 × 15 inch | 8 × 18 inch | 4 × 30 inch |
| Battery | 6S | 6S | 12S |
| Take-off weight, no payload | 6.3 kg | 6.9 kg | 15.6 kg |
| Rough payload | 2–3 kg | about 4 kg | 9 kg or more |
| Swept circle | 1.40 m | 1.73 m | about 1.96 m |
| Survives a motor failure | possibly | possibly | no |
| Total cost | $1950–2650 | $2000–3200 | $2650–3750 |
| Mixer | eight rows | eight rows | the four-motor X mixer in Stage 2a |

What the quad costs besides money:

- **No redundancy.** One motor, ESC or propeller failure is a crash of a 15–25 kg machine.
- **12S everything.** The battery, charger and regulator are a different class from the 6S parts, and none transfer from an octocopter build.
- **Far more dangerous.** A 30 inch carbon propeller driven by 15 kg of thrust is not a bench-top object. The [seesaw rig](#the-seesaw-rig) as drawn is too light for it: it would need a steel or heavy timber frame bolted to the floor, a hard throttle cap, and a barrier.
- **Slow motors** limit how sharp the control can be.

A sensible order, if the quad is the goal: fly a small or mid-size 6S aircraft with this firmware first, and build the 12S quad afterwards with the same board and code. That matches the recommended order in `drone.md`.

## Can it be simulated?

Yes, and it should be before anything is ordered. `drone.md` already says the physics parameters live in one struct so a real airframe can be dropped in. Two changes are needed, and Stage 2a is not yet written, so now is the cheap time to make them:

- **Make the body hold N motors, not four.** Each motor gets a position, a spin direction and a `Motor`. The torque sums in [`Stage2-a.md`](Stage2-a.md) become loops over that list. The quad is then one list and the octocopter another.
- **Make the mixer a table.** One row per motor, with a roll, pitch and yaw factor. The quad's four formulas are a four-row table.

Everything else (the quaternion, the gyro model, the PIDs, the estimator, the tests) is the same code.

What the simulation will show for this airframe:

- **Slower dynamics.** High inertia and slow motors mean lower gains and a slower response than the quad. The Stage 1 gains will not carry over; retune on the octocopter parameters.
- **Yaw is weak,** as on any multirotor, and worse with slow motors.
- **A motor failure can be tested.** Zero one motor in the simulation and see whether the controller holds level. An octocopter has the thrust to survive it; whether my controller does is a thing to find out here and nowhere else.

### The mixer

The T15 is a flat octocopter. Seen from above with the nose at the top, the arms sit at 22.5°, 67.5° and so on, with spin directions alternating. Keeping this project's order (clockwise from the front left) and its signs (positive roll is right side down, positive pitch is nose up, positive yaw is nose right):

| Index | Angle from nose | Spins | roll | pitch | yaw |
|---|---|---|---|---|---|
| 0 | −22.5° (front, left of nose) | CW | +0.38 | +0.92 | − |
| 1 | +22.5° | CCW | −0.38 | +0.92 | + |
| 2 | +67.5° | CW | −0.92 | +0.38 | − |
| 3 | +112.5° | CCW | −0.92 | −0.38 | + |
| 4 | +157.5° | CW | −0.38 | −0.92 | − |
| 5 | −157.5° | CCW | +0.38 | −0.92 | + |
| 6 | −112.5° | CW | +0.92 | −0.38 | − |
| 7 | −67.5° | CCW | +0.92 | +0.38 | + |

The roll factor is `−sin(angle)` and the pitch factor is `cos(angle)`: a motor further to the side does more for roll, and one further forward does more for pitch. Each column still sums to zero, so the three commands leave the total thrust alone and do not disturb each other.

The motor order and spin directions are my choice, since the code is mine. They only have to match the wiring, and the "two together" tests in Stage 2a check the code side of that.

## Tuning it without damaging it

A 6 kg machine with eight 15 inch carbon propellers is not something to tune by trial and error in free flight. The propellers can cause serious injury, and the stored energy in a 16000 mAh pack is large. The order below moves risk as late as possible.

1. **Host simulation with the octocopter parameters.** Tune there, including noise, delay and the ±30% margin test from Stage 1d. Add the motor-failure test.
2. **Firmware on the emulator** (Stages 4 and 5 of `drone.md`). This proves the driver, the eight PWM outputs, the timing and the arming logic with no hardware at risk.
3. **Bench test, propellers off.** Real board, real ESCs, real motors, frame strapped down. Check each motor's index and spin direction against the mixer table, one at a time. Tilt the frame by hand and confirm the motors on the low side speed up. A wrong sign found here costs nothing; found in the air it is a flip on take-off.
4. **Two-motor seesaw rig, propellers on.** Before assembling the frame, mount two of the eight motors on a bar with a pivot, as in [`Stage1-b.md`](Stage1-b.md). This measures the real thrust and motor time constant, corrects the simulation, and tunes the rate loop on real hardware with the propellers behind a guard. It needs only the motors, ESCs, propellers and battery, so it can happen before the frame is even in stock. See [The seesaw rig](#the-seesaw-rig).
5. **Tethered, low hover.** Whole aircraft, outdoors, tied down with slack lines so it can lift 20–30 cm and tilt a little but cannot flip or leave. Start with gains at about half the simulated values and work up.
6. **Free hover** only after the tethered one is boring.

Protections to build into the firmware before step 3. All of them can be tested in the simulation and the emulator first:

- **Arming:** motors stay off until an explicit arm command with the throttle at zero.
- **Kill switch:** one transmitter switch stops all motors at once, checked every loop.
- **Signal loss:** no valid receiver frame for a short time stops the motors.
- **Tilt cut-off:** past a set angle (say 45°) the motors stop. Already planned in Stage 3.
- **Throttle ceiling:** a compile-time cap, set low during tethered tests so it cannot climb hard whatever the PIDs ask.
- **PID output limits:** kept small, as in Stage 2a, so one bad axis cannot saturate the motors.
- **Loop watchdog:** if the 1 kHz loop stalls, the PWM outputs go to idle.

And outside the firmware:

- **An XT90-S arming plug** reachable without leaning over the propellers.
- **Propellers off whenever the board is on USB** or code is being flashed.
- **Nobody inside the propeller plane** when it is armed.

## The seesaw rig

Step 4 above, in detail. The rig is the seesaw from [`Stage1-b.md`](Stage1-b.md) built for real: one axis, two motors, bolted to a bench. It cannot lift off, flip or fly away.

### What to buy first

The purchase splits in two. The first half is everything except the frame:

- **Propulsion:** motors, ESCs and propellers. The rig needs two of each, so buy two now and six later, or all eight at once.
- **Power:** the 6S battery and its charger.
- **Electronics:** the SAME70 Xplained board, the gyro breakout and the 5 V UBEC.

All of these go on the T15 later. With two motors this first half is very roughly $700–1050; with all eight, about $1250–1700.

### Rig parts

None of this is specialist hardware. The costs are rough figures from memory, not looked up.

| Part | What to buy | Where | Rough cost |
|---|---|---|---|
| Bar | 1 inch (25 mm) square aluminium tube, cut to 1 m | hardware store (metal stock rack, near the fasteners) or McMaster-Carr | $15–25 |
| Pivot | 8 mm steel rod about 15 cm long, two "KP08" pillow block bearings, two 8 mm shaft collars | Amazon; sold as 3D printer parts | $10–15 |
| Stand | two 2×4 uprights about 40 cm tall, and a plywood base | hardware store | $10–15 |
| Hold-down | two C-clamps, or sandbags on the base | hardware store | $10–15 |
| Fasteners | M3 screws, nuts and standoffs for the motors and gyro; wood screws for the stand | hardware store | $5–10 |
| End stops | two wood posts | scrap | none |
| **Total** | | | **$50–80** |

Tools: a drill, an 8 mm bit and a 3 mm bit, a hacksaw and a screwdriver.

### Sketch

From the front, looking along the pivot rod:

```
      propeller                                     propeller
   ───────┬───────                               ───────┬───────
        ┌─┴─┐                gyro                     ┌─┴─┐
        │ M │                ┌──┐                     │ M │
   ┌────┴───┴────────────────┴──┴─────────────────────┴───┴────┐
   │                          ●  pivot rod                     │  bar, 1 m
   └─────────────────────────┬──┬──────────────────────────────┘
                             │  │
               ┌─┐           │  │  upright         ┌─┐
          stop │ │           │  │  40 cm      stop │ │
   ════════════╧═╧═══════════╧══╧══════════════════╧═╧════════════
                     plywood base, clamped to the bench

        │◄──────── 0.48 m ────────►│◄──────── 0.48 m ────────►│
```

From the side, looking along the bar:

```
                     bar, end on
          collar      ┌─────┐      collar
   ═══╤═════╪═════════╡     ╞═════════╪═════╤═══   8 mm rod
    ┌─┴─┐             └─────┘             ┌─┴─┐
    │ B │  bearing               bearing  │ B │
    ├───┤                                 ├───┤
    │   │                                 │   │
    │   │  upright               upright  │   │
    │   │                                 │   │
   ═╧═══╧═════════════════════════════════╧═══╧═
                       base
```

The ESCs sit on the bar next to their motors. The battery and the SAME70 board sit on the base.

### Assembly

1. **Drill an 8 mm hole sideways through the bar** at its exact centre and push the rod through.
2. **Screw a bearing to the top of each upright** and sit the rod in them. Fit a shaft collar each side of the bar so it cannot slide along the rod.
3. **Bolt a motor at each end,** shaft pointing up, the same distance from the centre. Take the hole pattern from the motor's drawing. One propeller is CW and the other CCW.
4. **Screw the gyro breakout to the bar at the centre** on M3 standoffs. Tape is not rigid enough.
5. **Fit the stops:** a post 25 cm either side of the centre, with its top 14 cm below the bar. That limits the tilt to about 30° and sits below the propellers.
6. **Clamp the base to the bench.**

Things to get right:

- **Height.** At 30° the bar's end drops about 25 cm, so the pivot must be at least 35–40 cm above the base.
- **Balance.** The pivot must be at the bar's balance point. If the weight hangs below the pivot, the bar levels itself like a pendulum and makes the controller look better than it is. With everything mounted, the bar should stay at whatever angle it is left at.
- **Wiring.** Use soft silicone wire with a slack loop from the battery to the bar, so the cables do not act as a spring.
- **Measure the real arm length** from the pivot to each motor shaft and use that number in the simulation.

Easier alternatives:

- **No bearings:** an M8 bolt through the bar and the two uprights, with nylon washers. It works for a first try and has more friction.
- **No drilling for mounts:** a 1 m length of "2020" aluminium extrusion from Amazon takes sliding T-nuts, so the motors and gyro bolt on anywhere and can be moved.

### What to do on it

1. **Measure thrust.** Run one motor pushing down on a kitchen scale at several throttle settings. This replaces the guessed thrust coefficient.
2. **Measure motor lag.** Step the throttle and log how fast the thrust follows. This replaces the guessed motor time constant.
3. **Record real gyro noise.** Log the gyro with both motors running. Propeller vibration is the largest thing the simulation cannot predict.
4. **Put those numbers into the seesaw simulation** and check that it now behaves like the real one.
5. **Close the rate loop** with the firmware's own PID, starting at low gains with a throttle cap. Then try the angle loop.

### What carries over to the octocopter

- **The corrected motor and noise model.** These belong to the motor, ESC and propeller, so they are the same on the T15.
- **Proven firmware.** The gyro driver, PWM output, loop timing, arming and kill switch have all run on real hardware.
- **D-term filter settings** that cope with real vibration.

The gains do not carry over, because the bar's inertia is nothing like the octocopter's. The octocopter's starting gains come from putting its mass and inertia into the corrected simulation.

### Limits

- **It is still dangerous close up.** Two 15 inch carbon propellers need a barrier or a guard. Stand out of the propeller plane, and keep the kill switch and arming plug within reach.
- **One axis only.** Yaw, the eight-motor mixer and coupling between axes are first tested on the tethered hover.
- **Pivot friction** damps the bar a little, which a flying drone does not have.

## Is it the right first build?

`drone.md` recommends a small trainer first, and that reasoning still holds: a first flight controller usually crashes, and on a 7 inch quad that costs a few dollars.

The octocopter has real points in its favour:

- **Slow dynamics are forgiving** of loop rate, delay and sensor noise.
- **It can survive a motor or ESC failure,** which a quad cannot.
- **It reaches the payload goal** without a second build.

Against it:

- **A crash is expensive.** An arm, a motor and a pair of propellers is $100 or more each time, and the frame's parts are hard to find.
- **It is more dangerous** to stand near while tuning.
- **It takes the whole budget,** leaving little for spares.

A middle path that keeps the T15 as the goal: buy the propulsion first (motors, ESCs, propellers, battery, charger), build [the two-motor seesaw](#the-seesaw-rig), and tune on that while the frame is sourced. None of that money is wasted, and the riskiest learning happens on a rig that cannot fly away.

## Sources

- [Tarot T15 TL15T00 frame, rcdrone.top](https://rcdrone.top/products/tarot-t15-tl15t00-octocopter-frame)
- [Tarot RC T15, air-rc.com](https://www.air-rc.com/aircraft/Tarot-RC_T15_T15)
- [Tarot T18 TL18T00 frame, GetFPV](https://www.getfpv.com/tarot-t18-1270mm-octocopter-foldable-frame-tl18t00.html)
- [Tarot T18 TL18T00 frame, rcdrone.top](https://rcdrone.top/products/tarot-t18-octocopter-frame-tl18t00)
- [Tarot T18 TL18T00 frame, QWinOut](https://qwinout.com/products/tarot-t18-fpv-octacopter-uav-octocopter-frame-tl18t00-25mm-carbon-fiber-1270mm-11kg-multi-rotor-for-photography-50-off)
- [Tarot 5008 340KV motor TL96020, Flying Tech](https://www.flyingtech.co.uk/product/tarot-5008-340kv-6s-multirotor-brushless-disc-motor-tl96020/)
- [Tarot folding propellers 1555–1965 with holder, rcdrone.top](https://rcdrone.top/products/tarot-1555-1655-1760-1865-1965-drone-propellers-cw-ccw-with-holder)
- [Tarot 1765 carbon folding propeller, Foxtech](https://store.foxtech.com/1765-carbon-fiber-folding-propeller)
- [EFT E410P agriculture drone frame, rcdrone.top](https://rcdrone.top/products/eft-e410p-agriculture-drone)
- [EFT E410P frame only, Indian Robo Store](https://indianrobostore.com/product/eft-e410p-10l-agricultural-drone-quadcopter-frame-only)
- [RJX1200D folding quadcopter frame, rcdrone.top](https://rcdrone.top/products/src-rjx1200d-rjx1200d-4-axis-carbon-fiber-umbrella-folding-quadcopter-frame-without-motor-mounts)
- [Hobbywing XRotor X8 series, Hobbywing Direct](https://hobbywingdirect.com/collections/xrotor-x8-series)
- [Hobbywing X8 power system, rcdrone.top](https://rcdrone.top/products/hobbywing-x8-power-system)
- [Hobbywing X8 power system pair, 3DXR](https://www.3dxr.co.uk/products/hobbywing-x8-power-system-1-pair-cw-ccw)
- [Tattu 12S 16000 mAh, rcdrone.top](https://rcdrone.top/products/tattu-16000mah-44-4v-12s-lipo-battery)
- [Herewin 12S 16000 mAh, rcdrone.top](https://rcdrone.top/products/herewin-12s-44-4v-16000mah-20c-lipo-agriculture-drone-battery)
- [Tarot 4114 320KV motor TL100B08, Flying Tech](https://www.flyingtech.co.uk/product/tarot-4114-320kv-6s-multirotor-brushless-motor-tl100b08-01)
- [Hobbywing XRotor Pro 40A, hobbywing.com](https://www.hobbywing.com/en/products/xrotor-pro-40a120)
- [XRotor Pro 40A, Hobbywing Direct](https://hobbywingdirect.com/products/xrotor-pro-40a-esc)
- [Tarot Martin 1555 folding propellers, rcdrone.top](https://rcdrone.top/products/tarot-martin-1555-15-5-inch-carbon-fiber-foldable-drone-propellers)
- [CNHL 16000 mAh 6S 15C, chinahobbyline.com](https://chinahobbyline.com/products/cnhl-16000mah-22-2v-6s-15c-lipo-battery-with-xt90-plug)
- [Tattu 16000 mAh 6S 30C, Pyrodrone](https://pyrodrone.com/collections/cinelifters/products/tattu-16000mah-30c-22-2v-6s1p-long-range-cinelifter-lipo-battery-xt90)
- [SAME70 Xplained Ultra DM320113, Future Electronics](https://www.futureelectronics.com/p/development-tools--analog-power/dm320113-microchip-2110665)
- [SAME70 Xplained Ultra user's guide DS70005389B](https://www.mouser.com/datasheet/3/282/1/SAME70_Xplained_Ultra_Evaluation_User's%20Guide_DS70005389B.pdf)
- [ICM-42688-P breakout, Elecrow](https://www.elecrow.com/icm-42688-p-ultra-low-power-imu-breakout.html)
- [ICM-42688-P Pmod board, DigiKey](https://www.digikey.com.au/en/products/detail/tdk-invensense/QCIOT-ICM42688P/25676496)
