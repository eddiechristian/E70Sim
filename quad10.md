# First Airframe: Mark4 V2 10 inch Quad

**Decision (2026-11-18):** the first real aircraft is a small quad on the **Mark4 V2 10 inch** carbon frame. It is the "small trainer" of [`drone.md`](drone.md#recommended-order). The larger options in [`octocopter.md`](octocopter.md) stay as later builds that reuse the same board and firmware.

**Bought (2026-11-18):** the frame, four 3115 900KV motors, the ESC, propellers, two batteries, the 5 V/9 V regulator, the charger, the SAM E70 Xplained Ultra board and the gyro board. [What is bought](#what-is-bought) lists them. Still to buy: a radio and small parts.

Prices were looked up on 2026-11-18. Those marked "paid" are from the order; the others are single listings, not quotes. None includes shipping or import duty.

The frame figures below come from other sellers of the same frame. The listing first noted for it, <https://rcdrone.top/products/mark4-v2-10-inch-fpv-racing-freestyle-drone-frame-kit>, turned out to be a kit with motors and electronics, sold out when checked, and gives no frame dimensions. **Measure the real frame when it arrives.**

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

### What is bought

| Item | Qty | Paid | Notes |
|---|---|---|---|
| Mark4 V2 10 inch frame | 1 | not recorded; $49–73 in other listings | |
| 3115 900KV motor | 4 | not recorded; $80–112 for four | 19×19 mm mount, 5 mm shaft. The two rcdrone.top 3115 motors both weigh about 110 g with leads; which one was bought is not recorded |
| OddityRC Vortex S60 4-in-1 ESC | 1 | $56.79 | 60 A (70 A peak), 3–8S, 30.5×30.5 mm. See [ESC choice](#esc-choice) |
| Gemfan 1050 three-blade propellers | 2 packs of 3 pairs | $59.96 | 10×5, glass-fibre nylon, 17 g each, 5 mm shaft. Six pairs: two on the aircraft and four spare |
| GNB BA-1 4S 5000 mAh 40C LiPo, XT60 | 2 | $139.72 | 475 g each, 39 × 45 × 138 mm, charge at 1C to 5C |
| Matek Micro BEC12S regulator | 1 | $11.80 | 6–60 V in; 5 V, 9 V or 12 V out (5 V as delivered); 2 g. See [Powering the board](#powering-the-board) |
| HOTA D6 Pro charger | 1 | $127.82 | mains powered, 200 W, two channels of up to 15 A, 1–6S |
| **The rcdrone.top order, without frame and motors** | | **$396.09** | |
| SAM E70 Xplained Ultra board (DM320113) | 1 | $190.84 | DigiKey |
| TDK EV_ICM-42688-P gyro evaluation board | 1 | $34.07 | DigiKey. See [The gyro board](#the-gyro-board) |
| Estimated tariff | | $2.04 | |
| **The DigiKey order** | | **$226.95** | |

### Still to buy

| Item | Qty | Cost | Notes |
|---|---|---|---|
| Radio transmitter and receiver | 1 | $100–250 | rough figure; needs a switch for the motor kill |
| 2.1 mm barrel plug with lead | 1 | a few dollars | from the regulator to the Xplained board |
| XT60 charge lead | 1 or 2 | a few dollars | the charger's listing includes only a mains cord and a manual |
| Capacitors, battery lead, wire, standoffs, adapter plate, landing legs | | $20–30 | rough figure; the ESC's listing does not say what comes with it and recommends two capacitors. See [Landing legs](#landing-legs) |
| LiPo bag, arming plug or smoke stopper | | $20–30 | rough figure |
| Jumper leads or a small adapter board, gyro to Xplained board | | a few dollars | for the bench; soldered or locking for flight |

Optional, for height above the ground ([`altitude.md`](altitude.md) explains the choice and the wiring):

| Item | Qty | Cost | Notes |
|---|---|---|---|
| Barometer: Adafruit BMP390 breakout (product 4816) | 1 | about $11 | DigiKey; I2C, on the gyro's bus |
| Lidar: Benewake TF-Luna | 1 | about $13–25 | rcdrone.top lists it from $13.41; needs 5 V, signals are 3.3 V |

These are not in the total below.

### Total

| | Cost |
|---|---|
| rcdrone.top order | $396 |
| DigiKey order | $227 |
| Frame and motors | about $130–185 |
| Still to buy | about $145–320 |
| **Total** | **about $900–1130** |

The first estimate was $670–1100 with the board. The low end has risen for three reasons: two batteries in place of one, batteries that cost more at this shop than the CNHL packs first priced, and a 200 W charger in place of a rough $50–100.

A 6S 5000 mAh pack for later is $60–65 (CNHL; about 810 g) or about $106 at rcdrone.top (HRB, XT60).

### The bundle

rcdrone.top also sells this frame as a $299 kit with four 3115 900KV motors, a 60 A 4-in-1 ESC, 1050 propellers and an F722 flight controller. Bought separately, the same parts without the flight controller come to about $205–280. The flight controller is not needed, because the SAME70 replaces it. The bundle is only worth it if the separate parts are out of stock.

### ESC choice

The ESC is the **OddityRC Vortex S60**, bought for $56.79 at rcdrone.top on 2026-11-18.

| | |
|---|---|
| Current per channel | 60 A continuous, 70 A peak |
| Battery | 3–8S |
| Firmware | BLHeli_S / Bluejay |
| Signal protocols | PWM, OneShot, DShot 150/300/600, MultiShot |
| Current sensor | yes |
| Mounting | 30.5×30.5 mm, M3 |
| Size | 43 × 48.8 mm |

It was chosen over two other 60 A boards on the same site for one reason above all: its listing states that it accepts plain PWM, which is what the SAME70 will send first. The others list DShot only.

What its listing does not say:

- **What is in the box.** No capacitor, battery lead or cable is listed. Check when it arrives; the "capacitors, battery lead, wire" line under [Still to buy](#still-to-buy) covers what is missing.
- **Capacitors.** The listing recommends two.
- **The signal plug's pinout.** It has to be read from the board's silkscreen or the manual before wiring it to the SAME70.
- **A 5 V output.** None is mentioned, which is why the separate regulator was bought.
- **Weight.**

The other two boards looked at on the same site:

| ESC | Price | Notes |
|---|---|---|
| JHEMCU RuiBet 65A | $55 (60 A version $49) | 3–6S, BLHeli_S; pinout given (VCC, GND, NC, CUR, IN4–IN1); comes with XT60, 12AWG wire, a 1000 µF capacitor and cable; no BEC; lists DShot600 only |
| HGLRC 60A 6S V1 | $54 | 2–6S, BLHeli_S; comes with XT60, lead, a 470 µF capacitor and cable; lists DShot only |

The SpeedyBee BLS 60A, at $47–61 from other shops, was the earlier placeholder.

### Things to check when the parts arrive

- **Battery connector.** The batteries have XT60 plugs. The ESC's listing does not say which lead it comes with, if any; fit an XT60 lead if it has none.
- **Motor wires reach the ESC.** On a 10 inch frame the arms are long. One of the two motors has 30 cm leads and the other 25 cm; extend them if they are short.
- **The frame's real size and weight,** against the figures in [The frame](#the-frame).
- **Which way the motor mounting holes and the stack holes line up** with the motors and the ESC.

### Powering the board

The Xplained board cannot be fed straight from the battery: a 4S pack is 16.8 V when full. The Matek regulator steps it down.

The board's user guide (in `reference/same70/`) does not agree with itself about the barrel jack. Section 5.14 gives 5.5 V to 19 V at 1 A or more; the schematic labels the same input 5 V to 14 V, with a diode in series, and notes that at 5 V the board's own regulator only passes the voltage through. So exactly 5 V into the jack is at or below the bottom of the range, and the board's 5 V rail would sag.

- **Set the regulator to 9 V and feed the barrel jack** (2.1 mm). That is inside both stated ranges, and the board makes its own clean 5 V. This is the plan.
- **The alternative is 5 V into the board's 5V0 pin,** which the guide allows only within ±2%. That is tight for a small switching regulator under a changing load.
- **A receiver that needs 5 V** takes it from the board's 5 V pin.
- **The regulator has no protection against a reversed input** and has solder pads, not connectors. Check the polarity of the jack in the user guide, and of the regulator's input, before the first power-up.
- **It also covers the larger builds.** Its 60 V input survives a 12S pack, which `octocopter.md` says an ordinary regulator would not.

## The gyro board

The gyro is TDK's own evaluation board for the chip, the **EV_ICM-42688-P**. Its user guide, AN-000488, is in `reference/icm42688/`; the schematic is on page 5 and the connector and jumper tables on page 7. What follows is read from that guide.

### What is on it

- **The ICM-42688-P,** with its decoupling capacitors.
- **Two regulators,** making 3.0 V and 1.8 V from the board's supply input, `VIN`.
- **Two jumpers** that choose the chip's voltages.
- **10 kΩ pull-up resistors** on the clock, data and chip-select lines.
- **A 20-pin connector, CN1:** two rows of ten, 2.54 mm pitch, right-angled.
- **Four mounting holes.**

### Do not plug it into the Xplained board's EXT header

CN1 has 20 pins in two rows, the same shape as the Xplained board's EXT1 and EXT2 headers, but the pins are in different places. It was made for TDK's own host boards. Plugged straight into EXT1, the gyro board's supply pin would land on ground and its ground pins on signal lines. **Wire it pin by pin.**

### The connector

| CN1 pin | Signal | Use here |
|---|---|---|
| 19 | `VIN`, the supply to the two regulators | 3.3 V from the Xplained board |
| 11, 13 | ground | ground |
| 18 | SDA (I2C data), or SDI in SPI mode | I2C data |
| 16 | SCL (I2C clock), or SCLK in SPI mode | I2C clock |
| 3 | INT1 | the "data ready" interrupt |
| 5, 6 | INT2 / FSYNC | not used |
| 4 | CS (chip select) | leave unconnected for I2C |
| 20 | AD0 (I2C address), or SDO in SPI mode | leave unconnected for address `0x68` |
| 1, 2, 7, 8, 9, 10, 12, 14, 15, 17 | not connected | |

- **Pin 19 is from the schematic.** The guide's pin table leaves pin 19 out altogether, while the schematic shows it feeding both regulators. Check with a meter that pin 19 connects to the input of the regulators before applying power.
- **I2C is selected without doing anything.** The chip uses I2C when CS is high, and a pull-up on the board holds it high.
- **The address is `0x68` without doing anything.** A 100 kΩ resistor on the board holds AD0 low. That is the address `drone.md` plans for.
- **The I2C pull-ups are already there,** so none need adding.

### The jumpers

| Jumper | Position | Result |
|---|---|---|
| JP2 (chip supply, VDD) | pins 1–2 | **3.0 V** |
| | pins 2–3 | 1.8 V |
| JP1 (signal voltage, VDDIO) | pins 1–2 | **the same as VDD** |
| | pins 2–3 | 1.8 V |

**Set both to pins 1–2,** so that the chip and its signals run at 3.0 V. The SAME70's pins work at 3.3 V; 3.0 V signals read correctly there, and 3.3 V into a chip running at 3.0 V is within the 0.5 V the datasheet allows above the signal supply. With either jumper at 1.8 V the two boards must not be connected directly.

Check how the jumpers are set when the board arrives. Do not assume.

### Wiring to the Xplained board

On EXT1, which is the TWIHS0 controller the emulator and the gyro firmware already use:

| Gyro board, CN1 | Signal | Xplained EXT1 | SAM E70 pin |
|---|---|---|---|
| 19 | supply | 20 (3.3 V) | |
| 11 or 13 | ground | 19 (ground) | |
| 18 | I2C data | 11 | PA3 (TWD0) |
| 16 | I2C clock | 12 | PA4 (TWCK0) |
| 3 | INT1 | 9 | PD28 |

- **Five wires.** The interrupt wire can wait until the firmware uses it; the first driver can poll.
- **PD28 is the pin the BNO055's interrupt used,** so the emulator's wiring for a sensor interrupt already matches.
- **3.3 V into `VIN` is my choice, not the guide's.** The guide does not give a range for `VIN`. The 3.0 V regulator needs a little more than 3.0 V, which 3.3 V gives. From the regulators' part numbers 5 V should also be safe, but I have not checked their datasheets, and 3.3 V keeps every pin of the gyro board at or below the Xplained board's own level.
- **SPI later** uses the same connector: CS on pin 4, clock on 16, data in on 18 and data out on 20.

### On the aircraft

- **Jumper wires are for the bench only.** On a flying frame they shake loose, and a lost gyro wire is a crash. For flight, solder the five wires or use a small adapter board with a locking connector.
- **Mount the board by its four holes,** rigidly, at the centre of the frame. See [Mounting the Xplained board](#mounting-the-xplained-board).
- **Note which way the chip's axes point.** The chip's own x, y and z are printed in its datasheet (and drawn on page 4 of the guide). They have to be matched to the body frame of [Stage 2a](Stage2-a.md#the-body-frame), x forward, y right, z down, by the way the board is turned and by sign changes in the driver. Getting one sign wrong is the classic flip on take-off; the bench test by hand is where it is caught.

## Driving the ESC from the SAME70

A 4-in-1 ESC is four ESCs on one board. It has a small connector with battery voltage, ground, a current-sense output and four signal inputs, one per motor.

- **Signal.** The Vortex S60's listing names PWM, OneShot, DShot and MultiShot. PWM here is the ordinary 1000–2000 µs pulse. Still confirm with one motor on the bench before wiring all four.
- **Start with plain PWM** at 400 Hz from `PWM0`. It is the simplest thing to generate and to model in the emulator. OneShot125 (the same idea with 125–250 µs pulses) is an easy later step. DShot is a serial bit stream and would need more firmware and a new emulator model.
- **Throttle calibration.** With plain PWM the ESC has to learn the pulse widths that mean zero and full throttle. Do this once, propellers off.
- **Signal level.** The SAME70's 3.3 V output is what flight controllers use, so this should work directly.
- **Motor direction** is set by which two of a motor's three wires are swapped, or in the ESC's configuration tool. Check each against the Stage 2a table, propellers off.

## Battery: 4S first

On 6S this aircraft is very overpowered for a first self-written controller.

| | 4S | 6S |
|---|---|---|
| Take-off weight | about 1.5 kg | about 1.8 kg |
| Max thrust per motor | about 1.8 kg (my estimate) | about 4 kg (one vendor's claim) |
| Total thrust ÷ weight | about 5 | about 9 |
| Hover, as a share of full thrust | about 20% | about 11% |
| Hover command, if thrust goes as command squared | about 0.45 | about 0.33 |

- **Start on 4S.** The two packs bought are 4S. The motors are rated 3–6S. Less thrust means a mistake in the gains turns into a slower, smaller motion.
- **Add a throttle cap in firmware** as well, as planned for the larger builds.
- **ESC current.** One retailer's own test shows a 3115 900KV pulling 84 A at full throttle on 6S with a 10×5×3 propeller, above the ESC's 60 A continuous rating and its 70 A peak. On 4S, or with a cap, this does not arise.
- **The 4S thrust figure is an estimate** (thrust scaled by voltage squared from the 6S claim). The seesaw rig will measure the real one.

## Mounting the Xplained board

The frame's stack holes are 30.5 mm apart, made for a flight controller about 36 mm square. The Xplained Ultra is several times that size and has its own hole pattern, so it cannot bolt onto the stack.

This is an inconvenience, not a blocker:

- **Use an adapter plate.** A flat plate of 2–3 mm plywood, G10 or a 3D print bolts to the frame's top-plate standoffs, and the board bolts to it on its own standoffs. Measure the board and the top plate first; the board will probably overhang the sides.
- **The weight is not a problem.** The board and plate add perhaps 100–150 g to an aircraft with several kilograms of spare thrust.
- **Put the gyro, not the board, on the stack position.** The gyro board is small. Fix it firmly at the centre of the frame, where the flight controller would go, and run its wires up to the board. Where the board sits then matters little to the control.
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

Estimates for the simulation's parameter struct, on 4S, worked out from the weights of the parts bought. Replace them with measurements as the parts arrive.

| Parameter | Simulation now | This quad (estimate) |
|---|---|---|
| Mass | not used yet | 1.49 kg |
| Arm length `L` | 0.2 m | 0.2135 m (half of a 427 mm wheelbase) |
| `d = L / √2` | 0.141 m | 0.151 m |
| Max thrust per motor (`thrust_coeff`) | 6 N | about 18 N on 4S, about 40 N on 6S |
| `Ixx` (roll) | 0.015 kg·m² | about 0.013 kg·m² |
| `Iyy` (pitch) | 0.015 kg·m² | about 0.015 kg·m² |
| `Izz` (yaw) | 0.025 kg·m² | about 0.027 kg·m² |
| Yaw coefficient `c` | 0.015 m | not known; measure |
| Motor time constant | 0.04 s | perhaps 0.04–0.06 s; measure |
| Hover command | 0.7 is used | about 0.45 |

### Where the mass comes from

| Part | Mass | Source |
|---|---|---|
| Frame | 0.230 kg | other listings of the frame (227 and 235 g) |
| Four motors | 0.440 kg | 110 g each with leads, from the listings |
| Four propellers | 0.068 kg | 17 g each, from the listing |
| ESC | 0.020 kg | a guess; boards of this kind are 14–16 g and this one is larger |
| Regulator | 0.002 kg | from the listing |
| Battery | 0.475 kg | from the listing (±14 g) |
| Xplained board, plate, gyro, wiring, landing legs | 0.250 kg | a guess |
| **Total** | **1.485 kg** | |

About 0.27 kg of this is guessed. Weigh the finished aircraft.

### Where the inertia comes from

Inertia is each part's mass times the square of its distance from the axis, added up. The motors and propellers sit furthest out, so they decide most of it.

| Part | Treated as | `Ixx` | `Iyy` | `Izz` |
|---|---|---|---|---|
| Motors and propellers | four masses of 0.127 kg at the arm tips | 0.0116 | 0.0116 | 0.0232 |
| Arms | four rods of 0.03 kg from the centre to the tips | 0.0009 | 0.0009 | 0.0018 |
| Centre plates | a 0.11 kg plate, 200 × 60 mm | 0.0000 | 0.0004 | 0.0004 |
| Battery | a 0.475 kg block, 138 × 45 × 39 mm, lengthways, 4 cm above or below the rest | 0.0007 | 0.0013 | 0.0008 |
| Board and the rest | a 0.25 kg plate, 135 × 85 mm | 0.0002 | 0.0004 | 0.0005 |
| **Total, kg·m²** | | **0.0133** | **0.0146** | **0.0267** |

- **The motors and propellers are 80–87% of each figure,** and their masses and positions are the best-known numbers here. So the totals are probably good to 10–15%, which is inside the ±30% margin the gains are meant to tolerate.
- **Pitch is a little slower than roll,** because the battery lies lengthways.
- **The assumptions:** a square X with a 427 mm wheelbase; a frame split into 0.12 kg of arms and 0.11 kg of plates; and the sizes of the plates and the board. The swing test in [How accurate the model is](drone.md#how-accurate-the-model-is) measures the real values.

### What this means for the gains

The geometry and inertia are close to the simulated ones. The thrust is not: 18 N per motor against 6 N. At hover, one unit of PID output turns this quad about 2.3 times harder than it turns the simulated one (roll acceleration of about 730 rad/s² per unit, against 320). It is less than the three-fold thrust would suggest, because this quad hovers at a lower command, 0.45 against 0.7, where thrust changes more slowly with command.

So the rate gains chosen in the simulation are about 2.3 times too strong for this aircraft on 4S, and more than that on 6S. Retune in the simulation with these numbers before anything flies. The simulation's defaults are unchanged for now, so that the tests and the pictures of Stages 1 and 2 stay reproducible.

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
- [EV_ICM-42688-P evaluation board user guide, AN-000488 (a copy is in `reference/icm42688/`)](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/8927/EV_ICM-42688-P.pdf)
- [EV_ICM-42688-P evaluation board, DigiKey](https://www.digikey.com.au/en/products/detail/tdk-invensense/EV-ICM-42688-P/18634550)
- [Gemfan 1050 propellers, 3 pairs, rcdrone.top (bought)](https://rcdrone.top/products/gemfan-1050-propeller)
- [GNB BA-1 4S 5000 mAh battery, rcdrone.top (bought)](https://rcdrone.top/products/gaoneng-gnb-ba-1-lipo-battery)
- [Matek Micro BEC12S regulator, rcdrone.top (bought)](https://rcdrone.top/products/matek-system-mateksys-micro-bec-6-60v-to-5v-9v-12v-adj-fpv-racing-drone-rc-accessories-parts)
- [HOTA D6 Pro charger, rcdrone.top (bought)](https://rcdrone.top/products/hota-d6-pro-325w-15a-1-6s-dual-channel-ac-dc-smart-charger)
- [HRB 4S and 6S 5000 mAh batteries, rcdrone.top](https://rcdrone.top/products/hrb-2s-3s-5s-4s-6s-lipo-battery)
- [OddityRC Vortex S60 4-in-1 ESC, rcdrone.top (bought)](https://rcdrone.top/products/vortex-s60-8s-escbls)
- [JHEMCU RuiBet 60A/65A 4-in-1 ESC, rcdrone.top](https://rcdrone.top/products/jhemcu-ruibet-4in1-esc)
- [HGLRC 60A 6S V1 4-in-1 ESC, rcdrone.top](https://rcdrone.top/products/hglrc-60a-6s-v1-esc-for-racing-freestyle-fpv-drone)
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
