# Height above the ground: lidar and barometer (an option)

This is an option, not part of the plan's stages yet. [`drone.md`](drone.md#later) lists "altitude hold" under Later, and says it needs a barometer or a range sensor. This document chooses both, says how to fuse them, and suggests parts that fit the SAM E70 Xplained Ultra.

Nothing in Stages 1 and 2 needs it. Holding an angle needs only the gyro and the accelerometer. Height matters from Stage 3, where the simulation gains gravity: the throttle then decides whether the drone climbs or sinks, and nothing in the flight code yet knows which.

## What each sensor measures

| | Lidar rangefinder | Barometer |
|---|---|---|
| Measures | the distance to whatever is under the drone, by timing a pulse of light | air pressure, which falls as the drone climbs |
| Gives | height above the surface below | height above the place where it was zeroed |
| Good at | accuracy near the ground: a few centimetres | any height; no limit of range |
| Bad at | range (8–12 m for the small ones); bright sun; tilt; a surface that changes under it | noise of 10–30 cm; slow drift with the weather; the propellers' own wind |
| Updates | about 100 times a second | 25–100 times a second |

They fail in opposite ways, which is why having both is worth it:

- **The lidar is exact but short-sighted.** Near the ground, where a landing is decided by centimetres, it is the best information there is. Above its range it says nothing.
- **The barometer is vague but never blind.** It works at any height. But on its own it cannot tell 20 cm from the ground, and its zero wanders: the weather can move the pressure by an amount worth several metres in an hour.

This is the same situation as the gyro and the accelerometer in [the estimator](drone.md#the-estimator), and it gets the same kind of answer.

### The lidar's reading is a slant

The lidar is fixed to the body and points along the body's "down". When the drone tilts, the beam hits the ground further away:

```
height = range × cos(roll) × cos(pitch)
```

At 20° of roll a range of 2.00 m is a height of 1.88 m. The roll and pitch come from the estimator, which is already there. Past about 30° of tilt the reading is best ignored: the beam is far off to the side, and the correction multiplies any error in the angle.

### The barometer's reading is a pressure

Near the ground, pressure falls by about 12 Pa for each metre climbed. So:

```
height = (pressure at the ground − pressure now) / 12        (Pa, metres; near sea level)
```

The pressure at the ground is taken during the [start-up calibration](drone.md#calibration-before-flight), while the drone sits still, as the gyro bias is. The full formula, for larger heights, is in the sensor's datasheet.

Two numbers show the barometer's limits. A sensor that resolves 3 Pa resolves 25 cm. And a weather change of 1 hPa (100 Pa) in an hour, which is ordinary, moves the answer by 8 m in that hour, or about 14 cm a minute.

## Fusing them

There are three sources, not two. The accelerometer is the third: it feels the drone speeding up or slowing down vertically at once, long before the height has changed enough for the other two to notice.

| Source | Fast? | Drifts? | Use |
|---|---|---|---|
| Accelerometer (vertical part) | instant | badly: it must be added up twice to give a height | the moment-to-moment motion |
| Lidar | 100 Hz | no | the truth near the ground |
| Barometer | 25–100 Hz, noisy | slowly | the truth everywhere else |

The fusion is done in two steps.

### Step 1: one reference height from the lidar and the barometer

```
if the lidar's reading is usable:
    reference = lidar height
    offset    = slowly follows (barometer height − lidar height)
else:
    reference = barometer height − offset
```

- **While the lidar can see the ground, it is believed,** and the barometer is quietly re-zeroed against it. `offset` is what the barometer is wrong by; it is updated with a time constant of a few seconds, so the barometer's noise does not get into it.
- **When the lidar loses the ground, the barometer takes over** from where the lidar left off, with its error already removed. There is no jump at the hand-over.
- **When the lidar finds the ground again,** on the way down, it takes over again. Any drift the barometer collected while high up shows as a step at that moment. The step is fed in gradually, over a second or so, and not all at once.

"Usable" means all of: the range is inside the sensor's limits (for an 8 m sensor, say 0.2 m to 7 m); the sensor reports a good signal strength; and the tilt is under 30°.

### Step 2: the accelerometer, corrected by the reference

This is a complementary filter, as in Stage 2b, with one more level because acceleration has to be added up twice.

```
vertical acceleration = the accelerometer's reading turned into the world frame, with gravity taken off

velocity += vertical acceleration × dt
height   += velocity × dt

error     = reference − height
height   += k1 × error × dt
velocity += k2 × error × dt
```

The first three lines predict from the accelerometer. The last three pull the prediction towards the reference. `k1` and `k2` set how hard: large values trust the reference, small ones trust the accelerometer. They can be written as one time constant, as the estimator's blend was.

The filter gives two things: a **height** and a **vertical speed**. The speed is as valuable as the height. An altitude-hold loop is a cascade like the angle and rate loops: an outer loop turns a height error into a climb rate to aim for, and an inner loop turns the climb-rate error into throttle.

### Things to decide when it is built

- **The lidar follows the ground, the barometer does not.** Flying over a table, the lidar's height drops by the height of the table. Near the ground that is what a landing needs. For holding a level path it is wrong. A sudden jump in the lidar with no matching acceleration is a change in the ground, not in the drone, and can be treated as one.
- **Ground effect and prop wash.** Close to the ground the propellers raise the pressure under the drone, and the barometer reads low. Trusting the lidar there avoids the problem.
- **The barometer needs shade and still air.** Light falling on the sensor and wind across it both change the reading. A small piece of open-cell foam over it is the usual cure.
- **Update rates differ.** The control loop runs at 1 kHz, the lidar at about 100 Hz and the barometer slower. The filter predicts every step and corrects only when a new reading has arrived.

## Suggested parts

All three are small breakout boards or modules with 3.3 V signals, which is what the SAM E70's pins use.

### Barometer

| | Adafruit BMP390 (product 4816) | Adafruit DPS310 (product 4494) |
|---|---|---|
| Chip | Bosch BMP390 | Infineon DPS310 |
| Price | $10.95 at DigiKey, in stock when checked | $6.95 at Adafruit |
| Interfaces | I2C and SPI | I2C and SPI |
| Supply | 3–5 V; regulator and level shifting on the board | the same |
| Maker's headline figure | relative accuracy ±3 Pa, about ±25 cm | precision ±0.2 Pa, about ±2 cm; relative accuracy ±6 Pa |
| I2C address | `0x77`, or `0x76` with a jumper | `0x77`, or `0x76` with a jumper |

**Suggested: the BMP390.** It is the one most often found on current flight controllers, DigiKey has it, and one more order there is easy. The DPS310 is as good a choice and cheaper; flight controllers use it widely too. Either will do, and the fusion above does not depend on which.

The accuracy figures and the addresses are from the makers' datasheets and Adafruit's pages as I remember them, not from the searches listed below, which confirmed only the prices and interfaces. Check them against the datasheet when the part is chosen.

### Lidar

| | Benewake TF-Luna | Benewake TFmini Plus |
|---|---|---|
| Range | 0.2–8 m indoors on a bright surface; 0.2–2.5 m on a dark one | 0.1–12 m |
| Price | from $13.41 at rcdrone.top; about $21–25 elsewhere | about $40–50 |
| Interfaces | UART or I2C (address `0x10`), 3.3 V signals | UART or I2C |
| Supply | 3.7–5.2 V, 70 mA average, 150 mA peak | 5 V |
| Weight | about 5 g | 12 g |
| Outdoors | range falls in bright sun | sealed to IP65; rated for 70 klux of ambient light |
| Update rate | 100 Hz as delivered | 100 Hz as delivered |

**Suggested: the TF-Luna to start.** It is cheap, light, and its 8 m covers a first hover with room to spare. rcdrone.top lists it, where the rest of the airframe came from. Its range on a dark surface and in bright sun is much shorter than the headline figure, so the fusion's "usable" test matters.

**The TFmini Plus is the upgrade** if the TF-Luna proves too weak outdoors: longer range, sealed, and built for sunlight.

Two that I would not start with:

- **MicoAir MTF-01** ($42.80 at rcdrone.top; some of its pages showed it sold out). It combines an 8 m lidar with an optical-flow camera, which measures sideways movement over the ground. That is what a later position hold needs. But it speaks a message protocol made for existing flight controllers, which is more firmware to write than reading a distance.
- **ST VL53L1X** and its relatives. A few dollars, I2C, but about 4 m at best and much less in daylight. Fine indoors, not for this.

### Cost

| Item | Cost |
|---|---|
| BMP390 breakout | about $11 |
| TF-Luna | about $13–25 |
| Leads, foam | a few dollars |
| **Total** | **about $30–40** |

## Wiring to the Xplained board

Both can share the I2C bus the gyro is on: EXT1 pins 11 and 12, which are TWIHS0. Every device on it has its own address.

| Device | Address |
|---|---|
| ICM-42688-P gyro | `0x68` |
| BMP390 barometer | `0x77` |
| TF-Luna lidar | `0x10` |

| Device | Supply | Data | Clock | Notes |
|---|---|---|---|---|
| Barometer | 3.3 V (EXT1 pin 20) | EXT1 pin 11 (PA3) | EXT1 pin 12 (PA4) | ground on EXT1 pin 19 |
| TF-Luna | 5 V | EXT1 pin 11 (PA3) | EXT1 pin 12 (PA4) | its pin 5 tied to ground selects I2C; left open it is UART |

- **The TF-Luna needs 5 V,** not 3.3 V. The Xplained board has a 5 V pin on its power header. Its signals are still 3.3 V, so they connect straight to the SAM E70. Confirm the pin on the board's user guide before wiring; I have not checked it here.
- **Pull-ups are already there.** The gyro board and the Adafruit board each have their own on the two bus lines.
- **Scan the bus first.** A first firmware test that asks every address and prints the ones that answer will confirm all three before any driver is written.

**One thing to watch: time on the bus.** At 400 kHz, reading the gyro's twelve bytes takes about a third of a millisecond, every millisecond. The barometer and the lidar add their own reads. It fits, but not by a wide margin. Two ways out, if it gets tight:

- **Put the lidar on a UART.** That is how it is delivered: it sends a nine-byte frame 100 times a second without being asked. It costs no bus time, and needs a serial receive pin and a small parser. EXT1 has a UART on pins 13 and 14.
- **Move the gyro to SPI,** which `drone.md` already lists under Later.

Mounting:

- **The lidar looks straight down,** from the underside, with nothing in its narrow beam: not a landing leg, not the battery strap.
- **The barometer sits away from the propellers' wind,** under foam, and out of direct light.

## Where it goes in the plan

Nothing here changes Stages 1 and 2. If the option is taken, it adds work in three places:

| Where | What |
|---|---|
| Stage 3, host simulation | Once the body has a height: a lidar model (range from height and tilt, a few centimetres of noise, nothing beyond its range) and a barometer model (pressure from height, noise, a slow drift) in `drone/physics/`. The fusion in `drone/flight/`, tested against the true height the simulation knows. Then an altitude-hold loop on the throttle. |
| Stage 4, emulator | Models of the two chips as I2C devices, next to the ICM-42688-P. |
| Stage 5, firmware | Drivers for the two chips, feeding the same fusion code. |

The simulation is again where the design is settled. It knows the true height, which a real drone never does, so the fused height can be plotted against it: on the ground, through the hand-over at the top of the lidar's range, and over a step in the ground.

## Sources

- [Benewake TF-Luna, datasheet A05 (Waveshare copy)](https://files.waveshare.com/upload/c/c3/SJ-GU-TF-Luna_A05_Datasheet.pdf)
- [Benewake TF-Luna, RobotShop](https://www.robotshop.com/products/benewake-tf-luna-8m-lidar-distance-sensor)
- [Lidar sensors, rcdrone.top](https://rcdrone.top/collections/lidar-sensors)
- [MicoAir MTF-01, rcdrone.top](https://rcdrone.top/products/mtf-01-micoair-optical-flow-lidar-sensor)
- [MicoAir MTF-01, ArduPilot documentation](https://ardupilot.org/copter/docs/common-mtf-01.html)
- [Benewake TFmini, PX4 documentation](https://docs.px4.io/main/uk/sensor/tfmini.html)
- [TFmini Plus, DFRobot wiki](https://wiki.dfrobot.com/SEN0309/)
- [Adafruit BMP390, product 4816](https://www.adafruit.com/product/4816)
- [Adafruit BMP390, DigiKey](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4816/13543385)
- [Adafruit DPS310, product 4494](https://www.adafruit.com/products/4494)
