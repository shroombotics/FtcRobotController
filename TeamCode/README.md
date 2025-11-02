# TeamCode TeleOp Modes — Readme for non-Java developers

This short README explains two TeleOp Java files in plain, non-technical language so a coach, driver, or teammate can understand what the code does and how to use it on the field.

Files covered (paths relative to repo root):
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/StarterBotTeleopHS.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/StarterBotTeleOpHSNew.java`

If you want a one-line recommendation: use `StarterBotTeleOpHSNew` for driver control where the launcher only runs when the driver presses RIGHT BUMPER, and B immediately stops everything.

---

## What the programs control
Both programs control three main subsystems:
1. Drive (left and right motors) — used for driving the robot with the gamepad sticks.
2. Launcher (a wheel or pair of wheels spun at a specific speed) — used to launch a game element.
3. Feeders (usually two continuous-rotation servos) — run briefly to feed a single element into the launcher.

## Hardware names the code expects
Before running the code, make sure your Robot Configuration uses these device names (they appear in `hardwareMap.get(...)` in the code):
- Drive motors: `left_drive`, `right_drive`
- Launcher motor: `launcher` (must be an encoder-capable motor like a REV or goBILDA motor)
- Feeders (continuous rotation servos): `left_feeder`, `right_feeder`

If your configuration uses different names, either rename devices in the Robot Controller configuration or update the Java file.

---

## Common terms (plain English)
- "Spin-up / velocity control": the launcher is commanded to a particular wheel speed (not just a power), and the software reads the encoder to see how fast it is spinning. This makes shots repeatable.
- "Preset": one of three pre-configured speeds (low, medium, high) that drivers can choose with buttons.
- "Up to speed": the launcher is spinning fast enough to shoot reliably. The code uses a threshold called `LAUNCHER_MIN_VELOCITY` to check this.
- "One-shot-per-press": pressing the feeder button once feeds a single ball, even if the button is held down; you must release and press again to feed another.

---

## How `StarterBotTeleopHS.java` works (step-by-step)
This is the original example/variant with a small state machine to manage a shot cycle.

1. INIT (before START):
   - The code finds all motors and servos by their names.
   - It configures directions (so pushing the stick forward moves the robot forward) and braking behavior.
   - It sets the launcher to use encoder-based velocity control and sets PIDF values (these keep the launcher at a steady speed).

2. START (when the driver presses START):
   - The launcher is commanded to begin spinning at the current `LAUNCHER_TARGET_VELOCITY`.

3. LOOP (repeats many times per second while enabled):
   - Driving: joystick input is converted to left/right motor power.
   - Presets and velocity adjustments: the driver can change launcher speed using D-Pad or buttons (the code updates `LAUNCHER_TARGET_VELOCITY`).
   - Shot request: when the driver requests a shot (in this code example the right bumper's press event is used), a state machine runs through:
     - SPIN_UP: command the launcher to the target speed and wait until `launcher.getVelocity()` > `LAUNCHER_MIN_VELOCITY`.
     - LAUNCH: start the feeder(s) for `FEED_TIME_SECONDS`.
     - LAUNCHING: wait for feed timer to finish, stop feeders, and return to IDLE.
   - Telemetry: state, launcher velocity, and other helpful information is sent to the Driver Station screen.

Why use a state-machine? It sequences events safely: spin up -> feed -> stop feeders, without blocking (so the rest of the loop can continue to run and update telemetry).

---

## How `StarterBotTeleOpHSNew.java` works (step-by-step and what changed)
This file is derived from the original but is aimed at driver control with explicit activation & safety.

Goals implemented:
- The launcher MUST be activated by pressing RIGHT BUMPER (it does not start automatically on START).
- While the launcher is active, drivers can switch preset speeds with A, X, and Y.
- Pressing B immediately stops launcher and feeders; launcher will stay off until RIGHT BUMPER is pressed again.
- LEFT BUMPER will feed a single shot only if the launcher is active and has reached the minimum velocity threshold.

Main loop flow summary:
1. The code tracks button presses (it detects the moment a button is pressed rather than just whether it is held down). This lets the code trigger actions once per press.
2. RIGHT BUMPER (rising edge press):
   - `launcherActive` is set to true and the launcher is commanded to the default preset (the medium preset by default).
3. While `launcherActive` is true, pressing A/X/Y (rising edge) will change `LAUNCHER_TARGET_VELOCITY` to low/medium/high values.
4. If `launcherActive` is false, A/X/Y do nothing (this prevents accidental speed changes when the launcher is stopped).
5. B (rising edge):
   - Immediately stops the launcher and feeder and sets `launcherActive = false`. The launcher will not restart until RIGHT BUMPER is pressed again.
6. LEFT BUMPER (rising edge) requests a single shot:
   - The feeder only starts if `launcherActive` is true and `launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY`.
   - If the launcher is not yet at speed, the feeder is not started. While the driver still holds LEFT BUMPER down, the code keeps checking velocity; when the velocity becomes high enough, the feeders will start automatically and the feeding timer will run.
   - The feeders run for `FEED_TIME_SECONDS` and then stop. A `shotLatch` prevents multiple shots being fired while LEFT BUMPER remains held down.
7. Telemetry: the code prints `Launcher Velocity (pre-shoot)` before attempting to feed so drivers can see whether the launcher is ready.

Why this approach is useful for drivers:
- The launcher won't unexpectedly spin up during teleop; the driver must explicitly enable it with RIGHT BUMPER.
- The B button is a "panic stop" for the launcher and feeders.
- The left bumper feeding is safe: it only occurs when the wheel is up to speed.

---

## Controls summary (for `StarterBotTeleOpHSNew`)
- Right bumper (press): enable/start launcher at default speed (medium). Launcher stays running until B is pressed.
- A: set launcher to low preset (only if launcher is active)
- X: set launcher to medium preset (only if launcher is active)
- Y: set launcher to high preset (only if launcher is active)
- Left bumper (press): feed one element (only when launcherActive and the wheel is up to speed)
- B (press): immediately stop launcher and feeders; launcher remains disabled until RIGHT BUMPER is pressed again
- Left / Right sticks: drive the robot (arcade or tank style depending on the file variant; both files use a simple mixing method)

---

## How to tune values safely (recommendations)
- Start with a low `LAUNCHER_TARGET_VELOCITY` and test the wheel direction and behavior before increasing.
- Increase `FEED_TIME_SECONDS` slowly if the feeder does not reliably push the game element into the launcher.
- `LAUNCHER_MIN_VELOCITY` is normally set slightly below the target velocity (target - 50). If you want the shooter to be more permissive, lower the delta; if you want to be more conservative, raise it.
- PIDF values for launcher velocity control (`launcher.setPIDFCoefficients(...)`) may be tuned by an experienced mentor. If the launcher overshoots or oscillates when you change target velocity, the PIDF needs adjustment.

---

## How to test on the field — quick checklist
1. Verify hardware names in Robot Controller configuration match the names listed above.
2. Deploy the app or build the `TeamCode` module and load OpModes on the Driver Station.
3. Select `StarterBotTeleOpHSNew` on the Driver Station.
4. With the robot safely raised or launcher wheel unobstructed, press RIGHT BUMPER — launcher should spin up at the default medium speed.
5. Watch telemetry `Launcher Velocity (pre-shoot)` — it should show the wheel velocity increasing toward the preset.
6. When the velocity is at or above the minimum threshold, press LEFT BUMPER to feed a single element. Confirm feeder runs briefly.
7. Press B to stop the launcher and feeders. Attempt to press LEFT BUMPER — it should not feed while launcher is disabled.

---

## Building (optional quick commands)
From the repository root (where `gradlew` is located):

```bash
# Build the project (may take minutes the first time)
./gradlew assembleDebug
```

For rapid iteration, open the project in Android Studio and build the `TeamCode` module or use the IDE run configuration to install to your Robot Controller device.

---

## Where to edit if you want different behavior
- Change presets, feed time, or thresholds near the top of the Java files where constants are declared.
- Button mapping can be found in the `loop()` method of each file (look for checks on `gamepad1.a`, `gamepad1.x`, `gamepad1.y`, `gamepad1.right_bumper`, `gamepad1.b`, `gamepad1.left_bumper`).
- Hardware device names are in `init()` where `hardwareMap.get(...)` is called.

---

If you'd like, I can also:
- Add a short PDF/one-sheet cheat sheet for drivers with button mappings and a recommended test sequence.
- Run a Gradle compilation here and report any compile errors (if you want me to, say so and I'll start the build).

If any wording or example needs to be clarified for your team’s knowledge level, tell me what to simplify further and I will revise this README.