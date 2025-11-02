# README1 — StarterBotTeleOpHSNew.java (driver-friendly explanation)

This file explains the `StarterBotTeleOpHSNew.java` TeleOp in simple terms so non-Java developers (drivers, coaches, mentors) can understand what the code does, which buttons to press, and how to test/tune it safely.

Path: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/StarterBotTeleOpHSNew.java`

---

## Short summary (one-liner)
Press RIGHT BUMPER to enable the launcher (it will spin at a default preset). While enabled use A/X/Y to pick low/medium/high speeds. Press LEFT BUMPER to feed one shot (only if launcher is up to speed). Press B to immediately stop launcher and feeders; restart by pressing RIGHT BUMPER again.

---

## Hardware names expected by the code
Make sure your Robot Configuration (on the Robot Controller phone) has these device names:
- `left_drive` — left drive motor
- `right_drive` — right drive motor
- `launcher` — launcher motor (DcMotorEx / encoder-capable)
- `left_feeder` — left continuous-rotation feeder servo
- `right_feeder` — right continuous-rotation feeder servo

If your hardware uses different names, either rename the devices in the Robot Controller configuration or edit the Java file `init()` to match your names.

---

## Driver controls (complete mapping)
- RIGHT BUMPER (press): START/ENABLE the launcher. This is required to allow presets and shooting.
- A (press while launcher enabled): set launcher to LOW preset (safe/slow). Works only if launcher is already enabled by RIGHT BUMPER.
- X (press while launcher enabled): set launcher to MEDIUM preset (default). Works only if launcher is already enabled.
- Y (press while launcher enabled): set launcher to HIGH preset (fast). Works only if launcher is already enabled.
- LEFT BUMPER (press and release): request a single shot (feeder runs briefly). The feeder runs only if the launcher is enabled and up to speed.
- B (press): EMERGENCY/IMMEDIATE STOP — stops launcher and feeders and disables launcher mode; must press RIGHT BUMPER again to restart the launcher.
- Left stick / Right stick: drive the robot (arcade-style mixing is used in code).

Notes:
- Preset buttons (A/X/Y) do nothing if the launcher hasn't been enabled by RIGHT BUMPER.
- LEFT BUMPER supports one-shot-per-press: holding it down will not fire repeatedly.

---

## Key constants (where they live in the code and what they mean)
These values are at the top of the Java file and are the first place to look when tuning:
- `FEED_TIME_SECONDS` — how long the feeder servos run for a single shot (default: 0.20 seconds). Increase if your feeder needs more time to push a game element into the launcher, decrease if it overruns.
- `LAUNCHER_SPEED_A` — velocity for the LOW preset (e.g., 900 in encoder units).
- `LAUNCHER_SPEED_X` — velocity for the MEDIUM preset (default activation speed, e.g., 1125).
- `LAUNCHER_SPEED_Y` — velocity for the HIGH preset (e.g., 1400).
- `LAUNCHER_TARGET_VELOCITY` — the current target velocity being commanded to the launcher. The code updates this when presets are chosen.
- `LAUNCHER_MIN_VELOCITY` — the threshold that determines if the launcher is considered "up to speed". Typical value is `target - 50`.

Tuning tip: start with conservative (lower) `LAUNCHER_SPEED_*` values and short `FEED_TIME_SECONDS` during initial tests.

---

## Internal behavior (plain-language code walk-through)
1. Init phase (when you press INIT on the Driver Station):
   - The code finds motors/servos by the names listed above.
   - Drive motor directions and braking are set so forward stick moves the robot forward.
   - The launcher motor is put into a mode that allows the code to command a velocity and read the actual velocity (encoder-based control).
   - Feeder servos are set to stopped.
   - PIDF (control) parameters for the launcher are set to reasonable defaults — these help the launcher hold a steady velocity.

2. Start phase (when you press START):
   - The launcher does NOT start automatically. `launcherActive` is false until the driver presses RIGHT BUMPER.

3. Main loop (repeats many times per second while the mode is running):
   - Driving: joystick values are mixed and sent to `left_drive` and `right_drive` so the robot moves.
   - RIGHT BUMPER: if the driver presses RIGHT BUMPER the code sets `launcherActive = true`, sets the target velocity to the medium preset, and commands the launcher to spin.
   - B: if pressed, the code immediately stops launcher and feeders, and disables `launcherActive` (so presets and shooting are ignored until RIGHT BUMPER is pressed again).
   - Presets (A/X/Y): if `launcherActive` is true, pressing these buttons changes `LAUNCHER_TARGET_VELOCITY` to the corresponding preset and commands the launcher to the new speed.
   - Launcher command: whenever `launcherActive` is true the code commands the launcher to `LAUNCHER_TARGET_VELOCITY`. If `launcherActive` is false the launcher is commanded to stop.
   - LEFT BUMPER (shoot request): a one-shot is attempted when the button is pressed. The code checks two things before firing the feeder:
     - `launcherActive` must be true (launcher is enabled)
     - `launcher.getVelocity()` must be >= `LAUNCHER_MIN_VELOCITY` (wheel is up to speed)
     If both are true the feeders run for `FEED_TIME_SECONDS` and then stop. The code sets a latch so that holding LEFT BUMPER will not trigger multiple repeated feeds.
   - Telemetry: the code shows `Launcher Velocity (pre-shoot)` right before the feed check so the driver sees whether the launcher is ready.

---

## Safety checks & recommended test sequence (field checklist)
1. Confirm hardware names in Robot Controller match the device names above.
2. With the robot raised (launcher wheel free), select `StarterBotTeleOpHSNew` on the Driver Station and press INIT.
3. Confirm driving works: move the joysticks and ensure the robot moves correctly.
4. Press RIGHT BUMPER (launcher should start spinning at medium preset). Watch telemetry for `Launcher Velocity (pre-shoot)`.
5. After it stabilizes near the target velocity, press LEFT BUMPER once — the feeder should run briefly and stop.
6. Try A/X/Y while launcher is enabled to change speeds and watch velocity change on telemetry.
7. Press B — launcher and feeders should stop immediately. Confirm that LEFT BUMPER no longer fires feeder until RIGHT BUMPER is pressed again.

Safety notes:
- Always test with the launcher wheel unobstructed and clear of people. Use low preset speeds for initial tests.
- If feeders spin the wrong direction, either swap wiring or change the feeder direction in code (the code sets `left_feeder` reversed to make both feed together).

---

## Common troubleshooting
- Feeder doesn’t move: check the servo names and power wiring. Are they continuous rotation servos? Check servo direction (left feeder is reversed in code).
- Launcher doesn’t reach velocity: verify encoder/connection for the launcher motor and confirm the PIDF coefficients and target velocities are appropriate. Try lowering preset speeds.
- Feeder runs too long or too short: adjust `FEED_TIME_SECONDS`.

---

## Where to change behavior in the code (quick pointers for a mentor)
- To change presets and timing: edit the constants at the top of `StarterBotTeleOpHSNew.java` (`LAUNCHER_SPEED_A`, `LAUNCHER_SPEED_X`, `LAUNCHER_SPEED_Y`, `FEED_TIME_SECONDS`).
- To change which button activates the launcher: look for the RIGHT BUMPER handling in `loop()`.
- To modify the one-shot behavior or latch: look for `shotLatch` and `prevLeftBumper` logic.
- For hardware name changes: edit `hardwareMap.get(...)` calls in `init()`.

---

## Build / deploy
From the repo root (where `gradlew` is located):

```bash
# Build app (may take a few minutes the first time)
./gradlew assembleDebug
```

For quick iterations, open the project in Android Studio and run the `TeamCode` module to the Robot Controller device.

---

If you want, I can also:
- Produce a one-page driver cheat-sheet image with the button map and recommended test steps.
- Run a Gradle build here and report compile-time issues.

Tell me which of those you'd like next.