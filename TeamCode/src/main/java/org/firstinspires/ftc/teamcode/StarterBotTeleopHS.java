/*
 * Custom TeleOp based on StarterBotTeleOpHS.java
 * - Launcher runs in RUN_USING_ENCODER mode and begins spinning at a default velocity when START is pressed
 * - gamepad1.a, gamepad1.x, gamepad1.y select three preset launcher speeds
 * - left bumper (rising edge) requests a feed; feeder only begins if launcher velocity >= LAUNCHER_MIN_VELOCITY
 * - feeder (CRServo) runs for FEED_TIME_SECONDS then stops
 */
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleopHS", group = "StarterBotHS")
public class StarterBotTeleopHS extends OpMode {

    // Feeder timing
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Launcher velocity presets (A=low, X=medium, Y=high)
    double LAUNCHER_SPEED_A = 1260;
    double LAUNCHER_SPEED_X = 1175; // default
    double LAUNCHER_SPEED_Y = 1360;

    // Working target/minimum values
    double LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_X;
    double LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();

    // Simplified: launcher always in SPIN_UP (running) mode; feeding is a transient boolean
    private boolean isFeeding = false;

    // Edge detection for buttons
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevLeftBumper = false;
    // Latch to ensure one shot per bumper press
    private boolean shotLatch = false;

    // Drive telemetry
    double leftPower;
    double rightPower;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Use encoder-based velocity control for the launcher from the beginning
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // Initialize feeders stopped and directions consistent
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // Reasonable PIDF for velocity control (tune if required)
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized (Custom)");
        telemetry.setAutoClear(false);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // Start launcher spinning at the selected default speed immediately when START is pressed
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
    }

    @Override
    public void loop() {
        // Clear telemetry from previous loop to avoid accumulation
        telemetry.clearAll();

        // Basic arcade drive
        arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Handle preset buttons (rising edge)
        if (gamepad1.a && !prevA) {
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_A;
            LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("Preset", "A (low) set: %.0f", LAUNCHER_TARGET_VELOCITY);
        }
        if (gamepad1.x && !prevX) {
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_X;
            LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("Preset", "X (med) set: %.0f", LAUNCHER_TARGET_VELOCITY);
        }
        if (gamepad1.y && !prevY) {
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_Y;
            LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            telemetry.addData("Preset", "Y (high) set: %.0f", LAUNCHER_TARGET_VELOCITY);
        }

        if(gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
        }

        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

    // Left bumper handling: keep launcher spinning; start feeder once launcher reaches min velocity
        boolean bumperPressed = gamepad1.left_bumper;
        // Clear latch when bumper released so next press can trigger again
        if (!bumperPressed) {
            shotLatch = false;
        }

    // Show launcher velocity before attempting a shot
    telemetry.addData("Launcher Velocity (pre-shoot)", "%.1f", launcher.getVelocity());
    telemetry.update();

    // Pass bumper state to launch for handling (launch will set shotLatch when a shot is consumed)
    launch(bumperPressed);

        // Telemetry
        telemetry.addData("Feeding", isFeeding);
        telemetry.addData("Launcher Target/Min", "target=%.0f, min=%.0f", LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Launcher Velocity", "%.1f", launcher.getVelocity());
        telemetry.addData("Drive", "L: %.2f R: %.2f", leftPower, rightPower);
        telemetry.update();

        // Clear telemetry buffer so entries do not accumulate across loops
        telemetry.clearAll();
    }

    @Override
    public void stop() {
        // Stop launcher and feeders
        launcher.setVelocity(STOP_SPEED);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void launch(boolean bumperPressed) {
        // Always maintain launcher target velocity (SPIN_UP behavior)
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // If we're currently feeding, stop when the timer elapses
        if (isFeeding) {
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                isFeeding = false;
                // keep shotLatch true until bumper released to prevent repeated shots while held
            }
            return;
        }

        // If bumper is pressed and we haven't consumed this press (shotLatch == false)
        if (bumperPressed && !shotLatch) {
            // If launcher is at or above min velocity, start feeding immediately
            if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                isFeeding = true;
                shotLatch = true; // consume this bumper press
            } else {
                // Not at speed yet: ensure launcher is attempting to reach the target and keep waiting
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Do not set shotLatch here so that when velocity is reached while bumper still held,
                // the feeder will start in a subsequent loop iteration.
            }
        }
    }
}
