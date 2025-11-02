/*
 * TeleOp derived from StarterBotTeleopHS.java
 * - Right bumper (rising edge) starts the launcher at default speed. Launcher will not auto-start elsewhere.
 * - gamepad1.a / gamepad1.x / gamepad1.y change launcher speed only while launcher is active (i.e., after right bumper pressed)
 * - gamepad1.b stops launcher and feeder; launcher will only start again after right bumper is pressed
 * - left bumper (rising edge) feeds one shot only when launcher velocity >= LAUNCHER_MIN_VELOCITY
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

@TeleOp(name = "StarterBotTeleOpHSNew", group = "StarterBotHS")
public class StarterBotTeleOpHSNew extends OpMode {

    // Feeder timing
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Launcher velocity presets (A=low, X=medium, Y=high)
    final double LAUNCHER_SPEED_A = 1260;
    final double LAUNCHER_SPEED_X = 1175; // default
    final double LAUNCHER_SPEED_Y = 1360;

    // Working target/minimum values
    double LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_X;
    double LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;

    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Feeding timer
    ElapsedTime feederTimer = new ElapsedTime();

    // State
    private boolean launcherActive = false; // set true when right bumper is pressed
    private boolean prevRightBumper = false;
    private boolean prevB = false;
    private boolean prevA = false, prevX = false, prevY = false;

    // Shooting (left bumper) one-shot handling
    private boolean prevLeftBumper = false;
    private boolean shotLatch = false; // ensures one shot per press
    private boolean isFeeding = false;

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

        // Use encoder-based velocity control for the launcher
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // Initialize feeders stopped and directions consistent
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // PIDF - tune as needed
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        telemetry.addData("Status", "Initialized New TeleOp");
        telemetry.setAutoClear(false);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        // Do not start launcher automatically. It starts only when right bumper is pressed.
        launcherActive = false;
        launcher.setVelocity(STOP_SPEED);
    }

    @Override
    public void loop() {
        // Clear telemetry each loop
        telemetry.clearAll();

        // Drive inputs (arcade)
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Right bumper: rising edge starts the launcher at default speed
        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !prevRightBumper) {
            launcherActive = true;
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_X; // default on activation
            LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
        prevRightBumper = rightBumper;

        // B button: stop launcher and feeder; require right bumper to be pressed again to restart
        boolean bPressed = gamepad1.b;
        if (bPressed && !prevB) {
            // Stop everything
            launcherActive = false;
            launcher.setVelocity(STOP_SPEED);
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            isFeeding = false;
            shotLatch = false;
        }
        prevB = bPressed;

        // Only allow preset changes if launcher is active
        if (launcherActive) {
            if (gamepad1.a && !prevA) {
                LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_A;
                LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            }
            if (gamepad1.x && !prevX) {
                LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_X;
                LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            }
            if (gamepad1.y && !prevY) {
                LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_Y;
                LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY - 50;
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            }
        }
        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        // If launcher is active, keep commanding it to target; otherwise ensure it's stopped
        if (launcherActive) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(STOP_SPEED);
        }

        // Show launcher velocity before attempting a shot
        telemetry.addData("Launcher Velocity (pre-shoot)", "%.1f", launcher.getVelocity());
        telemetry.update();

        // Left bumper: shot request (one-shot per press). Only feed if launcher active and up to speed
        boolean leftBumper = gamepad1.left_bumper;
        boolean shotRequested = leftBumper && !prevLeftBumper;
        prevLeftBumper = leftBumper;

        // Feeding logic
        if (isFeeding) {
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                isFeeding = false;
            }
        } else if (shotRequested && !shotLatch) {
            // Consume the shot only if launcher is active and at/above minimum velocity
            if (launcherActive && launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                isFeeding = true;
                shotLatch = true;
            } else {
                // If not ready, ensure launcher is commanded and do not latch so it can fire once ready
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            }
        }

        // Clear shot latch when left bumper released so next press can trigger
        if (!leftBumper) {
            shotLatch = false;
        }

        // Telemetry summary
        telemetry.addData("LauncherActive", launcherActive);
        telemetry.addData("LauncherTarget/Min", "target=%.0f min=%.0f", LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Feeding", isFeeding);
        telemetry.addData("Drive", "L: %.2f R: %.2f", leftPower, rightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
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
}
