/*
 * TeleOp with simplified launcher control
 * - gamepad1.a / gamepad1.x / gamepad1.y spin up launcher and fire ALL loaded balls (1-3)
 * - gamepad1.b stops launcher and feeder immediately (emergency stop)
 * - ONE button press fires all balls, then launcher stops automatically
 * - Robot holds 1-3 balls - all will be fired in rapid succession
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

@TeleOp(name = "StarterBotTeleOpMB", group = "StarterBotMB")
public class StarterBotTeleOpMB extends OpMode {

    // Feeder timing - feeds ALL loaded balls (1-3) in rapid succession
    final double FEED_TIME_SECONDS = 0.75;  // Enough time for 3 balls (~0.25s each)
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    // Drive tuning parameters
    final double DRIVE_SPEED_MULTIPLIER = 0.7;  // Reduce max speed to 70%
    final double JOYSTICK_DEAD_ZONE = 0.1;      // Ignore joystick values below 10%

    // Launcher velocity presets (A=low, X=medium, Y=high)
    final double LAUNCHER_SPEED_A = 1175; // default
    final double LAUNCHER_SPEED_X = 1260;
    final double LAUNCHER_SPEED_Y = 1360;
    final double LAUNCHER_VELOCITY_BUFFER = 50;  // Velocity must be within this range to fire

    // Working target velocity
    double LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_A;

    // Hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Feeding timer
    ElapsedTime feederTimer = new ElapsedTime();

    // State
    private boolean prevB = false;
    private boolean prevA = false, prevX = false, prevY = false;

    // Auto-shooting state
    private boolean shotRequested = false;  // true when A/X/Y pressed, waiting for velocity
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
        // Launcher starts stopped. Press A/X/Y to spin up to desired speed.
        LAUNCHER_TARGET_VELOCITY = STOP_SPEED;
        launcher.setVelocity(STOP_SPEED);
    }

    @Override
    public void loop() {
        // Clear telemetry each loop
        telemetry.clearAll();

        // Drive inputs (arcade)
        arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

        // A/X/Y buttons: Select launcher speed and fire ALL balls
        if (gamepad1.a && !prevA) {
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_A;
            shotRequested = true;
        }
        if (gamepad1.x && !prevX) {
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_X;
            shotRequested = true;
        }
        if (gamepad1.y && !prevY) {
            LAUNCHER_TARGET_VELOCITY = LAUNCHER_SPEED_Y;
            shotRequested = true;
        }
        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevY = gamepad1.y;

        // B button: stop launcher and feeder
        boolean bPressed = gamepad1.b;
        if (bPressed && !prevB) {
            LAUNCHER_TARGET_VELOCITY = STOP_SPEED;
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            isFeeding = false;
            shotRequested = false;
        }
        prevB = bPressed;

        // Continuously command launcher to target velocity
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        // Auto-fire logic: Fire all loaded balls when launcher reaches speed
        if (isFeeding) {
            if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                isFeeding = false;
                shotRequested = false;
                LAUNCHER_TARGET_VELOCITY = STOP_SPEED;  // Stop launcher after shot
            }
        } else if (shotRequested && launcher.getVelocity() >= LAUNCHER_TARGET_VELOCITY - LAUNCHER_VELOCITY_BUFFER) {
            leftFeeder.setPower(FULL_SPEED);
            rightFeeder.setPower(FULL_SPEED);
            feederTimer.reset();
            isFeeding = true;
        }

        // Telemetry - show driver useful information
        telemetry.addData("Velocity", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Status", shotRequested ? (isFeeding ? "FIRING!" : "Spinning up...") : "Ready");
        telemetry.addData("Drive", "L: %.2f R: %.2f", leftPower, rightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setVelocity(STOP_SPEED);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    // Experimental drive code. Meant to provide smoother driving.
    void arcadeDrive(double forward, double rotate) {
        // Apply dead zone to prevent drift from joystick noise
        forward = applyDeadZone(forward);
        rotate = applyDeadZone(rotate);

        // Apply speed multiplier to reduce max speed for smoother control
        forward *= DRIVE_SPEED_MULTIPLIER;
        rotate *= DRIVE_SPEED_MULTIPLIER;

        // Calculate raw motor powers
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        // Normalize powers if any exceed ±1.0 to maintain turn ratio
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Set motor powers
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    double applyDeadZone(double value) {
        // Ignore small joystick movements to prevent drift
        if (Math.abs(value) < JOYSTICK_DEAD_ZONE) {
            return 0.0;
        }
        // Scale remaining range to maintain smooth response
        return (value - Math.signum(value) * JOYSTICK_DEAD_ZONE) / (1.0 - JOYSTICK_DEAD_ZONE);
    }
}
