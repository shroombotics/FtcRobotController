package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

/*
Our robot works by doing the following:
- Intake: Two servos spin to intake balls into the indexer
- Indexer: The indexer servos are a conveyor belt that moves balls into the release
- Release: The release is two servos that hold the ball until it is ready to fire
- Shooter: Shoots the ball from the release
- arcadeDrive: Controls movement of the wheels, allows straffing.
               The balls are loaded from the side.
*/

/*
- Fix kickstand; stuck on wheel
- Check analog code for kickstand
- Mode switch (between kickstand and drive)
- Launch code
- Intake control
- Drive code (forward is back and back is forward)
- Emergency code
 */

@TeleOp(name = "_BotV2Teleop.java")
public class _BotV2Teleop extends OpMode {

    // Setup drive variables
    private DcMotor driveFrontRight;
    private DcMotor driveBackRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackLeft;

    double motor_speed;
    double motor_speed_default;
    double y;
    double x;
    double rx;
    double denominator;

    // Setup intake variables
    private CRServo intakeLeft;
    private CRServo intakeRight;

    boolean intakesRunning;
    final double INTAKE_STOP_SPEED = 0.0;
    final double INTAKE_FULL_SPEED = 1.0;

    // Setup kickstand
    private Servo kickstandFront;
    private Servo kickstandBack;
    private boolean kickstandDeployed;
    private boolean kickstandButtonsWerePressed = false;

    // kickstand back
    final double KICKSTAND_BACK_REST = 0.585;
    final double KICKSTAND_BACK_DEPLOY = 0.5;

    // kickstand front
    final double KICKSTAND_FRONT_REST = 0.49;
    final double KICKSTAND_FRONT_DEPLOY = 0.575;

    double kickstandRightY;

    // Indexer
    private CRServo indexer;
    final double INDEXER_STOP_SPEED = 0.0;
    final double INDEXER_FULL_SPEED = 1.0;
    private double indexerStartTime = 0;
    private boolean indexerRunning = false;

    // Release
    private CRServo releaseRight;
    private CRServo releaseLeft;
    final double RELEASE_STOP_SPEED = 0.0;
    final double RELEASE_FULL_SPEED = 1.0;

    // Shooter
    private DcMotor shooter;
    final double SHOOTER_STOP_SPEED = 0.0;
    final double SHOOTER_FULL_SPEED = 1.0;
    
    private double shooterStartTime = 0;
    private boolean shooterSpinningUp = false;
    private boolean shooterFiring = false;

    @Override
    public void init() {
        driveFrontRight = hardwareMap.get(DcMotor.class, "DFR");
        driveBackRight = hardwareMap.get(DcMotor.class, "DBR");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "DFL");
        driveBackLeft = hardwareMap.get(DcMotor.class, "DBL");
        intakeLeft = hardwareMap.get(CRServo.class, "LF");
        intakeRight = hardwareMap.get(CRServo.class, "RF");

        motor_speed_default = 1;
        motor_speed = motor_speed_default;
        intakesRunning = false;

        // Reverse the right side motors
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set up intake directions
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);

        // Get kickstands
        kickstandFront = hardwareMap.get(Servo.class, "KF");
        kickstandBack = hardwareMap.get(Servo.class, "KB");

        // kickstand rest init
        kickstandFront.setPosition(KICKSTAND_FRONT_REST);
        kickstandBack.setPosition(KICKSTAND_BACK_REST);

        // Setup shooter
        shooter = hardwareMap.get(DcMotor.class, "SHTR");
        shooter.setDirection(DcMotor.Direction.FORWARD);

        // Setup release
        releaseRight = hardwareMap.get(CRServo.class, "RR");
        releaseLeft = hardwareMap.get(CRServo.class, "RL");

        releaseLeft.setDirection(DcMotor.Direction.FORWARD);
        releaseRight.setDirection(DcMotor.Direction.REVERSE);

        // Setup indexer
        indexer = hardwareMap.get(CRServo.class, "IND");
        indexer.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized TeleOp");
        telemetry.setAutoClear(false);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void stop() {
        // Stop all spinning devices
        intakeLeft.setPower(INTAKE_STOP_SPEED);
        intakeRight.setPower(INTAKE_STOP_SPEED);

        releaseRight.setPower(RELEASE_STOP_SPEED);
        releaseLeft.setPower(RELEASE_STOP_SPEED);

        indexer.setPower(INDEXER_STOP_SPEED);

        shooter.setPower(SHOOTER_STOP_SPEED);
    }

    @Override
    public void loop() {
        // Clear telemetry each time
        telemetry.clearAll();

        arcadeDrive();

        intake();

        load();

        shoot();

        kickstand();
    }
    
    public void load() {
        /*
            Run the indexer to load a ball into the release.
            Stop after 1s.
        */
       
        // Start the indexer when button is pressed and it's not already running
        if (gamepad1.left_bumper && !indexerRunning) {
            indexerRunning = true;
            indexerStartTime = getRuntime();
            indexer.setPower(INDEXER_FULL_SPEED);
        }

        // Stop the indexer after 1 second has elapsed
        if (indexerRunning && (getRuntime() - indexerStartTime) >= 1) {
            indexerRunning = false;
            indexer.setPower(INDEXER_STOP_SPEED);
        }
    }

    public void intake() {
        /*
            Run both intakes while the driver holds the left bumper
        */
        if (gamepad1.a) {
            intakesRunning = !intakesRunning;
        }

        if (intakesRunning) {
            intakeLeft.setPower(INTAKE_FULL_SPEED);
            intakeRight.setPower(INTAKE_FULL_SPEED);
        } else {
            intakeLeft.setPower(INTAKE_STOP_SPEED);
            intakeRight.setPower(INTAKE_STOP_SPEED);
        }
    }

    public void shoot() {
        /*
            Spin up the shooter to max speed. This takes 1s
            Set the releaseRight and releaseLeft to max speed to
            feed the ball to the shooter.
        */
        // Start shooter spin-up when right bumper is pressed
        if (gamepad1.right_bumper && !shooterSpinningUp && !shooterFiring) {
            shooterSpinningUp = true;
            shooterStartTime = getRuntime();
            shooter.setPower(SHOOTER_FULL_SPEED);
        }

        // After 1 second, start feeding the ball through release servos
        if (shooterSpinningUp && (getRuntime() - shooterStartTime) >= 1) {
            shooterSpinningUp = false;
            shooterFiring = true;
            releaseRight.setPower(RELEASE_FULL_SPEED);
            releaseLeft.setPower(RELEASE_FULL_SPEED);
        }

        // Stop firing when button is released
        if (!gamepad1.right_bumper && shooterFiring) {
            shooterFiring = false;
            shooter.setPower(SHOOTER_STOP_SPEED);
            releaseRight.setPower(RELEASE_STOP_SPEED);
            releaseLeft.setPower(RELEASE_STOP_SPEED);
        }
    }

    public void kickstand() {
        // Hitting the leftmost and rightmost button toggles the kickstand
        boolean kickstandButtonsPressed = gamepad1.dpad_left && gamepad1.b;

        // Only toggle on button press (rising edge), not while held
        if (kickstandButtonsPressed && !kickstandButtonsWerePressed) {
            if (kickstandDeployed) {
                kickstandFront.setPosition(KICKSTAND_FRONT_REST);
                kickstandBack.setPosition(KICKSTAND_BACK_REST);
            } else {
                kickstandBack.setPosition(KICKSTAND_BACK_DEPLOY);
                kickstandFront.setPosition(KICKSTAND_FRONT_DEPLOY);
            }

            kickstandDeployed = !kickstandDeployed;
        }

        // Update button state for next loop
        kickstandButtonsWerePressed = kickstandButtonsPressed;
    }

    /*
    public void kickstandAnalog() {
        double kickstandBackPos;
        double kickstandFrontPos;
        double kickstandBackPosNormal;
        double kickstandFrontPosNormal;
        double normalized;

        kickstandRightY = gamepad1.right_stick_y;
        normalized = (kickstandRightY + 1.0) / 2;

        kickstandBackPosNormal = KICKSTAND_BACK_REST * normalized * (KICKSTAND_BACK_DEPLOY - KICKSTAND_BACK_REST);
        kickstandFrontPosNormal = KICKSTAND_FRONT_REST * normalized * (KICKSTAND_FRONT_DEPLOY - KICKSTAND_FRONT_REST);

        kickstandBackPos = Range.clip(kickstandBackPosNormal, KICKSTAND_BACK_REST, KICKSTAND_BACK_DEPLOY);
        kickstandFrontPos = Range.clip(kickstandFrontPosNormal, KICKSTAND_FRONT_REST, KICKSTAND_FRONT_DEPLOY);

        kickstandBack.setPosition(kickstandBackPos);
        kickstandFront.setPosition(kickstandFrontPos);

        telemetry.addData("Kickstand Y", kickstandRightY);
        telemetry.addData("KickstandBackPos", kickstandBackPos);
        telemetry.addData("KickstandFrontPos", kickstandFrontPos);
    }
    */

    public void arcadeDrive() {
        // Allow the bot to be slowed down
        if (gamepad1.right_trigger > 0.1) {
            motor_speed = motor_speed_default / 4;
        } else {
            motor_speed = motor_speed_default;
        }
    
        // Remember, this is reversed!
        y = -gamepad1.left_stick_y * motor_speed;
        x = gamepad1.left_stick_x * motor_speed;

        // Counteract imperfect strafing
        rx = gamepad1.right_stick_x * 0.75 * motor_speed;

        // Denominator is the largest motor power
        // (absolute value) or 1.
        // This ensures all the powers maintain
        // the same ratio, but only when at least one is
        // out of the range [-1, 1].
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

        // Make sure your ID's match your configuration
        driveFrontLeft.setPower((y + x + rx) / denominator);
        driveBackLeft.setPower(((y - x) + rx) / denominator);
        driveFrontRight.setPower(((y - x) - rx) / denominator);
        driveBackRight.setPower(((y + x) - rx) / denominator);
    }
}
