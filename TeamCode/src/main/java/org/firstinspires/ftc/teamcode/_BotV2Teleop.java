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
- Loader: The loader servos are a conveyor belt that moves balls into the release
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

    // kickstand back
    final double KICKSTAND_BACK_REST = 0.6;
    final double KICKSTAND_BACK_DEPLOY = 0.5;

    // kickstand front
    final double KICKSTAND_FRONT_REST = 0.475;
    final double KICKSTAND_FRONT_DEPLOY = 0.575;

    double kickstandRightY;


    // Indexer
    private CRServo indexer;

    //Release
    private CRServo releaseRight;
    private CRServo releaseLeft;

    //Launcher
    private DcMotor shooter;
    final double SHOOTER_STOP_SPEED = 0.0;
    final double SHOOTER_FULL_SPEED = 1.0;
    double SHOOTER_CURRENT_SPEED;

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

        shooter = hardwareMap.get(DcMotor.class, "SHTR");
        indexer = hardwareMap.get(CRServo.class, "IND");
        releaseRight = hardwareMap.get(CRServo.class, "RR");
        releaseLeft = hardwareMap.get(CRServo.class, "RL");

        shooter.setDirection(DcMotor.Direction.FORWARD);

        releaseLeft.setDirection(DcMotor.Direction.FORWARD);
        releaseRight.setDirection(DcMotor.Direction.REVERSE);

        indexer.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized TeleOp");
        telemetry.setAutoClear(false);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        intakeLeft.setPower(INTAKE_STOP_SPEED);
        intakeRight.setPower(INTAKE_STOP_SPEED);
    }

    @Override
    public void loop() {
        // Clear telemetry each time
        telemetry.clearAll();

        if (gamepad1.left_bumper) {
            motor_speed = motor_speed_default / 4;
        } else {
            motor_speed = motor_speed_default;
        }

        // Handle drive code
        arcadeDrive();

        indexer.setPower(1);
        shooter.setPower(1);
        releaseLeft.setPower(.5);
        releaseRight.setPower(.5);


        // Shot control tolerence
        if (gamepad1.right_trigger > 0.1) {
            intakesRunning = !intakesRunning;
        }

        if (intakesRunning) {
            intakeLeft.setPower(INTAKE_FULL_SPEED);
            intakeRight.setPower(INTAKE_FULL_SPEED);
        } else {
            intakeLeft.setPower(INTAKE_STOP_SPEED);
            intakeRight.setPower(INTAKE_STOP_SPEED);
        }

        if (gamepad1.b && gamepad1.left_bumper) {
            kickstand();
        }

        if (gamepad1.a && gamepad1.left_bumper) {
            kickstandFront.setPosition(KICKSTAND_FRONT_REST);
            kickstandBack.setPosition(KICKSTAND_BACK_REST);
        }

    }

    public void load() {

    }

    public void shoot() {

    }

    public void kickstand() {
        kickstandBack.setPosition(KICKSTAND_BACK_DEPLOY);
        kickstandFront.setPosition(KICKSTAND_FRONT_DEPLOY);
    }

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

    public void arcadeDrive() {
        // Remember, this is reversed!
        y = gamepad1.left_stick_y * motor_speed;
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

        /*
        driveFrontLeft.setPower((y - x - rx) / denominator);
        driveBackLeft.setPower(((y + x) - rx) / denominator);
        driveFrontRight.setPower(((y + x) + rx) / denominator);
        driveBackRight.setPower(((y - x) + rx) / denominator);
        */
    }

}
