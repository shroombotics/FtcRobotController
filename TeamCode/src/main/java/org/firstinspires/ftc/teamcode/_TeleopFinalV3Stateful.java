package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * _TeleopFinalV3Stateful
 *
 * A simplified TeleOp that uses a straightforward alignment method
 * and simple helpers so the strafing logic is easy to follow.
 */
@TeleOp(name = "_TeleopFinalV3Stateful")
public class _TeleopFinalV3Stateful extends OpMode {

    // ...existing declarations copied from V2 (kept unchanged for familiarity)
    private DcMotor driveFrontRight;
    private DcMotor driveBackRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackLeft;

    private CRServo intakeLeft;
    private CRServo intakeRight;

    private Servo kickstandFront;
    private Servo kickstandBack;

    private CRServo releaseRight;
    private CRServo releaseLeft;

    private DcMotor indexer;
    private DcMotorEx shooter;

    private ServoImplEx light = null;
    private Limelight3A llcam = null;

    private static final double DEFAULT_MOTOR_SPEED = 1.0;
    private static final double SLOW_TRIGGER_THRESHOLD = 0.1;
    private static final double SLOW_MODE_DIVISOR = 4.0;
    private static final double ROTATION_SCALE = 0.75;

    private static final double INTAKE_STOP_SPEED = 0.0;
    private static final double INTAKE_FULL_SPEED = 1.0;
    private static final double INDEXER_STOP_SPEED = 0.0;
    private static final double INDEXER_FULL_SPEED = 1.0;
    private static final double RELEASE_STOP_SPEED = 0.0;
    private static final double RELEASE_FULL_SPEED = 1.0;
    private static final double FEED_TIME_SECONDS = 0.20;
    private static final double FEEDER_FULL_SPEED = 1.0;

    private static final double SHOOTER_RPM_AT_FULL_POWER = 4000.0;
    private static final double SHOOTER_VELOCITY_SAFETY_MULTIPLIER = 1.05;
    private static final double DEFAULT_SHOOTER_RPM_NO_TY = 1100.0;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;

    private static final double ALIGN_KP = 0.04;              // degrees -> power gain
    private static final double ALIGN_MAX_POWER = 0.50;      // clamp of strafe
    private static final double ALIGN_TX_THRESHOLD_DEG = 1.0; // considered "aligned"
    private static final double ALIGN_SLOW_ZONE_DEG = 6.0;   // slow region near center
    private static final double ALIGN_SLOW_MIN_POWER = 0.12; // minimum power in slow zone
    private static final double STRAFE_INVERT = 1.0;         // flip this to change hardware polarity

    private static final double TX_CACHE_TTL_SECONDS = 0.5;
    private static final double LEFT_BUMPER_WAIT_TIMEOUT_SECONDS = 8.0;
    private static final int MAX_SHOTS = 3;
    private static final double SHOOT_WINDOW_SECONDS = 5.0;
    private static final double JOYSTICK_DEADBAND = 0.05;
    private static final double INCH_TO_METER = 0.0254;
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 15.0;
    private static final double GOAL_HEIGHT_INCHES = 30.0;
    private static final double CAMERA_HEIGHT_INCHES = 14.0;
    private static final double LAUNCHER_FACTOR_BASE = 4.8;
    private static final double VELOCITY_DISTANCE_SCALE = 0.02;
    private static final double WHEEL_RADIUS_INCH = 2.0;
    private static final double TICKS_PER_REV_DEFAULT = 28.0;
    private static final double FACTOR_OFFSET_INCHES = 10.0;
    private static final double FACTOR_SCALE_INCHES = 125.0;
    private static final double ALIGN_TIMEOUT_SECONDS = 6.0;
    private static final double TX_ACQUIRE_TIMEOUT_SECONDS = 1.0;
    private static final double KICKSTAND_BACK_REST = 0.585;
    private static final double KICKSTAND_BACK_DEPLOY = 0.5;
    private static final double KICKSTAND_FRONT_REST = 0.49;
    private static final double KICKSTAND_FRONT_DEPLOY = 0.575;

    private static final boolean DEBUG_TELEMETRY = false;

    double motor_speed;
    double motor_speed_default;

    boolean intakesRunning;
    boolean indexerRunning = false;

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    // previous state for the A button (intake toggle)
    private boolean prevA = false;

    private double cachedTx = Double.NaN;
    private double cachedTy = Double.NaN;
    private long cachedTxNanoTime = 0L;
    private boolean hasLimelight = false;
    private boolean hasFeeders = false;
    private boolean kickstandDeployed = false;
    private boolean kickstandButtonsWerePressed = false;
    private String indexerMappedName = null;

    private boolean waitingForTx = false;
    private final ElapsedTime txAcquireTimer = new ElapsedTime();

    // Keep simpler alignment state (we don't use multi-step auto-tests here)
    private final ElapsedTime alignTimer = new ElapsedTime();

    private final ElapsedTime feederTimer = new ElapsedTime();
    private boolean isFeeding = false;
    private final ElapsedTime shooterSpinTimer = new ElapsedTime();
    private final ElapsedTime shootStateTimer = new ElapsedTime();

    private boolean shooterInit = false;
    private boolean shooterActive = false;
    private double shooterTargetPower = 0.0;
    private double shooterTargetVelocityTicks = 0.0;
    private double shooterMinVelocityTicks = 0.0;
    // last measured shooter velocity (ticks/sec) for telemetry
    private double lastShooterVelocityTicks = Double.NaN;

    private enum RobotState { DRIVE, ALIGN, SPIN_UP, SHOOT_READY }
    private RobotState robotState = RobotState.DRIVE;
    private int shotAttempts = 0;

    private double lastAlignTx = Double.NaN;
    private double lastAlignSp = 0.0;
    private double lastAlignTy = Double.NaN;

    @Override
    public void init() {
        driveFrontRight = hardwareMap.get(DcMotor.class, "DFR");
        driveBackRight = hardwareMap.get(DcMotor.class, "DBR");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "DFL");
        driveBackLeft = hardwareMap.get(DcMotor.class, "DBL");
        intakeLeft = hardwareMap.get(CRServo.class, "LF");
        intakeRight = hardwareMap.get(CRServo.class, "RF");

        motor_speed_default = DEFAULT_MOTOR_SPEED;
        motor_speed = motor_speed_default;

        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);

    // try to map kickstand servos if present
    try { kickstandFront = hardwareMap.get(Servo.class, "KF"); } catch (Exception ignored) { kickstandFront = null; }
    try { kickstandBack = hardwareMap.get(Servo.class, "KB"); } catch (Exception ignored) { kickstandBack = null; }
    try { if (kickstandFront != null) kickstandFront.setPosition(KICKSTAND_FRONT_REST); } catch (Exception ignored) {}
    try { if (kickstandBack != null) kickstandBack.setPosition(KICKSTAND_BACK_REST); } catch (Exception ignored) {}

    try { shooter = hardwareMap.get(DcMotorEx.class, "SHTR"); } catch (Exception ignored) { shooter = null; }
        try { light = hardwareMap.get(ServoImplEx.class, "SL"); } catch (Exception ignored) { light = null; }
        try { llcam = hardwareMap.get(Limelight3A.class, "CAM"); } catch (Exception ignored) { llcam = null; }
        try { releaseRight = hardwareMap.get(CRServo.class, "RR"); } catch (Exception ignored) { releaseRight = null; }
        try { releaseLeft = hardwareMap.get(CRServo.class, "RL"); } catch (Exception ignored) { releaseLeft = null; }

    // map indexer motor (single expected name: "IND")
    indexer = null; indexerMappedName = null;
    try {
        indexer = hardwareMap.get(DcMotor.class, "IND");
        if (indexer != null) {
            try { indexer.setDirection(DcMotor.Direction.REVERSE); } catch (Exception ignored) {}
            try { indexer.setPower(0.0); } catch (Exception ignored) {}
            indexerMappedName = "IND";
        }
    } catch (Exception ignored) { indexer = null; indexerMappedName = null; }

        if (llcam != null) { try { llcam.pipelineSwitch(0); llcam.start(); hasLimelight = true; } catch (Exception ignored) {} }
        if (releaseLeft != null || releaseRight != null) {
            hasFeeders = true;
            try { if (releaseLeft != null) releaseLeft.setPower(0.0); } catch (Exception ignored) {}
            try { if (releaseRight != null) releaseRight.setPower(0.0); } catch (Exception ignored) {}
        }

    telemetry.addData("Status", "Initialized TeleOp Simplified V3");
    // Use autoClear so we don't accumulate lines and cause scrolling on the driver station.
    telemetry.setAutoClear(true);
    }

    @Override
    public void loop() {
        pollLimelightCache();

        if (waitingForTx) {
            double tx = cachedTx; double ty = cachedTy;
            if (!Double.isNaN(tx)) {
                waitingForTx = false;
                lastAlignTy = ty; alignTimer.reset(); lastAlignTx = tx; robotState = RobotState.ALIGN;
            } else if (txAcquireTimer.seconds() > TX_ACQUIRE_TIMEOUT_SECONDS) {
                waitingForTx = false; showOneShotTelemetry("Limelight", "no tx - aim at tag and press again"); prevLeftBumper = false;
            } else {
                telemetry.addData("Limelight", "acquiring tx...");
            }
        }

    // Process toggles and quick-stop first so buttons work in any state (matches V1 behavior)
    processIntakeAndLoaderButtons();
    handleQuickStopOnX();
    kickstand();

    // Ensure indexer power is always applied to hardware so Y-toggle works in any state
    try { if (indexer != null) setIndexerPower(indexerRunning ? INDEXER_FULL_SPEED : INDEXER_STOP_SPEED); } catch (Exception ignored) {}

    if (detectJoystickMovement() && robotState != RobotState.DRIVE) cancelShootingAndReturnDrive("joystick moved");

    // indexer telemetry is shown in the concise summary below

    switch (robotState) {
            case DRIVE:
                runDriveControls();
                handleLeftBumperStart();
                // allow quick-stop and toggleable controls while driving
                handleQuickStopOnX();
                break;
            case ALIGN:
                performAlignmentStepSimple();
                break;
            case SPIN_UP:
                monitorShooterAndSetLed();
                break;
            case SHOOT_READY:
                break;
        }



        handleShootingWithRightBumperSimplified();
        updateSummaryTelemetry();
    }

    // -- simplified alignment method --
    private void performAlignmentStepSimple() {
        // ensure camera is present
        if (llcam == null) { 
            robotState = RobotState.DRIVE; 
            // entering DRIVE: reset shot attempts so bumpers can be used again
            shotAttempts = 0;
            return; 
        }

        // read tx (prefers cached sample)
        double tx = Double.NaN;
        if (isCachedTxFresh()) tx = cachedTx; else { try { tx = llcam.getLatestResult().getTx(); } catch (Exception ignored) {} }
        if (Double.isNaN(tx)) { telemetry.addData("Align", "no tx yet - aim and retry"); return; }

        // if aligned, stop and prepare shooter
        if (Math.abs(tx) <= ALIGN_TX_THRESHOLD_DEG) {
            setStrafePower(0.0);
            double usedTy = !Double.isNaN(cachedTy) ? cachedTy : Double.NaN;
            computeShooterFromTy(usedTy);
            enterSpinUp();
            return;
        }

        // proportional control (simple)
        double raw = ALIGN_KP * tx; // signed
        double sp = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, raw));

        // ensure minimum gentle motion in slow zone
        double absTx = Math.abs(tx);
        if (absTx <= ALIGN_SLOW_ZONE_DEG && absTx > ALIGN_TX_THRESHOLD_DEG) {
            double min = ALIGN_SLOW_MIN_POWER * Math.signum(sp != 0.0 ? sp : tx);
            if (Math.abs(sp) < Math.abs(min)) sp = min;
        }

        sp = sp * STRAFE_INVERT; // apply hardware direction

        lastAlignTx = tx; lastAlignSp = sp;
        setStrafePower(sp);
    }

    // -- the rest of the helpers are copied from V2 for parity --
    private void handleShootingWithRightBumperSimplified() {
        boolean rightBumper = gamepad1.right_bumper;
        boolean rightBumperEdge = rightBumper && !prevRightBumper;
        if (rightBumperEdge) {
            if (robotState == RobotState.SHOOT_READY) {
                if (shooterActive) {
                    if (!isFeeding) {
                        if (releaseLeft != null && releaseRight != null) {
                            setFeederPower(FEEDER_FULL_SPEED);
                            feederTimer.reset(); isFeeding = true; shotAttempts++; if (shotAttempts == 1) shootStateTimer.reset();
                            showOneShotTelemetry("Fired", String.format("attempt=%d", shotAttempts));
                        } else showOneShotTelemetry("Feeders", "not present");
                    }
                } else { showOneShotTelemetry("Shot", "shooter not ready"); }
            } else { showOneShotTelemetry("Shot", "not in shoot ready"); }
        }
        if (isFeeding && feederTimer.seconds() > FEED_TIME_SECONDS) {
            setFeederPower(0.0);
            isFeeding = false;
        }
        if (shotAttempts >= MAX_SHOTS && !isFeeding) cancelShootingAndReturnDrive("max shots reached");
        prevRightBumper = rightBumper;
    }

    // Note: helper methods below perform safe hardware calls with their own try/catch blocks.

    // Direct helpers that avoid repeated small lambdas and are easier to read.
    private void setDrivePowersDirect(double pFL, double pBL, double pFR, double pBR) {
        try { driveFrontLeft.setPower(pFL); } catch (Exception ignored) {}
        try { driveBackLeft.setPower(pBL); } catch (Exception ignored) {}
        try { driveFrontRight.setPower(pFR); } catch (Exception ignored) {}
        try { driveBackRight.setPower(pBR); } catch (Exception ignored) {}
    }

    private void setStrafePower(double sp) {
        // Use the direct helper so the strafing sign logic is in one place and easier to read.
        setDrivePowersDirect(sp, -sp, -sp, sp);
    }

    private void setIntakePower(double leftPower, double rightPower) {
        try { if (intakeLeft != null) intakeLeft.setPower(leftPower); } catch (Exception ignored) {}
        try { if (intakeRight != null) intakeRight.setPower(rightPower); } catch (Exception ignored) {}
    }

    private void setFeederPower(double power) {
        try { if (releaseLeft != null) releaseLeft.setPower(power); } catch (Exception ignored) {}
        try { if (releaseRight != null) releaseRight.setPower(power); } catch (Exception ignored) {}
    }

    private void setIndexerPower(double power) {
        try { if (indexer != null) indexer.setPower(power); } catch (Exception ignored) {}
    }

    private void setShooterPower(double power) {
        try { if (shooter != null) shooter.setPower(power); } catch (Exception ignored) {}
    }

    private void runDriveControls() {
        // Simple arcade-style drive: forward/back (y), strafe (x), rotate (rx)
        double y = -gamepad1.left_stick_y * motor_speed;
        double x = gamepad1.left_stick_x * motor_speed;
        double rx = gamepad1.right_stick_x * ROTATION_SCALE * motor_speed;

        // denominator prevents scaled values > 1. Use readable logic: max(1, |y|+|x|+|rx|)
        double sum = Math.abs(y) + Math.abs(x) + Math.abs(rx);
        double denominator = Math.max(1.0, sum);

        double pFL = (y + x + rx) / denominator;
        double pBL = ((y - x) + rx) / denominator;
        double pFR = ((y - x) - rx) / denominator;
        double pBR = ((y + x) - rx) / denominator;

        setDrivePowersDirect(pFL, pBL, pFR, pBR);
    }

    private boolean detectJoystickMovement() {
        double lx = Math.abs(gamepad1.left_stick_x);
        double ry = Math.abs(gamepad1.right_stick_y);
        if (lx > JOYSTICK_DEADBAND) return true;
        if (ry > JOYSTICK_DEADBAND) return true;
        return false;
    }

    private void processIntakeAndLoaderButtons() {
        // Intake toggle on A: press once to start, press again to stop
        boolean intakeButton = gamepad1.a;
        if (intakeButton && !prevA) {
            intakesRunning = !intakesRunning; // toggle state
        }
        prevA = intakeButton;

        if (intakesRunning) {
            setIntakePower(INTAKE_FULL_SPEED, INTAKE_FULL_SPEED);
        } else {
            setIntakePower(INTAKE_STOP_SPEED, INTAKE_STOP_SPEED);
        }

        // indexer toggle on Y (simple)
        boolean indexerButton = gamepad1.y;
        if (indexerButton && !prevY) {
            indexerRunning = !indexerRunning;
        }
        prevY = indexerButton;
        if (indexer != null) setIndexerPower(indexerRunning ? INDEXER_FULL_SPEED : INDEXER_STOP_SPEED);
    }

    // Quick-stop on X: stop drive, intake, indexer, shooter, feeders immediately and return to DRIVE
    private void handleQuickStopOnX() {
        boolean xPressed = gamepad1.x;
        if (xPressed && !prevX) {
            // stop everything safely
            setFeederPower(0.0);
            isFeeding = false;
            feederTimer.reset();
            shooterInit = false;
            shooterActive = false;
            setShooterPower(0.0);
            setDrivePowersDirect(0.0, 0.0, 0.0, 0.0);
            setIntakePower(0.0, 0.0);
            try { if (indexer != null) indexer.setPower(0.0); } catch (Exception ignored) {}
            indexerRunning = false;
            // reset attempts so bumpers work again
            shotAttempts = 0;
            // clear any waiting flags
            waitingForTx = false;
            showOneShotTelemetry("QuickStop", "Emergency stop (X)");
            robotState = RobotState.DRIVE;
        }
        prevX = xPressed;
    }

    // Kickstand deploy: press dpad_left + B together to toggle
    private void kickstand() {
        boolean kickstandButtonsPressed = gamepad1.dpad_left && gamepad1.b;
        if (kickstandButtonsPressed && !kickstandButtonsWerePressed) {
            if (kickstandDeployed) {
                try { if (kickstandFront != null) kickstandFront.setPosition(KICKSTAND_FRONT_REST); } catch (Exception ignored) {}
                try { if (kickstandBack != null) kickstandBack.setPosition(KICKSTAND_BACK_REST); } catch (Exception ignored) {}
            } else {
                try { if (kickstandBack != null) kickstandBack.setPosition(KICKSTAND_BACK_DEPLOY); } catch (Exception ignored) {}
                try { if (kickstandFront != null) kickstandFront.setPosition(KICKSTAND_FRONT_DEPLOY); } catch (Exception ignored) {}
            }
            kickstandDeployed = !kickstandDeployed;
        }
        kickstandButtonsWerePressed = kickstandButtonsPressed;
    }

    private void handleLeftBumperStart() {
        boolean leftBumper = gamepad1.left_bumper; boolean leftBumperEdge = leftBumper && !prevLeftBumper; if (!leftBumperEdge) { prevLeftBumper = leftBumper; return; }
        if (robotState != RobotState.DRIVE) { prevLeftBumper = leftBumper; return; }
        if (hasLimelight && llcam != null) {
            if (isCachedTxFresh()) { lastAlignTx = cachedTx; lastAlignTy = cachedTy; robotState = RobotState.ALIGN; alignTimer.reset(); prevLeftBumper = leftBumper; return; }
            try { LLResult r = llcam.getLatestResult(); double tx = r.getTx(); double ty = r.getTy(); if (!Double.isNaN(tx)) { lastAlignTx = tx; lastAlignTy = ty; robotState = RobotState.ALIGN; alignTimer.reset(); prevLeftBumper = leftBumper; return; } } catch (Exception ignored) {}
            waitingForTx = true; txAcquireTimer.reset(); showOneShotTelemetry("Limelight", "acquiring tx..."); prevLeftBumper = leftBumper; robotState = RobotState.ALIGN; return;
        }
        computeShooterFromTy(Double.NaN); enterSpinUp(); prevLeftBumper = leftBumper;
    }

    private void enterSpinUp() { robotState = RobotState.SPIN_UP; shootStateTimer.reset(); }

    private void cancelShootingAndReturnDrive(String reason) {
        // stop feeders safely
        setFeederPower(0.0);
        shooterInit = false;
        shooterActive = false;
        setShooterPower(0.0);
        robotState = RobotState.DRIVE;
        // Reset shot attempts when returning to DRIVE so bumpers are usable again.
        shotAttempts = 0;
        showOneShotTelemetry("Abort", reason);
    }

    private void pollLimelightCache() {
        if (llcam == null) return;
        try {
            LLResult r = llcam.getLatestResult();
            double tx = r.getTx();
            double ty = r.getTy();
            if (!Double.isNaN(tx)) {
                cachedTx = tx;
                cachedTy = ty;
                cachedTxNanoTime = System.nanoTime();
            }
        } catch (Exception ignored) {}
    }

    private boolean isCachedTxFresh() {
        if (cachedTxNanoTime == 0L) return false;
        double ageSec = (System.nanoTime() - cachedTxNanoTime) / 1e9;
        return ageSec <= TX_CACHE_TTL_SECONDS;
    }

    private void computeShooterFromTy(double ty) {
        double targetRPM;
        if (!Double.isNaN(ty)) {
            double distanceMeters = estimateDistanceFromTy(ty, LIMELIGHT_MOUNT_ANGLE_DEG, GOAL_HEIGHT_INCHES * INCH_TO_METER, CAMERA_HEIGHT_INCHES * INCH_TO_METER);
            double distanceInches = distanceMeters / INCH_TO_METER;
            double factor = calculatedFactor(LAUNCHER_FACTOR_BASE, distanceInches);
            double requiredVelocity = factor + (VELOCITY_DISTANCE_SCALE * distanceInches);
            double newRPM = (requiredVelocity / (2.0 * Math.PI * WHEEL_RADIUS_INCH)) * 60.0;
            targetRPM = newRPM * SHOOTER_VELOCITY_SAFETY_MULTIPLIER;
            if (DEBUG_TELEMETRY) showOneShotTelemetry("Calc", String.format("ty=%.2f dist_in=%.1f rpm=%.1f", ty, distanceInches, targetRPM));
        } else {
            targetRPM = DEFAULT_SHOOTER_RPM_NO_TY;
            if (DEBUG_TELEMETRY) showOneShotTelemetry("Calc", String.format("ty=NaN default_rpm=%.1f", targetRPM));
        }
        double ticksPerRev = TICKS_PER_REV_DEFAULT; if (shooter != null) try { ticksPerRev = shooter.getMotorType().getTicksPerRev(); } catch (Exception ignored) {}
        shooterTargetVelocityTicks = (targetRPM / 60.0) * ticksPerRev;
        shooterTargetPower = Math.max(0.0, Math.min(1.0, (targetRPM) / SHOOTER_RPM_AT_FULL_POWER));
        double rpmDelta = 0.05 * SHOOTER_RPM_AT_FULL_POWER; double deltaTicks = (rpmDelta / 60.0) * ticksPerRev; double shooterMinVelocityTicksLocal = Math.max(0.0, shooterTargetVelocityTicks - deltaTicks);
        shooterInit = true; shooterSpinTimer.reset(); try { if (shooter != null) shooter.setVelocity(shooterTargetVelocityTicks); } catch (Exception ignored) {} shooterMinVelocityTicks = shooterMinVelocityTicksLocal;
    }

    private void monitorShooterAndSetLed() {
        double rawTicks = 0.0; double ticksPerRev = 28.0;
        if (shooter != null) { try { rawTicks = shooter.getVelocity(); } catch (Exception ignored) {} try { ticksPerRev = shooter.getMotorType().getTicksPerRev(); } catch (Exception ignored) {} }
        // remember last measured velocity for telemetry
        lastShooterVelocityTicks = rawTicks;
        boolean atOrAbove = shooterInit && rawTicks >= shooterMinVelocityTicks;
        boolean sustained = shooterSpinTimer.seconds() >= SHOOTER_SPINUP_SECONDS;
        if (atOrAbove && sustained) { shooterActive = true; if (robotState == RobotState.SPIN_UP) robotState = RobotState.SHOOT_READY; } else shooterActive = false;
        try { if (shooterActive) { if (light != null) light.setPosition(0.29); } else if (shooterInit) { if (light != null) light.setPosition(0.64); } else { if (light != null) light.setPosition(0.52); } } catch (Exception ignored) {}
    }

    private void updateSummaryTelemetry() {
        // Keep a short, focused summary so the driver sees only useful info.
        telemetry.clear();
        telemetry.addData("State", robotState.toString());
        telemetry.addData("Intake", intakesRunning ? "ON" : "OFF");
        telemetry.addData("Indexer", indexer != null ? (indexerRunning ? "ON" : "OFF") : "MISSING");
        telemetry.addData("HasLimelight", hasLimelight);
        if (hasLimelight) telemetry.addData("LL tx/ty", (!Double.isNaN(cachedTx) ? String.format("%.2f" , cachedTx) : "n/a") + " / " + (!Double.isNaN(cachedTy) ? String.format("%.2f", cachedTy) : "n/a"));

        switch (robotState) {
            case DRIVE:
                telemetry.addData("Shooter", shooterInit ? (shooterActive ? "READY" : "SPINUP") : "IDLE");
                // show current drive motor powers (helpful to diagnose stuck motors)
                try {
                    double pfl = driveFrontLeft.getPower(); double pbl = driveBackLeft.getPower(); double pfr = driveFrontRight.getPower(); double pbr = driveBackRight.getPower();
                    telemetry.addData("pFL/pBL/pFR/pBR", String.format("%.2f %.2f %.2f %.2f", pfl, pbl, pfr, pbr));
                } catch (Exception ignored) {}
                telemetry.addData("ShotAttempts", shotAttempts);
                break;
            case ALIGN:
                telemetry.addData("ALIGN tx", !Double.isNaN(lastAlignTx) ? String.format("%.2f", lastAlignTx) : "n/a");
                telemetry.addData("ALIGN ty", !Double.isNaN(lastAlignTy) ? String.format("%.2f", lastAlignTy) : "n/a");
                telemetry.addData("AlignCmd", String.format("%.2f", lastAlignSp));
                telemetry.addData("AlignTime_s", String.format("%.2f", alignTimer.seconds()));
                break;
            case SPIN_UP:
                telemetry.addData("TargetPwr", String.format("%.2f", shooterTargetPower));
                telemetry.addData("TargetVel(ticks/s)", String.format("%.0f", shooterTargetVelocityTicks));
                telemetry.addData("ActVel(ticks/s)", !Double.isNaN(lastShooterVelocityTicks) ? String.format("%.0f", lastShooterVelocityTicks) : "n/a");
                telemetry.addData("SpinTimer_s", String.format("%.2f", shooterSpinTimer.seconds()));
                telemetry.addData("MinVelReq", String.format("%.0f", shooterMinVelocityTicks));
                break;
            case SHOOT_READY:
                telemetry.addData("ShooterReady", shooterActive);
                telemetry.addData("Attempts", shotAttempts);
                telemetry.addData("ActVel(ticks/s)", !Double.isNaN(lastShooterVelocityTicks) ? String.format("%.0f", lastShooterVelocityTicks) : "n/a");
                telemetry.addData("SpinTimer_s", String.format("%.2f", shooterSpinTimer.seconds()));
                telemetry.addData("FeedActive", isFeeding ? "YES" : "NO");
                telemetry.addData("FeedTimer_s", String.format("%.2f", feederTimer.seconds()));
                break;
        }
        telemetry.update();
    }

    private int shotCounter = 0;
    private double shotTimer = 0.0;

    private void showOneShotTelemetry(String title, String... lines) {
        shotCounter++;
        telemetry.clearAll();
        telemetry.addData("Event", String.format("%s #%d", title, shotCounter));
        for (String l : lines) {
            telemetry.addLine(l);
        }
        telemetry.update();
    }

    private double calculatedFactor(double defaultFactor, double distanceInches) {
        double decimalValue = (distanceInches - FACTOR_OFFSET_INCHES) / FACTOR_SCALE_INCHES;
        return defaultFactor + decimalValue;
    }

    private double degToRad(double deg) {
        return Math.toRadians(deg);
    }

    public double estimateDistanceFromTy(double tyDegrees, double cameraAngleDegrees,
                                         double targetHeightMeters, double cameraHeightMeters) {
        double a2 = degToRad(tyDegrees);
        double a1 = degToRad(cameraAngleDegrees);
        double numerator = targetHeightMeters - cameraHeightMeters;
        double denominator = Math.tan(a1 + a2);
        if (Math.abs(denominator) < 1e-6) return 0.0;
        return numerator / denominator;
    }

}
