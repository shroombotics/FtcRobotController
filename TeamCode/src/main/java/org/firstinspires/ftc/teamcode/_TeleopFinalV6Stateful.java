package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.List;

/**
 * _TeleopFinalV6Stateful
 *
 * Mini contract for new drivers:
 * - Inputs: gamepad1 sticks (drive), right-stick-button (align/turn to tag), LB (spin shooter), RB (feed), Y (intake+indexer toggle), X (quick stop), DS auto-stop on object.
 * - Outputs: drive motors, intake/indexer, feeders, shooter, optional light.
 * - Safety: flip DS_ACTIVE_LOW if polarity is wrong; STRAFE_INVERT flips rotation direction; quick-stop on X.
 * V6: alignment rotates robot toward tag (no strafe). Alignment is on right-stick button; left bumper only spins shooter.
 */
@TeleOp(name = "_TeleopFinalV6Stateful")
public class _TeleopFinalV6Stateful extends OpMode {

    // DRIVETRAIN
    private DcMotor driveFrontRight;
    private DcMotor driveBackRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackLeft;
    private static final double DEFAULT_MOTOR_SPEED = 1.0;
    private static final double SLOW_TRIGGER_THRESHOLD = 0.1;
    private static final double SLOW_MODE_DIVISOR = 4.0;
    private static final double ROTATION_SCALE = 0.75;

    // INTAKE / INDEXER / FEEDERS
    private CRServo intakeLeft;
    private CRServo intakeRight;
    private CRServo releaseRight;
    private CRServo releaseLeft;
    private DcMotor indexer;
    private static final double INTAKE_STOP_SPEED = 0.0;
    private static final double INTAKE_FULL_SPEED = 1.0;
    private static final double INDEXER_STOP_SPEED = 0.0;
    private static final double INDEXER_FULL_SPEED = 1.0;
    private static final double RELEASE_STOP_SPEED = 0.0;
    private static final double RELEASE_FULL_SPEED = 1.0;
    private static final double FEED_TIME_SECONDS = 1.0;
    private static final double FEEDER_FULL_SPEED = 1.0;

    // SHOOTER
    private DcMotorEx shooter;
    private static final double SHOOTER_RPM_AT_FULL_POWER = 4000.0;
    private static final double SHOOTER_VELOCITY_SAFETY_MULTIPLIER = 1.05;
    private static final double DEFAULT_SHOOTER_RPM_NO_TY = 1100.0;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;
    // PIDF tuning for shooter velocity regulation (units: ticks/sec based)
    // Boost F for stronger initial speed, keep mild I to hold through the burst, trim D for stability.
    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(60.0, 0.08, 4.0, 12.5);

    // ALIGNMENT (rotation toward target using Tx). Flip STRAFE_INVERT if turn direction is wrong.
    private static final double ALIGN_KP = 0.04;              // degrees -> power gain
    // Lower clamp to slow the head turn; adjust if still too fast.
    private static final double ALIGN_MAX_POWER = 0.30;       // clamp of rotation
    private static final double ALIGN_TX_THRESHOLD_DEG = 1.0; // considered "aligned"
    // Negative here fixes robots that were rotating the wrong way (not turning toward tag)
    private static final double STRAFE_INVERT = -1.0;         // flip sign if hardware reversed

    // BUTTON / INPUT CONSTANTS
    private static final double QUICKSTOP_POWER = 0.0; // power applied to all motors when X is pressed
    private static final double INTAKE_MANUAL_STOP_POWER = 0.0;
    private static final double INDEXER_MANUAL_STOP_POWER = 0.0;
    private static final double SHOOTER_MANUAL_STOP_POWER = 0.0;
    private static final double FEEDER_MANUAL_STOP_POWER = 0.0;

    // VISION / GEOMETRY
    private Limelight3A llcam = null;
    private static final double TX_CACHE_TTL_SECONDS = 0.5;
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
    private static final double TX_ACQUIRE_TIMEOUT_SECONDS = 1.0;
    private static final double ALIGN_TIMEOUT_SECONDS = 6.0;

    // GAMEPLAY TIMERS / LIMITS
    private static final double LEFT_BUMPER_WAIT_TIMEOUT_SECONDS = 8.0;
    private static final int MAX_SHOTS = 6;
    private static final double SHOOT_WINDOW_SECONDS = 10.0;
    private static final double JOYSTICK_DEADBAND = 0.05;

    // KICKSTAND (optional)
    private Servo kickstandFront;
    private Servo kickstandBack;
    private static final double KICKSTAND_BACK_REST = 0.585;
    private static final double KICKSTAND_BACK_DEPLOY = 0.5;
    private static final double KICKSTAND_FRONT_REST = 0.49;
    private static final double KICKSTAND_FRONT_DEPLOY = 0.575;

    // RELEASE SERVO DIRECTIONS (for clarity and consistency)
    private static final DcMotorSimple.Direction RELEASE_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction RELEASE_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // OPTIONAL LIGHT
    private ServoImplEx light = null;

    // DIGITAL SENSOR (DS) for auto-stop on objects
    private DigitalChannel ds;
    // Digital sensor polarity: true if sensor outputs LOW when object present (common beam-break)
    private static final boolean DS_ACTIVE_LOW = false;
    // Allowed AprilTag IDs for precise targeting; change these to suit your field tags.
    private static final int[] ALLOWED_APRILTAG_IDS = new int[] { 20, 24 };

    private static final boolean DEBUG_TELEMETRY = false;

    double motor_speed;
    double motor_speed_default;

    boolean intakesRunning;
    boolean indexerRunning = false;

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevA = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    // latch to honor an operator quick-stop until explicitly restarted
    private boolean manualStopLatched = false;
    private boolean prevRightStick = false;

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
    // one-shot flag to stage the next ball after DS clears
    private boolean stageNextBallPending = false;

    private boolean hasValidAprilTag = false;
    private int latestAprilTagId = -1;
    // A small flag to indicate a precomputed shooter setpoint is available
    private boolean shooterSetpointPrecomputed = false;

    private enum RobotState { DRIVE, ALIGN, SPIN_UP, SHOOT_READY }
    private RobotState robotState = RobotState.DRIVE;
    private int shotAttempts = 0;

    private double lastAlignTx = Double.NaN;
    private double lastAlignSp = 0.0;
    private double lastAlignTy = Double.NaN;

    // tiny helpers to reduce try/catch noise when mapping hardware
    private <T> T safeMap(Class<T> clazz, String name) {
        try { return hardwareMap.get(clazz, name); } catch (Exception ignored) { return null; }
    }

    private void safeSetPosition(Servo servo, double position) {
        try { if (servo != null) servo.setPosition(position); } catch (Exception ignored) {}
    }

    @Override
    public void init() {
        driveFrontRight = safeMap(DcMotor.class, "DFR");
        driveBackRight = safeMap(DcMotor.class, "DBR");
        driveFrontLeft = safeMap(DcMotor.class, "DFL");
        driveBackLeft = safeMap(DcMotor.class, "DBL");
        intakeLeft = safeMap(CRServo.class, "LF");
        intakeRight = safeMap(CRServo.class, "RF");

        motor_speed_default = DEFAULT_MOTOR_SPEED;
        motor_speed = motor_speed_default;

        if (driveFrontRight != null) try { driveFrontRight.setDirection(DcMotor.Direction.FORWARD); } catch (Exception ignored) {}
        if (driveBackRight != null) try { driveBackRight.setDirection(DcMotor.Direction.FORWARD); } catch (Exception ignored) {}
        if (driveFrontLeft != null) try { driveFrontLeft.setDirection(DcMotor.Direction.REVERSE); } catch (Exception ignored) {}
        if (driveBackLeft != null) try { driveBackLeft.setDirection(DcMotor.Direction.REVERSE); } catch (Exception ignored) {}

        if (intakeLeft != null) try { intakeLeft.setDirection(DcMotor.Direction.REVERSE); } catch (Exception ignored) {}
        if (intakeRight != null) try { intakeRight.setDirection(DcMotor.Direction.FORWARD); } catch (Exception ignored) {}

        // optional kickstands
        kickstandFront = safeMap(Servo.class, "KF");
        kickstandBack = safeMap(Servo.class, "KB");
        safeSetPosition(kickstandFront, KICKSTAND_FRONT_REST);
        safeSetPosition(kickstandBack, KICKSTAND_BACK_REST);

        shooter = safeMap(DcMotorEx.class, "SHTR");
        light = safeMap(ServoImplEx.class, "SL");
        llcam = safeMap(Limelight3A.class, "CAM");
        releaseLeft = safeMap(CRServo.class, "RR");
        releaseRight = safeMap(CRServo.class, "RL");

        // configure shooter PIDF if present
        if (shooter != null) {
            try {
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);
            } catch (Exception ignored) {}
        }

        // map indexer motor (single expected name: "IND")
        indexer = safeMap(DcMotor.class, "IND");
        if (indexer != null) {
            try { indexer.setDirection(DcMotor.Direction.REVERSE); } catch (Exception ignored) {}
            try { indexer.setPower(0.0); } catch (Exception ignored) {}
            indexerMappedName = "IND";
        }

        // map digital sensor named "DS" and set it as input if present
        ds = safeMap(DigitalChannel.class, "DS");
        if (ds != null) {
            try { ds.setMode(DigitalChannel.Mode.INPUT); } catch (Exception ignored) {}
        }

        if (llcam != null) { try { llcam.pipelineSwitch(0); llcam.start(); hasLimelight = true; } catch (Exception ignored) {} }
        if (releaseLeft != null || releaseRight != null) {
            hasFeeders = true;
            try { if (releaseLeft != null) releaseLeft.setPower(0.0); } catch (Exception ignored) {}
            try { if (releaseRight != null) releaseRight.setPower(0.0); } catch (Exception ignored) {}
            try { if (releaseLeft != null) releaseLeft.setDirection(RELEASE_LEFT_DIRECTION); } catch (Exception ignored) {}
            try { if (releaseRight != null) releaseRight.setDirection(RELEASE_RIGHT_DIRECTION); } catch (Exception ignored) {}
        }

    telemetry.addData("Status", "Initialized TeleOp Simplified V6");
    // Use autoClear so we don't accumulate lines and cause scrolling on the driver station.
    telemetry.setAutoClear(true);
    }

    @Override
    public void loop() {
        // Keep polling Limelight and fiducials
        pollLimelightCacheAndFiducials();

        if (waitingForTx) {
            double tx = cachedTx; double ty = cachedTy;
            if (!Double.isNaN(tx)) {
                waitingForTx = false;
                lastAlignTy = ty; alignTimer.reset(); lastAlignTx = tx; robotState = RobotState.ALIGN;
            } else if (txAcquireTimer.seconds() > TX_ACQUIRE_TIMEOUT_SECONDS) {
                waitingForTx = false; showOneShotTelemetry("Limelight", "no tx - aim at tag and press again"); prevRightStick = false;
            } else {
                telemetry.addData("Limelight", "acquiring tx...");
            }
        }

    // Process toggles and quick-stop first so buttons work in any state
    processIntakeAndLoaderButtons();
    handleQuickStopOnX();
    handleManualShooterOnA();
    kickstand();

    // Update intake/indexer/release automatically based on DS sensor (if present)
    updateIntakeIndexerReleaseFromSensor();

    // Ensure indexer power is always applied to hardware so Y-toggle works in any state
    try { if (indexer != null) setIndexerPower(indexerRunning ? INDEXER_FULL_SPEED : INDEXER_STOP_SPEED); } catch (Exception ignored) {}

    if (detectJoystickMovement() && robotState != RobotState.DRIVE) cancelShootingAndReturnDrive("joystick moved");

    switch (robotState) {
            case DRIVE:
                runDriveControls();
                handleRightStickStart(); // alignment is now initiated by right-stick press
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

        // Left bumper now is only used to spin up the shooter
        handleLeftBumperSpinUp();
        // optional backup alignment while holding right trigger (strafe-based)
        handleRightTriggerStrafeAlign();

        handleShootingWithRightBumperSimplified();
        updateSummaryTelemetry();
    }

    // -- alignment uses rotation instead of strafing --
    private void performAlignmentStepSimple() {
        if (llcam == null) { robotState = RobotState.DRIVE; shotAttempts = 0; return; }

        double tx = Double.NaN;
        if (isCachedTxFresh()) {
            tx = cachedTx;
        } else {
            try {
                LLResult r = llcam.getLatestResult();
                if (r != null && r.isValid()) tx = r.getTx();
            } catch (Exception ignored) {}
        }
        if (Double.isNaN(tx)) { telemetry.addData("Align", "no tx yet - aim and retry"); return; }

        // if aligned, stop rotation and compute shooter setpoint (do NOT start shooter here)
        if (Math.abs(tx) <= ALIGN_TX_THRESHOLD_DEG) {
            setRotationPower(0.0);
            double usedTy = !Double.isNaN(cachedTy) ? cachedTy : Double.NaN;
            computeShooterFromTyNoSpin(usedTy);
            shooterSetpointPrecomputed = true;
            // alignment finished: return to DRIVE so driver can press left bumper to spin up
            robotState = RobotState.DRIVE;
            return;
        }

        // proportional control applied to rotation (turn toward the tag) with simple clamp
        double sp = Range.clip(ALIGN_KP * tx, -ALIGN_MAX_POWER, ALIGN_MAX_POWER) * STRAFE_INVERT;

        lastAlignTx = tx; lastAlignSp = sp; lastAlignTy = cachedTy;
        setRotationPower(sp);
    }

    // helper: rotation-only power (turn in place). Positive sp -> left side forward, right side backward.
    private void setRotationPower(double sp) {
        // left wheels forward, right wheels backward -> rotate
        setDrivePowersDirect(sp, sp, -sp, -sp);
    }

    // Left bumper now only spins up the shooter immediately
    private void handleLeftBumperSpinUp() {
        boolean leftBumper = gamepad1.left_bumper;
        boolean leftEdge = leftBumper && !prevLeftBumper;
        if (leftEdge) {
            // Start shooter with precomputed setpoint when available, otherwise compute from cachedTy
            if (shooterSetpointPrecomputed && shooterTargetVelocityTicks > 0.0 && shooter != null) {
                try { shooter.setVelocity(shooterTargetVelocityTicks); } catch (Exception ignored) {}
                shooterInit = true; shooterSpinTimer.reset(); robotState = RobotState.SPIN_UP;
            } else {
                computeShooterFromTy(Double.isNaN(cachedTy) ? Double.NaN : cachedTy);
            }
        }
        prevLeftBumper = leftBumper;
    }

    // Backup: strafe-based alignment while holding right trigger (uses same ALIGN_KP).
    private void handleRightTriggerStrafeAlign() {
        double rt = gamepad1.right_trigger;
        if (rt < 0.1) return; // only when held
        double tx = Double.NaN;
        if (isCachedTxFresh()) {
            tx = cachedTx;
        } else {
            try {
                LLResult r = llcam != null ? llcam.getLatestResult() : null;
                if (r != null && r.isValid()) tx = r.getTx();
            } catch (Exception ignored) {}
        }
        if (Double.isNaN(tx)) return;

        // Use strafe to center on tx; clamp with ALIGN_MAX_POWER and STRAFE_INVERT.
        double sp = Range.clip(ALIGN_KP * tx, -ALIGN_MAX_POWER, ALIGN_MAX_POWER) * STRAFE_INVERT;
        setDrivePowersDirect(sp, -sp, -sp, sp);
        lastAlignTx = tx; lastAlignSp = sp; lastAlignTy = cachedTy;
    }

    // New: alignment triggered by right-stick button
    private void handleRightStickStart() {
        boolean rs = gamepad1.right_stick_button;
        boolean rsEdge = rs && !prevRightStick;
        if (rsEdge) {
            if (robotState != RobotState.DRIVE) { prevRightStick = rs; return; }
            if (hasLimelight && llcam != null) {
                if (isCachedTxFresh()) { lastAlignTx = cachedTx; lastAlignTy = cachedTy; robotState = RobotState.ALIGN; alignTimer.reset(); prevRightStick = rs; return; }
                try { LLResult r = llcam.getLatestResult(); if (r != null && r.isValid()) { double tx = r.getTx(); double ty = r.getTy(); if (!Double.isNaN(tx)) { lastAlignTx = tx; lastAlignTy = ty; robotState = RobotState.ALIGN; alignTimer.reset(); prevRightStick = rs; return; } } } catch (Exception ignored) {}
                waitingForTx = true; txAcquireTimer.reset(); showOneShotTelemetry("Limelight", "acquiring tx..."); prevRightStick = rs; robotState = RobotState.ALIGN; return;
            }
        }
        prevRightStick = rs;
    }

    // other helpers copied from V5 (intake/indexer toggles, DS sensor, feeder, etc.)
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
                            // mark to stage next ball after DS clears
                            stageNextBallPending = true;
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

    private void setDrivePowersDirect(double pFL, double pBL, double pFR, double pBR) {
        try { driveFrontLeft.setPower(pFL); } catch (Exception ignored) {}
        try { driveBackLeft.setPower(pBL); } catch (Exception ignored) {}
        try { driveFrontRight.setPower(pFR); } catch (Exception ignored) {}
        try { driveBackRight.setPower(pBR); } catch (Exception ignored) {}
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
        double y = -gamepad1.left_stick_y * motor_speed;
        double x = gamepad1.left_stick_x * motor_speed;
        double rx = gamepad1.right_stick_x * ROTATION_SCALE * motor_speed;

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
        boolean toggleButton = gamepad1.y;
        boolean toggleEdge = toggleButton && !prevY;
        if (toggleEdge) {
            boolean newState = !(intakesRunning || indexerRunning);
            intakesRunning = newState;
            indexerRunning = newState;
            // if operator re-enables, clear any latched quick-stop
            if (newState) manualStopLatched = false;
        }
        prevY = toggleButton;

        if (intakesRunning) {
            setIntakePower(INTAKE_FULL_SPEED, INTAKE_FULL_SPEED);
        } else {
            setIntakePower(INTAKE_STOP_SPEED, INTAKE_STOP_SPEED);
        }

        if (indexer != null) setIndexerPower(indexerRunning ? INDEXER_FULL_SPEED : INDEXER_STOP_SPEED);

        // If no sensor is present, also mirror feeder power to the toggle (avoid overriding an active feed cycle)
        if (ds == null && !isFeeding) setFeederPower(intakesRunning ? FEEDER_FULL_SPEED : FEEDER_MANUAL_STOP_POWER);
    }

    private boolean dsDetectsObject() {
        if (ds == null) return false;
        try { boolean state = ds.getState(); return DS_ACTIVE_LOW ? !state : state; } catch (Exception ignored) { return false; }
    }

    private void updateIntakeIndexerReleaseFromSensor() {
        if (manualStopLatched) return; // honor quick-stop until operator re-enables
        if (ds == null) return;
        boolean objectPresent = dsDetectsObject();
        if (objectPresent) {
            //setIntakePower(INTAKE_STOP_SPEED, INTAKE_STOP_SPEED);
            setIndexerPower(INDEXER_STOP_SPEED);
            // also drop run flags so feeder/intake/indexer don't keep running and push past DS
            //intakesRunning = false;
            indexerRunning = false;
            if (!isFeeding) setFeederPower(FEEDER_MANUAL_STOP_POWER);
        } else {
            // sensor clear: if a shot just fired, stage next ball once; otherwise honor toggles
            if (stageNextBallPending) {
                intakesRunning = true;
                indexerRunning = true;
                stageNextBallPending = false;
            }
            setIntakePower(intakesRunning ? INTAKE_FULL_SPEED : INTAKE_STOP_SPEED, intakesRunning ? INTAKE_FULL_SPEED : INTAKE_STOP_SPEED);
            setIndexerPower(indexerRunning ? INDEXER_FULL_SPEED : INDEXER_STOP_SPEED);
            if (!isFeeding) setFeederPower(intakesRunning ? FEEDER_FULL_SPEED : FEEDER_MANUAL_STOP_POWER);
        }
    }

    private void handleQuickStopOnX() {
        boolean xPressed = gamepad1.x;
        if (xPressed && !prevX) {
            setFeederPower(0.0); isFeeding = false; feederTimer.reset(); stageNextBallPending = false;
            shooterInit = false; shooterActive = false; setShooterPower(0.0);
            setDrivePowersDirect(0.0, 0.0, 0.0, 0.0); setIntakePower(0.0, 0.0);
            try { if (indexer != null) indexer.setPower(0.0); } catch (Exception ignored) {}
            indexerRunning = false; intakesRunning = false; manualStopLatched = true; shotAttempts = 0; waitingForTx = false; showOneShotTelemetry("QuickStop", "Emergency stop (X)"); robotState = RobotState.DRIVE;
        }
        prevX = xPressed;
    }

    // Manual shooter fallback: press A to spin to default RPM when tag/ty is unavailable
    private void handleManualShooterOnA() {
        boolean aPressed = gamepad1.a;
        boolean aEdge = aPressed && !prevA;
        if (aEdge) {
            // Use the same computation path as alignment/left-bumper: derive ticks for default RPM (1100) then spin
            computeShooterFromTyNoSpin(Double.NaN);
            if (shooter != null) try { shooter.setVelocity(shooterTargetVelocityTicks); } catch (Exception ignored) {}
            shooterInit = true;
            shooterSpinTimer.reset();
            robotState = RobotState.SPIN_UP;
            showOneShotTelemetry("Shooter", "Manual default RPM (A)");
        }
        prevA = aPressed;
    }

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

    private void enterSpinUp() { robotState = RobotState.SPIN_UP; shootStateTimer.reset(); }

    private void cancelShootingAndReturnDrive(String reason) {
        setFeederPower(0.0);
        shooterInit = false;
        shooterActive = false;
        setShooterPower(0.0);
        robotState = RobotState.DRIVE;
        shotAttempts = 0;
        // clear any pending auto-stage so DS won't restart intake unexpectedly
        stageNextBallPending = false;
        showOneShotTelemetry("Abort", reason);
    }

    private void pollLimelightCacheAndFiducials() {
        if (llcam == null) return;
        try {
            LLResult r = llcam.getLatestResult();
            if (r != null && r.isValid()) {
                double tx = r.getTx(); double ty = r.getTy();
                if (!Double.isNaN(tx)) { cachedTx = tx; cachedTy = ty; cachedTxNanoTime = System.nanoTime(); }
                List<LLResultTypes.FiducialResult> fid = r.getFiducialResults();
                if (fid != null && fid.size() > 0) {
                    boolean found = false;
                    for (LLResultTypes.FiducialResult fr : fid) {
                        int id = fr.getFiducialId();
                        if (isAllowedAprilTagId(id)) {
                            latestAprilTagId = id; hasValidAprilTag = true; found = true;
                            double ftx = fr.getTargetXDegrees(); double fty = fr.getTargetYDegrees();
                            cachedTx = ftx; cachedTy = fty; cachedTxNanoTime = System.nanoTime();
                            computeShooterFromTyNoSpin(cachedTy);
                            shooterSetpointPrecomputed = true;
                            break;
                        }
                    }
                    if (!found) { hasValidAprilTag = false; latestAprilTagId = -1; }
                }
            }
        } catch (Exception ignored) {}
    }

    private boolean isAllowedAprilTagId(int id) { if (ALLOWED_APRILTAG_IDS == null) return false; for (int a : ALLOWED_APRILTAG_IDS) if (a == id) return true; return false; }

    private boolean isCachedTxFresh() { if (cachedTxNanoTime == 0L) return false; double ageSec = (System.nanoTime() - cachedTxNanoTime) / 1e9; return ageSec <= TX_CACHE_TTL_SECONDS; }

    private void computeShooterFromTyNoSpin(double ty) {
        double targetRPM;
        if (!Double.isNaN(ty)) {
            double distanceMeters = estimateDistanceFromTy(ty, LIMELIGHT_MOUNT_ANGLE_DEG, GOAL_HEIGHT_INCHES * INCH_TO_METER, CAMERA_HEIGHT_INCHES * INCH_TO_METER);
            double distanceInches = distanceMeters / INCH_TO_METER;
            double factor = calculatedFactor(LAUNCHER_FACTOR_BASE, distanceInches);
            double requiredVelocity = factor + (VELOCITY_DISTANCE_SCALE * distanceInches);
            double newRPM = (requiredVelocity / (2.0 * Math.PI * WHEEL_RADIUS_INCH)) * 60.0;
            targetRPM = newRPM * SHOOTER_VELOCITY_SAFETY_MULTIPLIER;
            if (DEBUG_TELEMETRY) showOneShotTelemetry("Calc", String.format("ty=%.2f dist_in=%.1f rpm=%.1f", ty, distanceInches, targetRPM));
        } else { targetRPM = DEFAULT_SHOOTER_RPM_NO_TY; if (DEBUG_TELEMETRY) showOneShotTelemetry("Calc", String.format("ty=NaN default_rpm=%.1f", targetRPM)); }
        double ticksPerRev = TICKS_PER_REV_DEFAULT; if (shooter != null) try { ticksPerRev = shooter.getMotorType().getTicksPerRev(); } catch (Exception ignored) {}
        shooterTargetVelocityTicks = (targetRPM / 60.0) * ticksPerRev;
        shooterTargetPower = Math.max(0.0, Math.min(1.0, (targetRPM) / SHOOTER_RPM_AT_FULL_POWER));
        double rpmDelta = 0.05 * SHOOTER_RPM_AT_FULL_POWER; double deltaTicks = (rpmDelta / 60.0) * ticksPerRev; double shooterMinVelocityTicksLocal = Math.max(0.0, shooterTargetVelocityTicks - deltaTicks);
        shooterMinVelocityTicks = shooterMinVelocityTicksLocal;
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
        } else { targetRPM = DEFAULT_SHOOTER_RPM_NO_TY; if (DEBUG_TELEMETRY) showOneShotTelemetry("Calc", String.format("ty=NaN default_rpm=%.1f", targetRPM)); }
        double ticksPerRev = TICKS_PER_REV_DEFAULT; if (shooter != null) try { ticksPerRev = shooter.getMotorType().getTicksPerRev(); } catch (Exception ignored) {}
        shooterTargetVelocityTicks = (targetRPM / 60.0) * ticksPerRev;
        shooterTargetPower = Math.max(0.0, Math.min(1.0, (targetRPM) / SHOOTER_RPM_AT_FULL_POWER));
        double rpmDelta = 0.05 * SHOOTER_RPM_AT_FULL_POWER; double deltaTicks = (rpmDelta / 60.0) * ticksPerRev; double shooterMinVelocityTicksLocal = Math.max(0.0, shooterTargetVelocityTicks - deltaTicks);
        shooterInit = true; shooterSpinTimer.reset(); try { if (shooter != null) shooter.setVelocity(shooterTargetVelocityTicks); } catch (Exception ignored) {} shooterMinVelocityTicks = shooterMinVelocityTicksLocal;
    }

    private void monitorShooterAndSetLed() {
        double rawTicks = 0.0; double ticksPerRev = 28.0;
        if (shooter != null) { try { rawTicks = shooter.getVelocity(); } catch (Exception ignored) {} try { ticksPerRev = shooter.getMotorType().getTicksPerRev(); } catch (Exception ignored) {} }
        lastShooterVelocityTicks = rawTicks;
        boolean atOrAbove = shooterInit && rawTicks >= shooterMinVelocityTicks;
        boolean sustained = shooterSpinTimer.seconds() >= SHOOTER_SPINUP_SECONDS;
        if (atOrAbove && sustained) { shooterActive = true; if (robotState == RobotState.SPIN_UP) robotState = RobotState.SHOOT_READY; } else shooterActive = false;
        try { if (shooterActive) { if (light != null) light.setPosition(0.29); } else if (shooterInit) { if (light != null) light.setPosition(0.64); } else { if (light != null) light.setPosition(0.52); } } catch (Exception ignored) {}
    }

    private void updateSummaryTelemetry() {
        telemetry.clear();
        telemetry.addData("State", robotState.toString());
        telemetry.addData("Intake", intakesRunning ? "ON" : "OFF");
        telemetry.addData("Indexer", indexer != null ? (indexerRunning ? "ON" : "OFF") : "MISSING");
        telemetry.addData("HasLimelight", hasLimelight);
        if (hasLimelight) telemetry.addData("LL tx/ty", (!Double.isNaN(cachedTx) ? String.format("%.2f" , cachedTx) : "n/a") + " / " + (!Double.isNaN(cachedTy) ? String.format("%.2f", cachedTy) : "n/a"));
        if (ds != null) { telemetry.addData("DS", dsDetectsObject() ? "OBJECT" : "CLEAR"); } else { telemetry.addData("DS", "MISSING"); }
        telemetry.addData("AprilTag", hasValidAprilTag ? String.format("seen id=%d", latestAprilTagId) : "none");

        String precompStatus = shooterSetpointPrecomputed && shooterTargetVelocityTicks > 0.0 ? String.format("SET %.0f ticks/s pwr %.2f", shooterTargetVelocityTicks, shooterTargetPower) : "NONE";
        telemetry.addData("Precomp", precompStatus);

        switch (robotState) {
            case DRIVE:
                telemetry.addData("Shooter", shooterInit ? (shooterActive ? "READY" : "SPINUP") : "IDLE");
                try { double pfl = driveFrontLeft.getPower(); double pbl = driveBackLeft.getPower(); double pfr = driveFrontRight.getPower(); double pbr = driveBackRight.getPower(); telemetry.addData("pFL/pBL/pFR/pBR", String.format("%.2f %.2f %.2f %.2f", pfl, pbl, pfr, pbr)); } catch (Exception ignored) {}
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

    private void showOneShotTelemetry(String title, String... lines) { shotCounter++; telemetry.clearAll(); telemetry.addData("Event", String.format("%s #%d", title, shotCounter)); for (String l : lines) telemetry.addLine(l); telemetry.update(); }

    private double calculatedFactor(double defaultFactor, double distanceInches) { double decimalValue = (distanceInches - FACTOR_OFFSET_INCHES) / FACTOR_SCALE_INCHES; return defaultFactor + decimalValue; }

    private double degToRad(double deg) { return Math.toRadians(deg); }

    public double estimateDistanceFromTy(double tyDegrees, double cameraAngleDegrees, double targetHeightMeters, double cameraHeightMeters) {
        double a2 = degToRad(tyDegrees); double a1 = degToRad(cameraAngleDegrees); double numerator = targetHeightMeters - cameraHeightMeters; double denominator = Math.tan(a1 + a2); if (Math.abs(denominator) < 1e-6) return 0.0; return numerator / denominator;
    }

}
