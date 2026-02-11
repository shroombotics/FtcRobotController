package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * _AutoFinalV2
 *
 * State machine derived from {@link StarterBotAuto} but wired to the full competition bot:
 * - MOVE_BACK: creep while scanning AprilTags with Limelight until a non-zero Ty is found.
 * - LAUNCH_READY: compute shooter velocity from Ty and spin up.
 * - LAUNCH: fire 3 balls using release feeders + indexer with DS gating.
 * - ROTATION: turn 45 degrees (positive). Adjust sign if needed.
 * - DRIVE_AWAY: drive a configurable distance to clear the zone.
 */
@Autonomous(name = "_AutoFinalV2", group = "Final")
public class _AutoFinalV2 extends OpMode {

    // ===== Feeder / Shooter timing =====
    private static final double FEED_TIME_SECONDS = 0.8;          // feeder run duration per shot
    private static final double STAGE_TIMEOUT_SECONDS = 1.25;     // max time to advance a ball to DS
    private static final double TIME_BETWEEN_SHOTS = 1.0;         // pause between shots
    private static final int SHOTS_TO_FIRE = 3;

    // ===== Shooter physics =====
    private static final double SHOOTER_RPM_AT_FULL_POWER = 4000.0;
    private static final double SHOOTER_VELOCITY_SAFETY_MULTIPLIER = 1.05;
    private static final double DEFAULT_SHOOTER_RPM_NO_TY = 1100.0;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;
    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(60.0, 0.08, 4.0, 12.5);

    // ===== Limelight geometry =====
    private static final double TY_CACHE_TTL_SECONDS = 0.5;
    private static final double LIMELIGHT_MOUNT_ANGLE_DEG = 15.0;
    private static final double GOAL_HEIGHT_INCHES = 30.0;
    private static final double CAMERA_HEIGHT_INCHES = 14.0;
    private static final double LAUNCHER_FACTOR_BASE = 4.8;
    private static final double VELOCITY_DISTANCE_SCALE = 0.02;
    private static final double WHEEL_RADIUS_INCH = 2.0;
    private static final double TICKS_PER_REV_DEFAULT = 28.0;
    private static final double FACTOR_OFFSET_INCHES = 10.0;
    private static final double FACTOR_SCALE_INCHES = 125.0;
    private static final int[] ALLOWED_APRILTAG_IDS = new int[]{20, 24};

    // ===== Drive kinematics (carried from StarterBotAuto, hardware from _TeleopFinalV6Stateful) =====
    private static final double DRIVE_SPEED = 0.5;
    private static final double ROTATE_SPEED = 0.2;
    private static final double DRIVE_HOLD_SECONDS = 1.0;
    private static final double ROTATE_HOLD_SECONDS = 1.0;
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double ENCODER_TICKS_PER_REV = 537.7;
    private static final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    private static final double TRACK_WIDTH_MM = 404;
    private static final double DRIVE_AWAY_DISTANCE_INCH = 30.0; // change to tune field exit distance
    private static final double ROTATE_ANGLE_DEG = 45.0;
    private static final double MOVE_BACK_POWER = -0.25; // base open-loop power during MOVE_BACK
    private static final boolean DS_ACTIVE_LOW = false; // flip to true if your digital sensor is active-low
    private static final double MOVE_BACK_TIMEOUT = 3.5; // safety timeout while searching Ty

    // ===== Hardware =====
    // drivetrain (matching _TeleopFinalV6Stateful mappings)
    private DcMotor driveFrontRight;
    private DcMotor driveBackRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackLeft;
    private DcMotorEx shooter;
    private CRServo releaseLeft;
    private CRServo releaseRight;
    private DcMotor indexer;
    private DigitalChannel ds;
    private Limelight3A llcam;

    // ===== Internal state =====
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime waitBetweenShotsTimer = new ElapsedTime();
    private final ElapsedTime shooterSpinTimer = new ElapsedTime();
    private final ElapsedTime moveBackTimer = new ElapsedTime();

    private enum AutoState { MOVE_BACK, LAUNCH_READY, LAUNCH, ROTATION, DRIVE_AWAY, COMPLETE }
    private AutoState autoState = AutoState.MOVE_BACK;

    private enum FeedState { FEEDING, STAGING, WAIT_BETWEEN, COMPLETE }
    private FeedState feedState = FeedState.FEEDING;

    private double cachedTy = Double.NaN;
    private long cachedTyNanoTime = 0L;
    private double shooterTargetVelocityTicks = 0.0;
    private double shooterMinVelocityTicks = 0.0;
    private double lastShooterVelocityTicks = Double.NaN;
    private boolean shooterInit = false;
    private boolean shooterActive = false;
    private int shotsFired = 0;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    // ===== Lifecycle =====
    @Override
    public void init() {
        driveFrontRight = hardwareMap.get(DcMotor.class, "DFR");
        driveBackRight = hardwareMap.get(DcMotor.class, "DBR");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "DFL");
        driveBackLeft = hardwareMap.get(DcMotor.class, "DBL");
        shooter = hardwareMap.get(DcMotorEx.class, "SHTR");
        releaseLeft = hardwareMap.get(CRServo.class, "RR");
        releaseRight = hardwareMap.get(CRServo.class, "RL");
        indexer = hardwareMap.get(DcMotor.class, "IND");
        ds = hardwareMap.get(DigitalChannel.class, "DS");
        llcam = hardwareMap.get(Limelight3A.class, "CAM");

        // drivetrain setup
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);

        stopAndResetDriveEncoders();
        setDriveZeroPower(BRAKE);

        // shooter + PIDF
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);

        // feeder directions
        releaseLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        releaseRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setFeederPower(0.0);

        // indexer
        indexer.setDirection(DcMotor.Direction.REVERSE);
        indexer.setPower(0.0);

        // DS
        ds.setMode(DigitalChannel.Mode.INPUT);

        // limelight
        try { llcam.pipelineSwitch(0); llcam.start(); } catch (Exception ignored) {}

        moveBackTimer.reset();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // allow alliance selection pre-start (B=RED, X=BLUE)
        if (gamepad1 != null) {
            if (gamepad1.b) alliance = Alliance.RED;
            else if (gamepad1.x) alliance = Alliance.BLUE;
        }
        telemetry.addData("Alliance", alliance);
        telemetry.addLine("Press B for RED, X for BLUE");
        telemetry.update();
    }

    @Override
    public void loop() {
        pollLimelight();
        monitorShooter();

        switch (autoState) {
            case MOVE_BACK:
                handleMoveBack();
                break;
            case LAUNCH_READY:
                handleLaunchReady();
                break;
            case LAUNCH:
                handleLaunch();
                break;
            case ROTATION:
                double targetAngle = (alliance == Alliance.RED) ? ROTATE_ANGLE_DEG : -ROTATE_ANGLE_DEG;
                if (rotate(ROTATE_SPEED, targetAngle, AngleUnit.DEGREES, ROTATE_HOLD_SECONDS)) {
                    stopAndResetDriveEncoders();
                    autoState = AutoState.DRIVE_AWAY;
                }
                break;
            case DRIVE_AWAY:
                if (drive(DRIVE_SPEED, DRIVE_AWAY_DISTANCE_INCH, DistanceUnit.INCH, DRIVE_HOLD_SECONDS)) {
                    autoState = AutoState.COMPLETE;
                    stopAllMotion();
                }
                break;
            case COMPLETE:
                stopAllMotion();
                break;
        }

        telemetry.addData("State", autoState);
        telemetry.addData("FeedState", feedState);
        telemetry.addData("Shots", shotsFired + "/" + SHOTS_TO_FIRE);
        telemetry.addData("Ty", Double.isNaN(cachedTy) ? "n/a" : String.format("%.2f", cachedTy));
        telemetry.addData("TyFresh", isTyFresh());
        telemetry.addData("ShooterReady", shooterActive);
        telemetry.addData("ShooterInit", shooterInit);
        telemetry.addData("ShooterVel", String.format("%.0f", lastShooterVelocityTicks));
        telemetry.update();
    }

    private void handleMoveBack() {
        boolean hasTy = isTyFresh() && !Double.isNaN(cachedTy) && Math.abs(cachedTy) > 1e-3;
        if (!hasTy && moveBackTimer.seconds() < MOVE_BACK_TIMEOUT) {
            runDriveOpenLoop(MOVE_BACK_POWER, MOVE_BACK_POWER);
            return;
        }
        // stop and latch Ty
        runDriveOpenLoop(0.0, 0.0);
        shooterInit = false;
        shooterActive = false;
        shooterSpinTimer.reset();
        autoState = AutoState.LAUNCH_READY;
    }

    private void handleLaunchReady() {
        // Require a fresh, non-zero Ty before computing shooter RPM
        if (!isTyFresh() || Double.isNaN(cachedTy) || Math.abs(cachedTy) < 1e-3) {
            shooterInit = false;
            shooterActive = false;
            return;
        }

        if (!shooterInit) {
            computeShooterFromTy(cachedTy);
            shooterSpinTimer.reset();
        }
        if (shooterActive) {
            feedState = FeedState.FEEDING;
            feederTimer.reset();
            stageTimer.reset();
            waitBetweenShotsTimer.reset();
            autoState = AutoState.LAUNCH;
        }
    }

    private void handleLaunch() {
        switch (feedState) {
            case FEEDING:
                setFeederPower(1.0);
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    setFeederPower(0.0);
                    shotsFired++;
                    feedState = (shotsFired >= SHOTS_TO_FIRE) ? FeedState.COMPLETE : FeedState.STAGING;
                    stageTimer.reset();
                }
                break;

            case STAGING:
                // push next ball until DS sees it (or timeout)
                setIndexerPower(1.0);
                boolean sensed = dsDetectsObject();
                if (sensed || stageTimer.seconds() > STAGE_TIMEOUT_SECONDS) {
                    setIndexerPower(0.0);
                    feedState = FeedState.WAIT_BETWEEN;
                    waitBetweenShotsTimer.reset();
                }
                break;

            case WAIT_BETWEEN:
                if (waitBetweenShotsTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    feederTimer.reset();
                    feedState = FeedState.FEEDING;
                }
                break;

            case COMPLETE:
                setFeederPower(0.0);
                setIndexerPower(0.0);
                shooter.setVelocity(0.0);
                autoState = AutoState.ROTATION;
                stopAndResetDriveEncoders();
                break;
        }
    }

    private void stopAllMotion() {
        runDriveOpenLoop(0.0, 0.0);
        setFeederPower(0.0);
        setIndexerPower(0.0);
        shooter.setVelocity(0.0);
    }

    // ===== Shooter helpers =====
    private void computeShooterFromTy(double ty) {
        if (Double.isNaN(ty)) return; // do not fallback; require valid Ty
        double targetRPM;
        double distanceMeters = estimateDistanceFromTy(ty, LIMELIGHT_MOUNT_ANGLE_DEG, inchesToMeters(GOAL_HEIGHT_INCHES), inchesToMeters(CAMERA_HEIGHT_INCHES));
        double distanceInches = metersToInches(distanceMeters);
        double factor = calculatedFactor(LAUNCHER_FACTOR_BASE, distanceInches);
        double requiredVelocity = factor + (VELOCITY_DISTANCE_SCALE * distanceInches);
        double newRPM = (requiredVelocity / (2.0 * Math.PI * WHEEL_RADIUS_INCH)) * 60.0;
        targetRPM = newRPM * SHOOTER_VELOCITY_SAFETY_MULTIPLIER;
        double ticksPerRev = TICKS_PER_REV_DEFAULT;
        try { ticksPerRev = shooter.getMotorType().getTicksPerRev(); } catch (Exception ignored) {}
        shooterTargetVelocityTicks = (targetRPM / 60.0) * ticksPerRev;
        double rpmDelta = 0.05 * SHOOTER_RPM_AT_FULL_POWER;
        double deltaTicks = (rpmDelta / 60.0) * ticksPerRev;
        shooterMinVelocityTicks = Math.max(0.0, shooterTargetVelocityTicks - deltaTicks);
        shooterInit = true;
        shooterSpinTimer.reset();
        try { shooter.setVelocity(shooterTargetVelocityTicks); } catch (Exception ignored) {}
    }

    private void monitorShooter() {
        double rawTicks = 0.0;
        try { rawTicks = shooter.getVelocity(); } catch (Exception ignored) {}
        lastShooterVelocityTicks = rawTicks;
        boolean atOrAbove = shooterInit && rawTicks >= shooterMinVelocityTicks;
        boolean sustained = shooterSpinTimer.seconds() >= SHOOTER_SPINUP_SECONDS;
        shooterActive = atOrAbove && sustained;
    }

    private boolean isTyFresh() {
        if (cachedTyNanoTime == 0L) return false;
        double ageSec = (System.nanoTime() - cachedTyNanoTime) / 1e9;
        return ageSec <= TY_CACHE_TTL_SECONDS;
    }

    // ===== Vision =====
    private void pollLimelight() {
        if (llcam == null) return;
        try {
            LLResult r = llcam.getLatestResult();
            if (r != null && r.isValid()) {
                double ty = r.getTy();
                if (!Double.isNaN(ty)) {
                    cachedTy = ty;
                    cachedTyNanoTime = System.nanoTime();
                }
                List<LLResultTypes.FiducialResult> fid = r.getFiducialResults();
                if (fid != null && fid.size() > 0) {
                    for (LLResultTypes.FiducialResult fr : fid) {
                        int id = fr.getFiducialId();
                        if (isAllowedAprilTagId(id)) {
                            double fty = fr.getTargetYDegrees();
                            cachedTy = fty;
                            cachedTyNanoTime = System.nanoTime();
                            break;
                        }
                    }
                }
            }
        } catch (Exception ignored) {}
    }

    private boolean isAllowedAprilTagId(int id) {
        for (int allowed : ALLOWED_APRILTAG_IDS) {
            if (allowed == id) return true;
        }
        return false;
    }

    // ===== Hardware helpers =====
    private void setFeederPower(double power) {
        try { releaseLeft.setPower(power); } catch (Exception ignored) {}
        try { releaseRight.setPower(power); } catch (Exception ignored) {}
    }

    private void setIndexerPower(double power) {
        try { indexer.setPower(power); } catch (Exception ignored) {}
    }

    private boolean dsDetectsObject() {
        try {
            boolean state = ds.getState();
            return DS_ACTIVE_LOW ? !state : state;
        } catch (Exception ignored) {
            return false;
        }
    }

    // ===== Drive helpers copied from StarterBotAuto =====
    private boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        setAllDriveTarget((int) targetPosition, (int) targetPosition, (int) targetPosition, (int) targetPosition);
        setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setAllDrivePower(speed, speed, speed, speed);

        if (Math.abs(targetPosition - driveFrontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }
        return (driveTimer.seconds() > holdSeconds);
    }

    private boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        setAllDriveTarget((int) leftTargetPosition, (int) leftTargetPosition, (int) rightTargetPosition, (int) rightTargetPosition);
        setAllDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        setAllDrivePower(speed, speed, speed, speed);

        if (Math.abs(leftTargetPosition - driveFrontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }
        return (driveTimer.seconds() > holdSeconds);
    }

    private void runDriveOpenLoop(double leftPower, double rightPower) {
        try { driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); } catch (Exception ignored) {}
        try { driveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); } catch (Exception ignored) {}
        try { driveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); } catch (Exception ignored) {}
        try { driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); } catch (Exception ignored) {}
        setAllDrivePower(leftPower, leftPower, rightPower, rightPower);
    }

    private void stopAndResetDriveEncoders() {
        try { driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); } catch (Exception ignored) {}
        try { driveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); } catch (Exception ignored) {}
        try { driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); } catch (Exception ignored) {}
        try { driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); } catch (Exception ignored) {}
    }

    private void setDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        try { driveFrontLeft.setZeroPowerBehavior(behavior); } catch (Exception ignored) {}
        try { driveBackLeft.setZeroPowerBehavior(behavior); } catch (Exception ignored) {}
        try { driveFrontRight.setZeroPowerBehavior(behavior); } catch (Exception ignored) {}
        try { driveBackRight.setZeroPowerBehavior(behavior); } catch (Exception ignored) {}
    }

    private void setAllDriveTarget(int fl, int bl, int fr, int br) {
        try { driveFrontLeft.setTargetPosition(fl); } catch (Exception ignored) {}
        try { driveBackLeft.setTargetPosition(bl); } catch (Exception ignored) {}
        try { driveFrontRight.setTargetPosition(fr); } catch (Exception ignored) {}
        try { driveBackRight.setTargetPosition(br); } catch (Exception ignored) {}
    }

    private void setAllDriveMode(DcMotor.RunMode mode) {
        try { driveFrontLeft.setMode(mode); } catch (Exception ignored) {}
        try { driveBackLeft.setMode(mode); } catch (Exception ignored) {}
        try { driveFrontRight.setMode(mode); } catch (Exception ignored) {}
        try { driveBackRight.setMode(mode); } catch (Exception ignored) {}
    }

    private void setAllDrivePower(double fl, double bl, double fr, double br) {
        try { driveFrontLeft.setPower(fl); } catch (Exception ignored) {}
        try { driveBackLeft.setPower(bl); } catch (Exception ignored) {}
        try { driveFrontRight.setPower(fr); } catch (Exception ignored) {}
        try { driveBackRight.setPower(br); } catch (Exception ignored) {}
    }

    // ===== Math helpers =====
    private double calculatedFactor(double defaultFactor, double distanceInches) {
        double decimalValue = (distanceInches - FACTOR_OFFSET_INCHES) / FACTOR_SCALE_INCHES;
        return defaultFactor + decimalValue;
    }

    private double degToRad(double deg) { return Math.toRadians(deg); }

    private double estimateDistanceFromTy(double tyDegrees, double cameraAngleDegrees, double targetHeightMeters, double cameraHeightMeters) {
        double a2 = degToRad(tyDegrees);
        double a1 = degToRad(cameraAngleDegrees);
        double numerator = targetHeightMeters - cameraHeightMeters;
        double denominator = Math.tan(a1 + a2);
        if (Math.abs(denominator) < 1e-6) return 0.0;
        return numerator / denominator;
    }

    private double inchesToMeters(double inches) { return inches * 0.0254; }
    private double metersToInches(double meters) { return meters / 0.0254; }
}
