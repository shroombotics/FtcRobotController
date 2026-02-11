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

@Autonomous(name = "_AutoFinalV3", group = "Final")
public class _AutoFinalV3 extends OpMode {

    // ===== Feeder / Shooter timing =====
    private static final double FEED_TIME_SECONDS = 0.8;          // feeder run duration per shot
    private static final double STAGE_TIMEOUT_SECONDS = 1.25;     // max time to advance a ball to DS
    private static final double TIME_BETWEEN_SHOTS = 1.0;         // pause between shots
    private static final int SHOTS_TO_FIRE = 3;

    // ===== Shooter physics =====
    private static final double SHOOTER_RPM_AT_FULL_POWER = 4000.0;
    private static final double SHOOTER_VELOCITY_SAFETY_MULTIPLIER = 1.05;
    private static final double SHOOTER_SPINUP_SECONDS = 1.0;
    private static final PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(60.0, 0.08, 4.0, 12.5);

    // ===== Vision =====
    private static final double TX_CACHE_TTL_SECONDS = 0.5;
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

    // ===== Alignment =====
    private static final double ALIGN_KP = 0.04;
    private static final double ALIGN_MAX_POWER = 0.30;
    private static final double ALIGN_TX_THRESHOLD_DEG = 1.0;
    private static final double ALIGN_TIMEOUT_SECONDS = 6.0;
    private static final double STRAFE_INVERT = -1.0;

    // ===== Drive kinematics =====
    private static final double DRIVE_SPEED = 0.5;
    private static final double ROTATE_SPEED = 0.2;
    private static final double DRIVE_HOLD_SECONDS = 1.0;
    private static final double ROTATE_HOLD_SECONDS = 1.0;
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double ENCODER_TICKS_PER_REV = 537.7;
    private static final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    private static final double TRACK_WIDTH_MM = 404;

    private static final double MOVE_FORWARD_DISTANCE_INCH = 10.0; // tune forward distance before first turn
    private static final double ROTATE1_ANGLE_DEG = 45.0;
    private static final double ROTATE2_ANGLE_DEG = 45.0;
    private static final double DRIVE_AWAY_DISTANCE_INCH = 30.0;
    private static final boolean DS_ACTIVE_LOW = false; // flip to true if digital sensor is active-low

    // drivetrain
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

    // timers
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final ElapsedTime waitBetweenShotsTimer = new ElapsedTime();
    private final ElapsedTime shooterSpinTimer = new ElapsedTime();
    private final ElapsedTime alignTimer = new ElapsedTime();

    private enum AutoState { MOVE_FORWARD, ROTATION1, ALIGN, LAUNCH_READY, LAUNCH, ROTATION2, DRIVE_AWAY, COMPLETE }
    private AutoState autoState = AutoState.MOVE_FORWARD;

    private enum FeedState { FEEDING, STAGING, WAIT_BETWEEN, COMPLETE }
    private FeedState feedState = FeedState.FEEDING;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    private double cachedTx = Double.NaN;
    private long cachedTxNanoTime = 0L;
    private double cachedTy = Double.NaN;
    private long cachedTyNanoTime = 0L;
    private double shooterTargetVelocityTicks = 0.0;
    private double shooterMinVelocityTicks = 0.0;
    private double lastShooterVelocityTicks = Double.NaN;
    private boolean shooterInit = false;
    private boolean shooterActive = false;
    private int shotsFired = 0;

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

        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveBackRight.setDirection(DcMotor.Direction.FORWARD);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);

        stopAndResetDriveEncoders();
        setDriveZeroPower(BRAKE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PIDF);

        releaseLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        releaseRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setFeederPower(0.0);

        indexer.setDirection(DcMotor.Direction.REVERSE);
        indexer.setPower(0.0);

        ds.setMode(DigitalChannel.Mode.INPUT);

        try { llcam.pipelineSwitch(0); llcam.start(); } catch (Exception ignored) {}

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
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
            case MOVE_FORWARD:
                if (drive(DRIVE_SPEED, MOVE_FORWARD_DISTANCE_INCH, DistanceUnit.INCH, DRIVE_HOLD_SECONDS)) {
                    stopAndResetDriveEncoders();
                    autoState = AutoState.ROTATION1;
                }
                break;

            case ROTATION1:
                double rot1 = (alliance == Alliance.RED) ? ROTATE1_ANGLE_DEG : -ROTATE1_ANGLE_DEG;
                if (rotate(ROTATE_SPEED, rot1, AngleUnit.DEGREES, ROTATE_HOLD_SECONDS)) {
                    stopAndResetDriveEncoders();
                    alignTimer.reset();
                    autoState = AutoState.ALIGN;
                }
                break;

            case ALIGN:
                if (performAlignment()) {
                    stopAndResetDriveEncoders();
                    autoState = AutoState.LAUNCH_READY;
                }
                break;

            case LAUNCH_READY:
                if (!isTyFresh() || Double.isNaN(cachedTy) || Math.abs(cachedTy) < 1e-3) {
                    shooterInit = false;
                    shooterActive = false;
                    break;
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
                break;

            case LAUNCH:
                handleLaunchCycle();
                break;

            case ROTATION2:
                double rot2 = (alliance == Alliance.RED) ? ROTATE2_ANGLE_DEG : -ROTATE2_ANGLE_DEG;
                if (rotate(ROTATE_SPEED, rot2, AngleUnit.DEGREES, ROTATE_HOLD_SECONDS)) {
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
        telemetry.addData("Alliance", alliance);
        telemetry.addData("FeedState", feedState);
        telemetry.addData("Shots", shotsFired + "/" + SHOTS_TO_FIRE);
        telemetry.addData("Tx", Double.isNaN(cachedTx) ? "n/a" : String.format("%.2f", cachedTx));
        telemetry.addData("TxFresh", isTxFresh());
        telemetry.addData("Ty", Double.isNaN(cachedTy) ? "n/a" : String.format("%.2f", cachedTy));
        telemetry.addData("TyFresh", isTyFresh());
        telemetry.addData("ShooterReady", shooterActive);
        telemetry.addData("ShooterVel", String.format("%.0f", lastShooterVelocityTicks));
        telemetry.update();
    }

    private void handleLaunchCycle() {
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
                stopAndResetDriveEncoders();
                autoState = AutoState.ROTATION2;
                break;
        }
    }

    // ===== Alignment using Tx =====
    private boolean performAlignment() {
        if (!isTxFresh() || Double.isNaN(cachedTx)) {
            // wait for a valid tx but honor timeout
            if (alignTimer.seconds() > ALIGN_TIMEOUT_SECONDS) {
                runDriveOpenLoop(0,0);
                return true; // give up and proceed
            }
            runDriveOpenLoop(0,0);
            return false;
        }
        double tx = cachedTx;
        double error = tx;
        if (Math.abs(error) <= ALIGN_TX_THRESHOLD_DEG) {
            runDriveOpenLoop(0,0);
            return true;
        }
        double cmd = STRAFE_INVERT * ALIGN_KP * error;
        cmd = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, cmd));
        runDriveOpenLoop(-cmd, cmd); // rotate in place
        if (alignTimer.seconds() > ALIGN_TIMEOUT_SECONDS) {
            runDriveOpenLoop(0,0);
            return true;
        }
        return false;
    }

    // ===== Shooter helpers =====
    private void computeShooterFromTy(double ty) {
        if (Double.isNaN(ty)) return;
        double distanceMeters = estimateDistanceFromTy(ty, LIMELIGHT_MOUNT_ANGLE_DEG, inchesToMeters(GOAL_HEIGHT_INCHES), inchesToMeters(CAMERA_HEIGHT_INCHES));
        double distanceInches = metersToInches(distanceMeters);
        double factor = calculatedFactor(LAUNCHER_FACTOR_BASE, distanceInches);
        double requiredVelocity = factor + (VELOCITY_DISTANCE_SCALE * distanceInches);
        double newRPM = (requiredVelocity / (2.0 * Math.PI * WHEEL_RADIUS_INCH)) * 60.0;
        double targetRPM = newRPM * SHOOTER_VELOCITY_SAFETY_MULTIPLIER;
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

    // ===== Vision =====
    private void pollLimelight() {
        if (llcam == null) return;
        try {
            LLResult r = llcam.getLatestResult();
            if (r != null && r.isValid()) {
                double tx = r.getTx();
                double ty = r.getTy();
                if (!Double.isNaN(tx)) {
                    cachedTx = tx;
                    cachedTxNanoTime = System.nanoTime();
                }
                if (!Double.isNaN(ty)) {
                    cachedTy = ty;
                    cachedTyNanoTime = System.nanoTime();
                }
                List<LLResultTypes.FiducialResult> fid = r.getFiducialResults();
                if (fid != null && fid.size() > 0) {
                    for (LLResultTypes.FiducialResult fr : fid) {
                        int id = fr.getFiducialId();
                        if (isAllowedAprilTagId(id)) {
                            double ftx = fr.getTargetXDegrees();
                            double fty = fr.getTargetYDegrees();
                            cachedTx = ftx;
                            cachedTy = fty;
                            long now = System.nanoTime();
                            cachedTxNanoTime = now;
                            cachedTyNanoTime = now;
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

    private boolean isTxFresh() {
        if (cachedTxNanoTime == 0L) return false;
        double ageSec = (System.nanoTime() - cachedTxNanoTime) / 1e9;
        return ageSec <= TX_CACHE_TTL_SECONDS;
    }

    private boolean isTyFresh() {
        if (cachedTyNanoTime == 0L) return false;
        double ageSec = (System.nanoTime() - cachedTyNanoTime) / 1e9;
        return ageSec <= TY_CACHE_TTL_SECONDS;
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

    private void stopAllMotion() {
        runDriveOpenLoop(0.0, 0.0);
        setFeederPower(0.0);
        setIndexerPower(0.0);
        shooter.setVelocity(0.0);
    }

    // drive helpers
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

    // math helpers
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
