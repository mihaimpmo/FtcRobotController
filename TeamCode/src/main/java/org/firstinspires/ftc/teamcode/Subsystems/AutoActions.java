package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Autonomous "puzzle pieces": moves in meters and degrees using GoBilda Pinpoint odometry and
 * {@link SwerveDrive}. This class owns the closed-loop driving loops (no separate driver class).
 * <p>
 * Pose follows Pinpoint after your configuration. Commands move along Pinpoint X/Y in the sensor
 * frame and hold/seek headings in Pinpoint's heading convention.
 */
@Configurable
public final class AutoActions {

    private static final double METERS_TO_INCHES = 39.37007874015748031496062992126;
    private static final double INCHES_TO_METERS = 1.0 / METERS_TO_INCHES;

    // --- Tunables (adjust on robot / via configurables dashboard) ---

    public static double DRIVE_POWER = 1.0;
    public static double TURN_POWER = 1.0;

    /** Heading error (deg) → small omega correction while translating (matches prior 0.03 scale). */
    public static double HEADING_CORRECTION_P = 0.03;

    /** Lateral/longitudinal drift (m) → small orthogonal chassis correction (~same strength as old 0.05/in). */
    public static double CROSS_AXIS_P_PER_M = 0.05 / 0.0254;

    /** Within this distance (m) of goal, linearly ramp from max power to {@link #MIN_POWER}. */
    public static double DECEL_DISTANCE_M = 8.0 * INCHES_TO_METERS;

    /** Within this angle (deg) of turn goal, ramp turn power. */
    public static double DECEL_ANGLE_DEG = 15.0;

    public static double MIN_POWER = 0.15;
    public static long SETTLE_MS = 200;

    /** Turn complete when within this many degrees of target. */
    public static double TURN_TOLERANCE_DEG = 3.0;

    private final SwerveDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final LinearOpMode opMode;

    /** Commanded end pose after each segment; next move is planned from here (reduces drift stacking). */
    private double expectedXM;
    private double expectedYM;
    private double expectedHeadingDeg;

    public AutoActions(SwerveDrive swerve, GoBildaPinpointDriver pinpoint, LinearOpMode opMode) {
        this.drive = swerve;
        this.pinpoint = pinpoint;
        this.opMode = opMode;
        PoseSnapshot p = readPoseSnapshot();
        this.expectedXM = p.xM;
        this.expectedYM = p.yM;
        this.expectedHeadingDeg = p.headingDeg;
    }

    /** Current Pinpoint pose (meters, degrees). */
    public Pose2D getPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public double getXMeters() {
        return getPose().getX(DistanceUnit.METER);
    }

    public double getYMeters() {
        return getPose().getY(DistanceUnit.METER);
    }

    public double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Translate along Pinpoint +X by {@code deltaXMeters} (positive = increasing Pinpoint X).
     * Uses heading and lateral correction while moving; then updates internal expected pose to the target.
     */
    public void forwardMeters(double deltaXMeters) {
        double targetX = expectedXM + deltaXMeters;
        double targetY = expectedYM;
        double targetH = expectedHeadingDeg;

        while (opMode.opModeIsActive()) {
            PoseSnapshot cur = readPoseSnapshot();
            double remaining = targetX - cur.xM;
            if (deltaXMeters >= 0 && remaining <= 0) break;
            if (deltaXMeters < 0 && remaining >= 0) break;

            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE_M);
            double yDrift = cur.yM - targetY;
            double strafeCorrection = CROSS_AXIS_P_PER_M * yDrift;
            double headingError = normalizeAngleDeg(targetH - cur.headingDeg);
            double rotCorrection = HEADING_CORRECTION_P * headingError;

            drive.drive(-power, strafeCorrection, -rotCorrection);
            drive.update();

            opMode.telemetry.addData("auto", "fwd m rem %.3f", remaining);
            opMode.telemetry.addData("drift", "y_m %.4f h_deg %.1f", yDrift, headingError);
            opMode.telemetry.update();
        }
        brakeAndSettle();
        syncExpectedToTarget(targetX, targetY, targetH);
    }

    /**
     * Translate along Pinpoint +Y by {@code deltaYMeters} (positive = increasing Pinpoint Y).
     */
    public void strafeMeters(double deltaYMeters) {
        double targetX = expectedXM;
        double targetY = expectedYM + deltaYMeters;
        double targetH = expectedHeadingDeg;

        while (opMode.opModeIsActive()) {
            PoseSnapshot cur = readPoseSnapshot();
            double remaining = targetY - cur.yM;
            if (deltaYMeters >= 0 && remaining <= 0) break;
            if (deltaYMeters < 0 && remaining >= 0) break;

            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE_M);
            double xDrift = cur.xM - targetX;
            double fwdCorrection = CROSS_AXIS_P_PER_M * xDrift;
            double headingError = normalizeAngleDeg(targetH - cur.headingDeg);
            double rotCorrection = HEADING_CORRECTION_P * headingError;

            drive.drive(fwdCorrection, -power, -rotCorrection);
            drive.update();

            opMode.telemetry.addData("auto", "strafe m rem %.3f", remaining);
            opMode.telemetry.addData("drift", "x_m %.4f h_deg %.1f", xDrift, headingError);
            opMode.telemetry.update();
        }
        brakeAndSettle();
        syncExpectedToTarget(targetX, targetY, targetH);
    }

    /** Rotate in place to absolute heading (degrees), normalized to ±180°. */
    public void turnToHeadingDegrees(double headingDeg) {
        double targetH = normalizeAngleDeg(headingDeg);
        double targetX = expectedXM;
        double targetY = expectedYM;

        while (opMode.opModeIsActive()) {
            PoseSnapshot cur = readPoseSnapshot();
            double error = normalizeAngleDeg(targetH - cur.headingDeg);
            if (Math.abs(error) < TURN_TOLERANCE_DEG) break;

            double power = rampedPower(TURN_POWER, error, DECEL_ANGLE_DEG);
            drive.drive(0, 0, -power);
            drive.update();

            opMode.telemetry.addData("auto", "turn %.1f -> %.1f", cur.headingDeg, targetH);
            opMode.telemetry.update();
        }
        brakeAndSettle();
        syncExpectedToTarget(targetX, targetY, targetH);
    }

    /** Rotate in place by delta (degrees) from current Pinpoint heading. */
    public void turnRelativeDegrees(double deltaHeadingDeg) {
        PoseSnapshot cur = readPoseSnapshot();
        turnToHeadingDegrees(cur.headingDeg + deltaHeadingDeg);
    }

    /** Zero chassis output immediately. */
    public void stopDrive() {
        drive.drive(0, 0, 0);
        drive.update();
    }

    /** Snap expected pose to current odometry (e.g. after vision reset or manual reposition). */
    public void syncExpectedFromPinpoint() {
        PoseSnapshot p = readPoseSnapshot();
        expectedXM = p.xM;
        expectedYM = p.yM;
        expectedHeadingDeg = p.headingDeg;
    }

    public static double inchesToMeters(double inches) {
        return inches * INCHES_TO_METERS;
    }

    public static double metersToInches(double meters) {
        return meters * METERS_TO_INCHES;
    }

    // -------------------------------------------------------------------------

    private PoseSnapshot readPoseSnapshot() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        return new PoseSnapshot(
                p.getX(DistanceUnit.METER),
                p.getY(DistanceUnit.METER),
                p.getHeading(AngleUnit.DEGREES)
        );
    }

    private void syncExpectedToTarget(double targetXM, double targetYM, double targetHeadingDeg) {
        PoseSnapshot cur = readPoseSnapshot();
        opMode.telemetry.addLine("--- auto drift (m, deg) ---");
        opMode.telemetry.addData("x", "%.4f", cur.xM - targetXM);
        opMode.telemetry.addData("y", "%.4f", cur.yM - targetYM);
        opMode.telemetry.addData("h", "%.2f", normalizeAngleDeg(cur.headingDeg - targetHeadingDeg));
        opMode.telemetry.update();

        expectedXM = targetXM;
        expectedYM = targetYM;
        expectedHeadingDeg = targetHeadingDeg;
    }

    private void brakeAndSettle() {
        drive.drive(0, 0, 0);
        drive.update();
        opMode.sleep(SETTLE_MS);
    }

    private static double rampedPower(double maxPower, double remaining, double decelZone) {
        double absRemaining = Math.abs(remaining);
        double power;
        if (absRemaining > decelZone) {
            power = maxPower;
        } else {
            power = MIN_POWER + (maxPower - MIN_POWER) * (absRemaining / decelZone);
        }
        return power * Math.signum(remaining);
    }

    private static double normalizeAngleDeg(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    private static final class PoseSnapshot {
        final double xM;
        final double yM;
        final double headingDeg;

        PoseSnapshot(double xM, double yM, double headingDeg) {
            this.xM = xM;
            this.yM = yM;
            this.headingDeg = headingDeg;
        }
    }
}
