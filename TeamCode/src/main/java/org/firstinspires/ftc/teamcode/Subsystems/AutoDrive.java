package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Dead-simple auto drive. Blocking methods — call forward(24) and it goes 24 inches.
 *
 * Axis mapping (pods swapped to correct heading):
 *   Forward distance = Pinpoint X
 *   Strafe distance  = Pinpoint Y
 *   drive(negative, 0, 0) = physical forward
 *   drive(0, 0, positive) = physical CW
 */
@Configurable
public class AutoDrive {

    public static double DRIVE_POWER = 0.8;
    public static double TURN_POWER = 0.8;
    public static double HEADING_CORRECTION_P = 0.2;
    public static double CROSS_AXIS_P = 0.05;

    // Deceleration: start ramping down power within this distance of target
    public static double DECEL_DISTANCE = 8.0;    // inches
    public static double DECEL_ANGLE = 15.0;       // degrees
    public static double MIN_POWER = 0.15;         // minimum power to keep moving

    // Settle: wait this long after stopping to let momentum die
    public static long SETTLE_MS = 200;

    // EMA filter on position only (not heading — heading needs instant response for P control)
    // 0.0 = full smoothing, 1.0 = no filter
    public static double EMA_ALPHA = 0.4;

    private final SwerveDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final LinearOpMode opMode;

    private double filteredX = Double.NaN;
    private double filteredY = Double.NaN;

    public AutoDrive(SwerveDrive drive, GoBildaPinpointDriver pinpoint, LinearOpMode opMode) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.opMode = opMode;
    }

    /** Read pose with EMA filtering on position only. Heading is raw for responsive correction. */
    private Pose2D readPose() {
        pinpoint.update();
        Pose2D raw = pinpoint.getPosition();
        double rawX = raw.getX(DistanceUnit.INCH);
        double rawY = raw.getY(DistanceUnit.INCH);

        if (Double.isNaN(filteredX)) {
            filteredX = rawX;
            filteredY = rawY;
        } else {
            filteredX = EMA_ALPHA * rawX + (1 - EMA_ALPHA) * filteredX;
            filteredY = EMA_ALPHA * rawY + (1 - EMA_ALPHA) * filteredY;
        }

        return new Pose2D(DistanceUnit.INCH, filteredX, filteredY,
                AngleUnit.DEGREES, raw.getHeading(AngleUnit.DEGREES));
    }

    /** Reset EMA filter (call after resetting pinpoint pose). */
    public void resetFilter() {
        filteredX = Double.NaN;
        filteredY = Double.NaN;
    }

    /**
     * Compute ramped power: full power far from target, ramps down linearly
     * within DECEL_DISTANCE, but never below MIN_POWER.
     */
    private double rampedPower(double maxPower, double remaining, double decelZone) {
        double absRemaining = Math.abs(remaining);
        double power;
        if (absRemaining > decelZone) {
            power = maxPower;
        } else {
            // Linear ramp: at decelZone → maxPower, at 0 → MIN_POWER
            power = MIN_POWER + (maxPower - MIN_POWER) * (absRemaining / decelZone);
        }
        return power * Math.signum(remaining);
    }

    /**
     * Drive forward by the given distance in inches. Maintains heading.
     * Positive = forward, negative = backward.
     */
    public void forward(double inches) {
        Pose2D pose = readPose();
        double startPos = pose.getX(DistanceUnit.INCH);
        double targetPos = startPos + inches;
        double holdHeading = pose.getHeading(AngleUnit.DEGREES);
        double holdY = pose.getY(DistanceUnit.INCH);

        while (opMode.opModeIsActive()) {
            pose = readPose();
            double current = pose.getX(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double currentY = pose.getY(DistanceUnit.INCH);

            double remaining = targetPos - current;

            // Check if we've passed the target
            if (inches >= 0 && remaining <= 0) break;
            if (inches < 0 && remaining >= 0) break;

            // Ramped power: slows down near target
            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE);

            // Heading correction
            double headingError = normalizeAngle(holdHeading - heading);
            double headingCorrection = -(HEADING_CORRECTION_P * headingError);

            // Cross-axis correction: hold Y (strafe) constant
            double yDrift = holdY - currentY;
            double strafeCorrection = CROSS_AXIS_P * yDrift;

            // drive(negative) = forward
            drive.drive(-power, strafeCorrection, headingCorrection);
            drive.update();

            opMode.telemetry.addData("Forward", "%.1f / %.1f in (rem %.1f)", current - startPos, inches, remaining);
            opMode.telemetry.addData("Power", "%.2f", power);
            opMode.telemetry.addData("Heading", "%.1f° (hold %.1f°)", heading, holdHeading);
            opMode.telemetry.addData("Y drift", "%.2f in", currentY - holdY);
            opMode.telemetry.update();
        }

        brakeAndSettle();
    }

    /**
     * Strafe by the given distance in inches. Maintains heading.
     * Positive = right, negative = left.
     */
    public void strafe(double inches) {
        Pose2D pose = readPose();
        double startPos = pose.getY(DistanceUnit.INCH);
        double targetPos = startPos + inches;
        double holdHeading = pose.getHeading(AngleUnit.DEGREES);
        double holdX = pose.getX(DistanceUnit.INCH);

        while (opMode.opModeIsActive()) {
            pose = readPose();
            double current = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double currentX = pose.getX(DistanceUnit.INCH);

            double remaining = targetPos - current;

            if (inches >= 0 && remaining <= 0) break;
            if (inches < 0 && remaining >= 0) break;

            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE);

            double headingError = normalizeAngle(holdHeading - heading);
            double headingCorrection = -(HEADING_CORRECTION_P * headingError);

            // Cross-axis correction: hold X (forward) constant
            double xDrift = holdX - currentX;
            double fwdCorrection = -(CROSS_AXIS_P * xDrift); // negative because drive(negative) = forward

            drive.drive(fwdCorrection, power, headingCorrection);
            drive.update();

            opMode.telemetry.addData("Strafe", "%.1f / %.1f in (rem %.1f)", current - startPos, inches, remaining);
            opMode.telemetry.addData("Power", "%.2f", power);
            opMode.telemetry.addData("Heading", "%.1f° (hold %.1f°)", heading, holdHeading);
            opMode.telemetry.addData("X drift", "%.2f in", currentX - holdX);
            opMode.telemetry.update();
        }

        brakeAndSettle();
    }

    /**
     * Turn to an absolute heading in degrees.
     * Ramps down power near target to avoid overshoot.
     */
    public void turnTo(double targetHeading) {
        while (opMode.opModeIsActive()) {
            Pose2D pose = readPose();
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double error = normalizeAngle(targetHeading - heading);

            if (Math.abs(error) < 3.0) break;

            // Ramped turn power: full far away, ramps down within DECEL_ANGLE
            double power = rampedPower(TURN_POWER, error, DECEL_ANGLE);

            // error positive = need CCW = need negative rot (drive +rot = CW)
            drive.drive(0, 0, -power);
            drive.update();

            opMode.telemetry.addData("Turn", "%.1f° → %.1f° (err %.1f°)", heading, targetHeading, error);
            opMode.telemetry.addData("Power", "%.2f", power);
            opMode.telemetry.update();
        }

        brakeAndSettle();
    }

    /** Stop motors and wait for robot to physically stop moving. */
    private void brakeAndSettle() {
        drive.drive(0, 0, 0);
        drive.update();
        opMode.sleep(SETTLE_MS);
    }

    public void stop() {
        drive.drive(0, 0, 0);
        drive.update();
    }

    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }
}
