package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


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

    private final SwerveDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final LinearOpMode opMode;

    public AutoDrive(SwerveDrive drive, GoBildaPinpointDriver pinpoint, LinearOpMode opMode) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.opMode = opMode;
    }

    private Pose2D readPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }


    public void resetFilter() {
        // No-op, filter removed
    }


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

    public void forward(double inches) {
        Pose2D pose = readPose();
        double startX = pose.getX(DistanceUnit.INCH);
        double startY = pose.getY(DistanceUnit.INCH);
        double targetX = startX + inches;

        while (opMode.opModeIsActive()) {
            pose = readPose();
            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);

            double remaining = targetX - currentX;
            if (inches >= 0 && remaining <= 0) break;
            if (inches < 0 && remaining >= 0) break;

            // Master: Forward power (-power for forward progress)
            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE);

            // Lock: Lateral. If we drift left (currentY > startY), move right (+strafe).
            double yDrift = currentY - startY;
            double strafeCorrection = CROSS_AXIS_P * yDrift;

            // drive(fwd, str, rot)
            drive.drive(-power, strafeCorrection, 0);
            drive.update();

            opMode.telemetry.addData("Forward", "%.1f / %.1f in", currentX - startX, inches);
            opMode.telemetry.addData("Lock", "Y-Drift: %.2f", yDrift);
            opMode.telemetry.update();
        }
        brakeAndSettle();
    }

    public void strafe(double inches) {
        Pose2D pose = readPose();
        double startX = pose.getX(DistanceUnit.INCH);
        double startY = pose.getY(DistanceUnit.INCH);
        double targetY = startY + inches;

        while (opMode.opModeIsActive()) {
            pose = readPose();
            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);

            double remaining = targetY - currentY;
            if (inches >= 0 && remaining <= 0) break;
            if (inches < 0 && remaining >= 0) break;

            // Master: Strafe power (-power for left progress)
            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE);

            // Lock: Longitudinal. If we creep forward (currentX > startX), move backward (+fwd).
            double xDrift = currentX - startX;
            double fwdCorrection = CROSS_AXIS_P * xDrift;

            // drive(fwd, str, rot)
            drive.drive(fwdCorrection, -power, 0);
            drive.update();

            opMode.telemetry.addData("Strafe", "%.1f / %.1f in", currentY - startY, inches);
            opMode.telemetry.addData("Lock", "X-Drift: %.2f", xDrift);
            opMode.telemetry.update();
        }
        brakeAndSettle();
    }

    public void turnTo(double targetHeading) {
        while (opMode.opModeIsActive()) {
            Pose2D pose = readPose();
            double heading = pose.getHeading(AngleUnit.DEGREES);
            double error = normalizeAngle(targetHeading - heading);

            if (Math.abs(error) < 3.0) break;

            double power = rampedPower(TURN_POWER, error, DECEL_ANGLE);

            // ONLY change the Rotation axis
            // Inverted rotation command to match user's physical setup
            drive.drive(0, 0, -power);
            drive.update();

            opMode.telemetry.addData("Turn", "%.1f° → %.1f°", heading, targetHeading);
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
