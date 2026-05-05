package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;
@Configurable
public class AutoHandler {
    private final SwerveDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final LinearOpMode opMode;
    private final Telemetry telemetry;

    public static double kP = 0.01;
    public static double kD = 0.01;
    public static double kHeadingP = 1.5;
    public static double kLateralP = 0.06;
    public static double minDrivePower = 0.05;
    public static double minStrafePower = 0.05;
    public static double posToleranceCm = 0.5;
    public static double headingToleranceDeg = 1.0;
    public static double loopTimeoutMs = 6000;
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;

    public AutoHandler(SwerveDrive drive, GoBildaPinpointDriver pinpoint, LinearOpMode opMode, Telemetry telemetry) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.opMode = opMode;
        this.telemetry = telemetry;
    }

    public void set0() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void moveForward(double distanceCm) {
        moveForwardInternal(distanceCm, false);
    }

    public void moveForwardCorrected(double distanceCm) {
        moveForwardInternal(distanceCm, true);
    }

    private void moveForwardInternal(double distanceCm, boolean correctLateralDrift) {
        telemetry.addData("Moving forward", "%.2f cm", distanceCm);
        pinpointData(telemetry);
        double lastError = 0;
        long startTimeMs = System.currentTimeMillis();

        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();
        double x0 = startPose.getX(DistanceUnit.CM);
        double y0 = startPose.getY(DistanceUnit.CM);
        double heading0 = startPose.getHeading(AngleUnit.RADIANS);

        while (opMode.opModeIsActive()) {
            if (System.currentTimeMillis() - startTimeMs > loopTimeoutMs) {
                break;
            }

            pinpoint.update();
            Pose2D current = pinpoint.getPosition();

            double x = current.getX(DistanceUnit.CM);
            double y = current.getY(DistanceUnit.CM);
            double heading = current.getHeading(AngleUnit.RADIANS);

            double distanceTraveled = x - x0;
            double error = distanceCm - distanceTraveled;
            if (Math.abs(error) < posToleranceCm) {
                break;
            }
            double derivative = error - lastError;
            double fwdPower = (kP * error) + (kD * derivative);
            fwdPower = clampWithMin(fwdPower, minDrivePower);

            double strafePower = 0;
            if (correctLateralDrift) {
                double yError = y0 - y;
                strafePower = clampWithMin(kLateralP * yError, minStrafePower);
            }

            double headingError = MathUtils.normalizeAngle(heading0 - heading);
            double rot = kHeadingP * headingError;
            rot = Math.max(-0.5, Math.min(0.5, rot));

            drive.drive(fwdPower, strafePower, rot);
            drive.update();

            lastError = error;
            opMode.idle();
        }

        drive.drive(0, 0, 0);
        drive.update();
    }

    public void moveRight(double distanceCm) {
        telemetry.addData("Moving right", "%.2f cm", distanceCm);
        pinpointData(telemetry);
        double lastError = 0;
        long startTimeMs = System.currentTimeMillis();

        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();
        double y0 = startPose.getY(DistanceUnit.CM);

        while (opMode.opModeIsActive()) {
            if (System.currentTimeMillis() - startTimeMs > loopTimeoutMs) {
                break;
            }

            pinpoint.update();
            Pose2D current = pinpoint.getPosition();
            double y = current.getY(DistanceUnit.CM);
            double distanceTraveled = y - y0;
            double error = distanceCm - distanceTraveled;
            if (Math.abs(error) < posToleranceCm) {
                break;
            }
            double derivative = error - lastError;
            double power = (kP * error) + (kD * derivative);
            power = clampWithMin(power, minStrafePower);

            drive.drive(0, power, 0);
            drive.update();

            lastError = error;
            opMode.idle();
        }

        drive.drive(0, 0, 0);
        drive.update();
    }

    public void moveAtAngle(double distanceCm, double angleRad) {
        telemetry.addData("Moving at angle", "%.2f cm, %.1f deg", distanceCm, Math.toDegrees(angleRad));
        pinpointData(telemetry);
        double lastError = 0;
        long startTimeMs = System.currentTimeMillis();

        pinpoint.update();
        Pose2D startPose = pinpoint.getPosition();
        double x0 = startPose.getX(DistanceUnit.CM);
        double y0 = startPose.getY(DistanceUnit.CM);

        double dirX = Math.cos(angleRad);
        double dirY = Math.sin(angleRad);

        while (opMode.opModeIsActive()) {
            if (System.currentTimeMillis() - startTimeMs > loopTimeoutMs) {
                break;
            }

            pinpoint.update();
            Pose2D current = pinpoint.getPosition();
            double x = current.getX(DistanceUnit.CM);
            double y = current.getY(DistanceUnit.CM);
            double dx = x - x0;
            double dy = y - y0;
            double distanceTraveled = dx * dirX + dy * dirY;
            double error = distanceCm - distanceTraveled;
            if (Math.abs(error) < posToleranceCm) {
                break;
            }
            double derivative = error - lastError;
            double power = (kP * error) + (kD * derivative);
            power = clampWithMin(power, minDrivePower);

            double fwd = power * dirX;
            double str = power * dirY;
            drive.drive(fwd, str, 0);
            drive.update();

            lastError = error;
            opMode.idle();
        }

        drive.drive(0, 0, 0);
        drive.update();
    }

    public void rotate(double degrees) {
        telemetry.addData("Rotating", "%.1f deg", degrees);
        pinpointData(telemetry);
        long startTimeMs = System.currentTimeMillis();
        pinpoint.update();
        double startHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        double targetHeading = startHeading + Math.toRadians(degrees);

        while (opMode.opModeIsActive()) {
            if (System.currentTimeMillis() - startTimeMs > loopTimeoutMs) {
                break;
            }

            pinpoint.update();
            double currentHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
            double error = MathUtils.normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(Math.toDegrees(error)) < headingToleranceDeg) {
                break;
            }

            double rotPower = kHeadingP * error;
            rotPower = clampWithMin(rotPower, minStrafePower);

            drive.drive(0, 0, rotPower);
            drive.update();

            opMode.idle();
        }

        drive.drive(0, 0, 0);
        drive.update();
    }

    private double clampWithMin(double power, double minAbs) {
        power = Math.max(-1, Math.min(1, power));
        if (power != 0 && Math.abs(power) < minAbs) {
            return minAbs * Math.signum(power);
        }
        return power;
    }

    public void pinpointData(Telemetry telemetry) {
        pinpoint.update();
        telemetry.addData("Pose X(cm)", pinpoint.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("Pose Y(cm)", pinpoint.getPosition().getY(DistanceUnit.CM));
        telemetry.addData("Pose heading(deg)", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

}
