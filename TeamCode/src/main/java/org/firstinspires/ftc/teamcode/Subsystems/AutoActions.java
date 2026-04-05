package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;

/**
 * Closed-loop autonomous helpers using GoBilda Pinpoint for translational and heading feedback.
 * Public distance inputs are in centimeters and heading inputs are in degrees. Commands are
 * expressed relative to the robot, then converted into field-space targets. During the move,
 * position error is converted back into robot-centric commands for the swerve drive.
 */
public class AutoActions {
    private final LinearOpMode opMode;
    private final SwerveDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer = new ElapsedTime();

    private Pose2D expectedPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);

    public AutoActions(LinearOpMode opMode, SwerveDrive drive, GoBildaPinpointDriver pinpoint) {
        this.opMode = opMode;
        this.drive = drive;
        this.pinpoint = pinpoint;
    }

    public boolean homeDrive() {
        return drive.homeAllModules(opMode);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(
                AutoConstants.PINPOINT_X_OFFSET_MM,
                AutoConstants.PINPOINT_Y_OFFSET_MM,
                DistanceUnit.MM
        );
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
        expectedPose = readPose();
    }

    public void setStartPose(double xCm, double yCm, double headingDegrees) {
        Pose2D pose = new Pose2D(DistanceUnit.CM, xCm, yCm, AngleUnit.DEGREES, headingDegrees);
        pinpoint.setPosition(pose);
        expectedPose = pose;
    }

    public Pose2D getPose() {
        return readPose();
    }

    public void forward(double centimeters) {
        moveRelative(centimeters, 0.0);
    }

    public void backward(double centimeters) {
        moveRelative(-centimeters, 0.0);
    }

    public void strafeLeft(double centimeters) {
        moveRelative(0.0, centimeters);
    }

    public void strafeRight(double centimeters) {
        moveRelative(0.0, -centimeters);
    }

    public void turnBy(double deltaDegrees) {
        turnTo(expectedPose.getHeading(AngleUnit.DEGREES) + deltaDegrees);
    }

    public void turnTo(double targetHeadingDegrees) {
        double targetX = expectedPose.getX(DistanceUnit.CM);
        double targetY = expectedPose.getY(DistanceUnit.CM);
        double normalizedTargetHeading = normalizeAngle(targetHeadingDegrees);

        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < AutoConstants.LOOP_TIMEOUT_MS) {
            Pose2D pose = readPose();
            double headingError = normalizeAngle(normalizedTargetHeading - pose.getHeading(AngleUnit.DEGREES));

            if (Math.abs(headingError) <= AutoConstants.HEADING_TOLERANCE_DEG) {
                break;
            }

            double rot = scaledPower(
                    headingError,
                    AutoConstants.TURN_P,
                    AutoConstants.MIN_ROTATION_SPEED,
                    AutoConstants.MAX_ROTATION_SPEED,
                    AutoConstants.DECEL_ANGLE_DEG
            );

            drive.drive(0.0, 0.0, AutoConstants.ROTATION_COMMAND_SIGN * rot);
            drive.update();
            logTurnTelemetry(pose, normalizedTargetHeading, headingError);
            opMode.idle();
        }

        brakeAndSettle();
        expectedPose = new Pose2D(DistanceUnit.CM, targetX, targetY, AngleUnit.DEGREES, normalizedTargetHeading);
        logDrift("Turn", targetX, targetY, normalizedTargetHeading);
    }

    public void stop() {
        drive.drive(0.0, 0.0, 0.0);
        drive.update();
    }

    public void pause(long durationMs) {
        stop();
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < durationMs) {
            pinpoint.update();
            drive.update();
            opMode.idle();
        }
    }

    private void moveRelative(double forwardCm, double strafeLeftCm) {
        double startX = expectedPose.getX(DistanceUnit.CM);
        double startY = expectedPose.getY(DistanceUnit.CM);
        double targetHeading = expectedPose.getHeading(AngleUnit.DEGREES);
        double headingRad = Math.toRadians(targetHeading);

        double targetX = startX
                + (forwardCm * Math.cos(headingRad))
                - (strafeLeftCm * Math.sin(headingRad));
        double targetY = startY
                + (forwardCm * Math.sin(headingRad))
                + (strafeLeftCm * Math.cos(headingRad));

        boolean forwardPrimary = Math.abs(forwardCm) >= Math.abs(strafeLeftCm);
        timer.reset();

        while (opMode.opModeIsActive() && timer.milliseconds() < AutoConstants.LOOP_TIMEOUT_MS) {
            Pose2D pose = readPose();
            double headingDeg = pose.getHeading(AngleUnit.DEGREES);
            double headingNowRad = Math.toRadians(headingDeg);
            double headingError = normalizeAngle(targetHeading - headingDeg);

            double fieldErrorX = targetX - pose.getX(DistanceUnit.CM);
            double fieldErrorY = targetY - pose.getY(DistanceUnit.CM);

            // Convert field-space translation error into robot-space commands.
            double robotForwardError = fieldErrorX * Math.cos(headingNowRad) + fieldErrorY * Math.sin(headingNowRad);
            double robotStrafeError = -fieldErrorX * Math.sin(headingNowRad) + fieldErrorY * Math.cos(headingNowRad);

            double primaryError = forwardPrimary ? robotForwardError : robotStrafeError;
            double crossAxisError = forwardPrimary ? robotStrafeError : robotForwardError;

            boolean positionDone = Math.abs(primaryError) <= AutoConstants.POSITION_TOLERANCE_CM;
            boolean crossAxisDone = Math.abs(crossAxisError) <= AutoConstants.CROSS_AXIS_TOLERANCE_CM;
            boolean headingDone = Math.abs(headingError) <= AutoConstants.HEADING_TOLERANCE_DEG;
            if (positionDone && crossAxisDone && headingDone) {
                break;
            }

            double fwdCmd;
            double strCmd;
            if (forwardPrimary) {
                fwdCmd = scaledPower(
                        robotForwardError,
                        AutoConstants.DRIVE_P,
                        AutoConstants.MIN_TRANSLATION_SPEED,
                        AutoConstants.MAX_TRANSLATION_SPEED,
                        AutoConstants.DECEL_DISTANCE_CM
                );
                strCmd = clamp(
                        robotStrafeError * AutoConstants.CROSS_AXIS_P,
                        -AutoConstants.MAX_TRANSLATION_SPEED,
                        AutoConstants.MAX_TRANSLATION_SPEED
                );
            } else {
                strCmd = scaledPower(
                        robotStrafeError,
                        AutoConstants.STRAFE_P,
                        AutoConstants.MIN_TRANSLATION_SPEED,
                        AutoConstants.MAX_TRANSLATION_SPEED,
                        AutoConstants.DECEL_DISTANCE_CM
                );
                fwdCmd = clamp(
                        robotForwardError * AutoConstants.CROSS_AXIS_P,
                        -AutoConstants.MAX_TRANSLATION_SPEED,
                        AutoConstants.MAX_TRANSLATION_SPEED
                );
            }

            double rotCmd = clamp(
                    headingError * AutoConstants.HEADING_P,
                    -AutoConstants.MAX_ROTATION_SPEED,
                    AutoConstants.MAX_ROTATION_SPEED
            );

            drive.drive(
                    AutoConstants.FORWARD_COMMAND_SIGN * fwdCmd,
                    AutoConstants.STRAFE_COMMAND_SIGN * strCmd,
                    AutoConstants.ROTATION_COMMAND_SIGN * rotCmd
            );
            drive.update();
            logMoveTelemetry(pose, targetX, targetY, targetHeading, robotForwardError, robotStrafeError, headingError);
            opMode.idle();
        }

        brakeAndSettle();
        expectedPose = new Pose2D(DistanceUnit.CM, targetX, targetY, AngleUnit.DEGREES, targetHeading);
        logDrift("Move", targetX, targetY, targetHeading);
    }

    private Pose2D readPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    private void brakeAndSettle() {
        stop();
        opMode.sleep(AutoConstants.SETTLE_MS);
    }

    private void logMoveTelemetry(
            Pose2D pose,
            double targetX,
            double targetY,
            double targetHeading,
            double robotForwardError,
            double robotStrafeError,
            double headingError
    ) {
        opMode.telemetry.addData("Target", "X %.1f  Y %.1f  H %.1f", targetX, targetY, targetHeading);
        opMode.telemetry.addData(
                "Pose",
                "X %.1f  Y %.1f  H %.1f",
                pose.getX(DistanceUnit.CM),
                pose.getY(DistanceUnit.CM),
                pose.getHeading(AngleUnit.DEGREES)
        );
        opMode.telemetry.addData("Robot Error", "F %.2f  S %.2f  H %.1f", robotForwardError, robotStrafeError, headingError);
        opMode.telemetry.update();
    }

    private void logTurnTelemetry(Pose2D pose, double targetHeading, double headingError) {
        opMode.telemetry.addData("Turn Target", "%.1f", targetHeading);
        opMode.telemetry.addData("Turn Pose", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        opMode.telemetry.addData("Turn Error", "%.2f", headingError);
        opMode.telemetry.update();
    }

    private void logDrift(String label, double targetX, double targetY, double targetHeading) {
        Pose2D currentPose = readPose();
        double errorX = currentPose.getX(DistanceUnit.CM) - targetX;
        double errorY = currentPose.getY(DistanceUnit.CM) - targetY;
        double errorHeading = normalizeAngle(currentPose.getHeading(AngleUnit.DEGREES) - targetHeading);

        opMode.telemetry.addLine(label + " drift");
        opMode.telemetry.addData("X Error", "%.2f cm", errorX);
        opMode.telemetry.addData("Y Error", "%.2f cm", errorY);
        opMode.telemetry.addData("H Error", "%.1f deg", errorHeading);
        opMode.telemetry.update();
    }

    private double scaledPower(double error, double p, double minPower, double maxPower, double decelWindow) {
        double proportional = error * p;
        double ramp = maxPower;
        double absError = Math.abs(error);
        if (absError < decelWindow) {
            ramp = minPower + (maxPower - minPower) * (absError / decelWindow);
        }

        double limited = clamp(proportional, -ramp, ramp);
        if (Math.abs(limited) < minPower) {
            return minPower * Math.signum(error);
        }
        return limited;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double normalizeAngle(double degrees) {
        degrees %= 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
}
