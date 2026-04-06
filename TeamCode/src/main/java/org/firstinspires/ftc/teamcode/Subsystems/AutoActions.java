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
 *
 * Coordinate assumptions:
 * - Heading is in degrees, CCW positive.
 * - At heading 0 deg, robot forward aligns with +X and robot left aligns with +Y.
 * - drive(fwd, str, rot) expects robot-centric commands:
 *      fwd = forward, str = left, rot = CCW positive
 *
 * Public distance inputs are centimeters and heading inputs are degrees.
 */
public class AutoActions {
    private final LinearOpMode opMode;
    private final SwerveDrive drive;
    private final GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer = new ElapsedTime();

    // Best-known pose. Updated from localization after each action.
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
        Pose2D pose = new Pose2D(
                DistanceUnit.CM,
                xCm,
                yCm,
                AngleUnit.DEGREES,
                normalizeAngle(headingDegrees)
        );
        pinpoint.setPosition(pose);
        expectedPose = pose;
    }

    public Pose2D getPose() {
        return readPose();
    }

    public boolean forward(double centimeters) {
        return moveRelative(centimeters, 0.0);
    }

    public boolean backward(double centimeters) {
        return moveRelative(-centimeters, 0.0);
    }

    public boolean strafeLeft(double centimeters) {
        return moveRelative(0.0, centimeters);
    }

    public boolean strafeRight(double centimeters) {
        return moveRelative(0.0, -centimeters);
    }

    public boolean turnBy(double deltaDegrees) {
        Pose2D pose = readPose();
        return turnTo(pose.getHeading(AngleUnit.DEGREES) + deltaDegrees);
    }

    public boolean turnTo(double targetHeadingDegrees) {
        Pose2D startPose = readPose();
        double holdX = startPose.getX(DistanceUnit.CM);
        double holdY = startPose.getY(DistanceUnit.CM);
        double targetHeading = normalizeAngle(targetHeadingDegrees);

        boolean success = false;
        timer.reset();

        while (opMode.opModeIsActive() && timer.milliseconds() < AutoConstants.LOOP_TIMEOUT_MS) {
            Pose2D pose = readPose();
            double headingNow = pose.getHeading(AngleUnit.DEGREES);
            double headingError = normalizeAngle(targetHeading - headingNow);

            if (Math.abs(headingError) <= AutoConstants.HEADING_TOLERANCE_DEG) {
                success = true;
                break;
            }

            double rotCmd = scaledPower(
                    headingError,
                    AutoConstants.TURN_P,
                    AutoConstants.MIN_ROTATION_SPEED,
                    AutoConstants.MAX_ROTATION_SPEED,
                    AutoConstants.DECEL_ANGLE_DEG
            );

            drive.drive(
                    0.0,
                    0.0,
                    AutoConstants.ROTATION_COMMAND_SIGN * rotCmd
            );
            drive.update();

            logTurnTelemetry(pose, targetHeading, headingError);
            opMode.idle();
        }

        brakeAndSettle();
        expectedPose = readPose();
        logDrift("Turn", holdX, holdY, targetHeading);

        return success;
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

        expectedPose = readPose();
    }

    private boolean moveRelative(double forwardCm, double strafeLeftCm) {
        Pose2D startPose = readPose();

        double startX = startPose.getX(DistanceUnit.CM);
        double startY = startPose.getY(DistanceUnit.CM);
        double targetHeading = normalizeAngle(startPose.getHeading(AngleUnit.DEGREES));
        double headingRad = Math.toRadians(targetHeading);

        // Convert robot-relative requested motion into a field-space target.
        double targetX = startX
                + (forwardCm * Math.cos(headingRad))
                - (strafeLeftCm * Math.sin(headingRad));
        double targetY = startY
                + (forwardCm * Math.sin(headingRad))
                + (strafeLeftCm * Math.cos(headingRad));

        boolean success = false;
        timer.reset();

        while (opMode.opModeIsActive() && timer.milliseconds() < AutoConstants.LOOP_TIMEOUT_MS) {
            Pose2D pose = readPose();

            double xNow = pose.getX(DistanceUnit.CM);
            double yNow = pose.getY(DistanceUnit.CM);
            double headingNowDeg = pose.getHeading(AngleUnit.DEGREES);
            double headingNowRad = Math.toRadians(headingNowDeg);

            double fieldErrorX = targetX - xNow;
            double fieldErrorY = targetY - yNow;
            double headingError = normalizeAngle(targetHeading - headingNowDeg);

            // Convert field-space translation error into robot-space commands.
            double robotForwardError =
                    fieldErrorX * Math.cos(headingNowRad) + fieldErrorY * Math.sin(headingNowRad);
            double robotStrafeError =
                    -fieldErrorX * Math.sin(headingNowRad) + fieldErrorY * Math.cos(headingNowRad);

            boolean forwardDone = Math.abs(robotForwardError) <= AutoConstants.POSITION_TOLERANCE_CM;
            boolean strafeDone = Math.abs(robotStrafeError) <= AutoConstants.CROSS_AXIS_TOLERANCE_CM;
            boolean headingDone = Math.abs(headingError) <= AutoConstants.HEADING_TOLERANCE_DEG;

            if (forwardDone && strafeDone && headingDone) {
                success = true;
                break;
            }

            double fwdCmd = scaledPower(
                    robotForwardError,
                    AutoConstants.DRIVE_P,
                    AutoConstants.MIN_TRANSLATION_SPEED,
                    AutoConstants.MAX_TRANSLATION_SPEED,
                    AutoConstants.DECEL_DISTANCE_CM
            );

            double strCmd = scaledPower(
                    robotStrafeError,
                    AutoConstants.STRAFE_P,
                    AutoConstants.MIN_TRANSLATION_SPEED,
                    AutoConstants.MAX_TRANSLATION_SPEED,
                    AutoConstants.DECEL_DISTANCE_CM
            );

            double rotCmd = scaledPower(
                    headingError,
                    AutoConstants.HEADING_P,
                    AutoConstants.MIN_ROTATION_SPEED,
                    AutoConstants.MAX_ROTATION_SPEED,
                    AutoConstants.DECEL_ANGLE_DEG
            );

            drive.drive(
                    AutoConstants.FORWARD_COMMAND_SIGN * fwdCmd,
                    AutoConstants.STRAFE_COMMAND_SIGN * strCmd,
                    AutoConstants.ROTATION_COMMAND_SIGN * rotCmd
            );
            drive.update();

            logMoveTelemetry(
                    pose,
                    targetX,
                    targetY,
                    targetHeading,
                    robotForwardError,
                    robotStrafeError,
                    headingError
            );

            opMode.idle();
        }

        brakeAndSettle();
        expectedPose = readPose();
        logDrift("Move", targetX, targetY, targetHeading);

        return success;
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
        opMode.telemetry.addData(
                "Robot Error",
                "F %.2f  S %.2f  H %.1f",
                robotForwardError,
                robotStrafeError,
                headingError
        );
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
        double errorHeading = normalizeAngle(
                currentPose.getHeading(AngleUnit.DEGREES) - targetHeading
        );

        opMode.telemetry.addLine(label + " drift");
        opMode.telemetry.addData("X Error", "%.2f cm", errorX);
        opMode.telemetry.addData("Y Error", "%.2f cm", errorY);
        opMode.telemetry.addData("H Error", "%.1f deg", errorHeading);
        opMode.telemetry.update();
    }

    private double scaledPower(double error, double p, double minPower, double maxPower, double decelWindow) {
        if (Math.abs(error) < 1e-6) {
            return 0.0;
        }

        double proportional = error * p;

        double rampLimit = maxPower;
        if (decelWindow > 1e-6 && Math.abs(error) < decelWindow) {
            rampLimit = minPower + (maxPower - minPower) * (Math.abs(error) / decelWindow);
        }

        double limited = clamp(proportional, -rampLimit, rampLimit);

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