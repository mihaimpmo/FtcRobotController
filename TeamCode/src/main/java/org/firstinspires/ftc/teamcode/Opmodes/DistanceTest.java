package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Distance Test", group = "Test")
public class DistanceTest extends LinearOpMode {

    private static final double HEADING_HOLD_P = 0.15;
    private static final double CROSS_AXIS_P = 0.1; // correct perpendicular drift

    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(6.64, 6.34, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Distance Test Ready");
        telemetry.addLine("Press START to home modules and begin");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing swerve modules...");
        telemetry.update();

        boolean homingSuccess = drive.homeAllModules(this);
        if (!homingSuccess) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }

        // Reset pose after homing
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();
        double holdHeading = 0;
        double holdX = 0; // locked strafe position when driving forward
        double holdY = 0; // locked forward position when strafing

        boolean lastCross = false;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double poseX = pose.getX(DistanceUnit.INCH);
            double poseY = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            // A/Cross = reset pose
            boolean cross = gamepad1.cross;
            if (cross && !lastCross) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                holdHeading = 0;
            }
            lastCross = cross;

            // Right stick X adjusts held heading
            double headingAdjust = gamepad1.right_stick_x;
            if (Math.abs(headingAdjust) > ControlConstants.DEADBAND) {
                holdHeading += headingAdjust * 1.5; // degrees per loop
            }

            // Heading hold correction
            double headingError = normalizeAngle(holdHeading - heading);
            double headingCorrection = -(HEADING_HOLD_P * headingError);

            // Drive inputs
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;

            if (Math.abs(forward) < ControlConstants.DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.DEADBAND) strafe = 0;

            drive.drive(forward * 5.0, strafe * 5.0, headingCorrection);
            drive.update();

            // Telemetry
            double displacement = Math.sqrt(poseX * poseX + poseY * poseY);

            telemetry.addLine("=== DISTANCE TEST ===");
            telemetry.addData("Pinpoint X (strafe)", "%.2f in", poseX);
            telemetry.addData("Pinpoint Y (forward)", "%.2f in", poseY);
            telemetry.addData("Heading", "%.1f° (hold %.1f°)", heading, holdHeading);
            telemetry.addLine("---");
            telemetry.addData("Forward dist", "%.2f in", poseY);
            telemetry.addData("Strafe dist", "%.2f in", poseX);
            telemetry.addData("Total displacement", "%.2f in", displacement);
            telemetry.addLine("---");
            telemetry.addData("Controls", "LStick=drive | RStickX=adjust heading | A=reset");
            telemetry.update();
        }
    }

    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }
}
