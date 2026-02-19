package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Pedro.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Pedro.SwerveFollowerConstants;
import org.firstinspires.ftc.teamcode.Pedro.SwervePinpointConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

/**
 * Simple teleop for testing Pinpoint odometry via Pedro's Follower.
 *
 * Drive around or push the robot by hand — X, Y, heading shown on Panels.
 * Press A to reset pose to (0, 0, 0).
 */
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);
        SwerveDrive drive = drivetrain.getSwerveDrive();

        Follower follower = new FollowerBuilder(SwerveFollowerConstants.followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(SwervePinpointConstants.localizerConstants)
                .build();

        follower.setStartingPose(new Pose(0, 0, 0));

        panels.debug("Odometry Test Ready", "Press START to home and begin");
        panels.update(telemetry);

        waitForStart();

        // Home swerve modules
        panels.debug("Homing...");
        panels.update(telemetry);
        drive.homeAllModules(this);

        // Start teleop drive mode so follower doesn't try to follow a path
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // Drive with gamepad
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            if (Math.abs(forward) < ControlConstants.DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.DEADBAND) rotation = 0;

            follower.setTeleOpDrive(forward, strafe, rotation, true);
            follower.update();

            // Reset pose on A
            if (gamepad1.a) {
                follower.setPose(new Pose(0, 0, 0));
            }

            // Read pose and velocity from follower (backed by Pinpoint)
            Pose pose = follower.getPose();
            Vector velocity = follower.getVelocity();

            panels.debug(
                    "--- POSE ---",
                    String.format("X: %.2f in", pose.getX()),
                    String.format("Y: %.2f in", pose.getY()),
                    String.format("Heading: %.2f deg", Math.toDegrees(pose.getHeading())),
                    "",
                    "--- VELOCITY ---",
                    String.format("Speed: %.1f in/s", velocity.getMagnitude()),
                    String.format("Direction: %.1f deg", Math.toDegrees(velocity.getTheta())),
                    "",
                    "Press A to reset pose"
            );
            panels.update(telemetry);
        }
    }
}
