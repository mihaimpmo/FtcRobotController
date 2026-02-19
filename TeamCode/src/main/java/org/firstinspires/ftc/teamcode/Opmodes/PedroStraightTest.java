package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Pedro.SwerveFollowerConstants;
import org.firstinspires.ftc.teamcode.Pedro.SwervePinpointConstants;

/**
 * Tuning OpMode: drives the robot 24 inches forward in a straight line.
 *
 * Use this to verify:
 * - Pinpoint reads correct X/Y/heading when pushed by hand
 * - Robot drives forward (not sideways) — if wrong, toggle FLIP_X / FLIP_Y in SwerveDrivetrain
 * - Robot stops within ~1 inch of target
 * - Drive/translational/heading PIDF values
 */
@Autonomous(name = "Pedro Straight Test", group = "Test")
public class PedroStraightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);

        Follower follower = new FollowerBuilder(SwerveFollowerConstants.followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(SwervePinpointConstants.localizerConstants)
                .build();

        Pose startPose = new Pose(0, 0, 0);
        Pose endPose = new Pose(24, 0, 0);

        follower.setStartingPose(startPose);

        PathChain straightPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(0)
                .build();

        telemetry.addLine("Pedro Straight Test Ready");
        telemetry.addLine("Press START to home modules, then drive 24in forward");
        telemetry.update();

        waitForStart();

        // Home swerve modules
        telemetry.addLine("Homing modules...");
        telemetry.update();
        boolean homed = drivetrain.getSwerveDrive().homeAllModules(this);
        if (!homed) {
            telemetry.addLine("WARNING: Homing failed!");
            telemetry.update();
            sleep(2000);
        }

        // Follow the straight path
        follower.followPath(straightPath, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            Pose pose = follower.getPose();
            telemetry.addData("X", "%.2f in", pose.getX());
            telemetry.addData("Y", "%.2f in", pose.getY());
            telemetry.addData("Heading", "%.2f deg", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Busy", follower.isBusy());
            drivetrain.getSwerveDrive().log(telemetry);
            telemetry.update();
        }

        // Hold end position
        telemetry.addLine("--- Path Complete ---");
        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();
            telemetry.addData("Final X", "%.2f in", pose.getX());
            telemetry.addData("Final Y", "%.2f in", pose.getY());
            telemetry.addData("Final Heading", "%.2f deg", Math.toDegrees(pose.getHeading()));
            drivetrain.getSwerveDrive().log(telemetry);
            telemetry.update();
        }
    }
}
