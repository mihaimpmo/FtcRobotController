package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pedro.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Pedro.SwerveFollowerConstants;
import org.firstinspires.ftc.teamcode.Pedro.SwervePinpointConstants;

/**
 * Sample autonomous skeleton demonstrating multi-segment paths with heading changes.
 * Replace poses and add subsystem actions (outtake/intake) between path segments.
 */
@Autonomous(name = "Swerve Auto", group = "Auto")
public class SwerveAuto extends LinearOpMode {

    private enum AutoState {
        HOMING,
        DRIVE_TO_SCORE,
        SCORING,
        DRIVE_TO_PICKUP,
        DONE
    }

    @Override
    public void runOpMode() {
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);

        Follower follower = new FollowerBuilder(SwerveFollowerConstants.followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(SwervePinpointConstants.localizerConstants)
                .build();

        // Define waypoints — adjust to your field positions
        Pose startPose = new Pose(0, 0, 0);
        Pose scorePose = new Pose(36, 0, Math.toRadians(90));
        Pose pickupControl = new Pose(48, 24);
        Pose pickupPose = new Pose(36, 48, Math.toRadians(180));

        follower.setStartingPose(startPose);

        // Path 1: straight line to scoring position with heading turn
        PathChain toScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // Path 2: curve to pickup with constant heading
        PathChain toPickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickupControl, pickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose.getHeading())
                .build();

        telemetry.addLine("Swerve Auto Ready");
        telemetry.update();

        waitForStart();

        AutoState state = AutoState.HOMING;

        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case HOMING:
                    telemetry.addLine("Homing...");
                    drivetrain.getSwerveDrive().homeAllModules(this);
                    follower.followPath(toScore, true);
                    state = AutoState.DRIVE_TO_SCORE;
                    break;

                case DRIVE_TO_SCORE:
                    if (!follower.isBusy()) {
                        // TODO: trigger scoring action here
                        state = AutoState.SCORING;
                    }
                    break;

                case SCORING:
                    // TODO: wait for scoring to complete, then continue
                    sleep(500);
                    follower.followPath(toPickup, true);
                    state = AutoState.DRIVE_TO_PICKUP;
                    break;

                case DRIVE_TO_PICKUP:
                    if (!follower.isBusy()) {
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    break;
            }

            Pose pose = follower.getPose();
            telemetry.addData("State", state);
            telemetry.addData("Pose", "X=%.1f Y=%.1f H=%.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Busy", follower.isBusy());
            telemetry.update();
        }
    }
}
