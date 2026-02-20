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

@Autonomous(name = "Pedro Back-and-Forth", group = "Test")
public class PedroStraightTest extends LinearOpMode {

    private enum State { HOMING, GO_FORWARD, GO_BACK, DONE }

    @Override
    public void runOpMode() {
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);

        Follower follower = new FollowerBuilder(SwerveFollowerConstants.followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(SwervePinpointConstants.localizerConstants)
                .build();

        Pose start = new Pose(0, 0, 0);
        Pose forward = new Pose(24, 0, 0);

        follower.setStartingPose(start);

        PathChain goForward = follower.pathBuilder()
                .addPath(new BezierLine(start, forward))
                .setConstantHeadingInterpolation(0)
                .build();

        PathChain goBack = follower.pathBuilder()
                .addPath(new BezierLine(forward, start))
                .setConstantHeadingInterpolation(0)
                .build();

        telemetry.addLine("Back-and-Forth Test - Press START");
        telemetry.update();

        waitForStart();

        State state = State.HOMING;

        while (opModeIsActive()) {
            switch (state) {
                case HOMING:
                    telemetry.addLine("Homing...");
                    telemetry.update();
                    drivetrain.getSwerveDrive().homeAllModules(this);
                    follower.followPath(goForward, true);
                    state = State.GO_FORWARD;
                    break;

                case GO_FORWARD:
                    if (!follower.isBusy()) {
                        follower.followPath(goBack, true);
                        state = State.GO_BACK;
                    }
                    break;

                case GO_BACK:
                    if (!follower.isBusy()) {
                        state = State.DONE;
                    }
                    break;

                case DONE:
                    break;
            }

            follower.update();

            Pose pose = follower.getPose();
            telemetry.addData("State", state);
            telemetry.addData("Pose", "X=%.2f Y=%.2f H=%.1f°", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Busy", follower.isBusy());
            drivetrain.logDebug(telemetry);
            telemetry.update();
        }
    }
}
