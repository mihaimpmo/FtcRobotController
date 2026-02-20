package org.firstinspires.ftc.teamcode.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Pedro.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Pedro.SwerveFollowerConstants;
import org.firstinspires.ftc.teamcode.Pedro.SwervePinpointConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);
        SwerveDrive drive = drivetrain.getSwerveDrive();

        Follower follower = new FollowerBuilder(SwerveFollowerConstants.followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(SwervePinpointConstants.localizerConstants)
                .build();

        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Odometry Test Ready - Press START");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();
        drive.homeAllModules(this);

        while (opModeIsActive()) {
            follower.update();

            if (gamepad1.a) {
            follower.setPose(new Pose(0, 0, 0));
            }

            Pose pose = follower.getPose();

            telemetry.addData("X", "%.2f in", pose.getX());
            telemetry.addData("Y", "%.2f in", pose.getY());
            telemetry.addData("Heading", "%.2f deg", Math.toDegrees(pose.getHeading()));
            telemetry.addLine("Press A to reset pose");
            telemetry.update();
        }
    }
}
