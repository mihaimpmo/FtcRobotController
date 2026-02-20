package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Straight Test", group = "Test")
@Configurable
public class StraightTest extends LinearOpMode {

    // Power to apply — tune from dashboard
    public static double DRIVE_POWER = 1.0;

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(6.64, 6.34, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Straight Test Ready");
        telemetry.addLine("A = drive forward | B = stop | X = reset pose");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();
        drive.homeAllModules(this);
        if (!opModeIsActive()) return;

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            if (gamepad1.a) {
                // Same sign as teleop: negative fwd = physical forward
                drive.drive(-DRIVE_POWER, 0, 0);
            } else if (gamepad1.b) {
                drive.drive(0, 0, 0);
            }

            drive.update();

            telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("");
            telemetry.addData("Drive Power", "%.2f", DRIVE_POWER);
            telemetry.addLine("A = forward | B = stop | X = reset pose");

            if (gamepad1.x) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }

            telemetry.update();
        }
    }
}
