package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

/**
 * Diagnostic: press a button, one axis of power is applied.
 * Watch telemetry to see what Pinpoint reads.
 * This tells us the exact sign mapping between drive() and odometry.
 *
 * Results to record:
 * 1. A (fwd=-0.3): does X increase or decrease? Does robot go forward or backward?
 * 2. B (str=+0.3): does Y increase or decrease? Does robot go left or right?
 * 3. X (rot=+0.3): does heading increase or decrease? Does robot turn CW or CCW?
 *
 * Press DPAD_UP to reset pose to 0,0,0.
 */
@TeleOp(name = "Sign Test", group = "Test")
public class SignTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(6.34, 6.64, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Sign Test — press START then home");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();
        drive.homeAllModules(this);
        if (!opModeIsActive()) return;

        String activeTest = "NONE";

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double h = pose.getHeading(AngleUnit.DEGREES);

            if (gamepad1.a) {
                // Test forward: drive(fwd=-0.3, str=0, rot=0)
                drive.drive(-0.3, 0, 0);
                activeTest = "FWD: drive(-0.3, 0, 0)";
            } else if (gamepad1.b) {
                // Test strafe: drive(fwd=0, str=+0.3, rot=0)
                drive.drive(0, 0.3, 0);
                activeTest = "STR: drive(0, +0.3, 0)";
            } else if (gamepad1.x) {
                // Test rotation: drive(fwd=0, str=0, rot=+1.0)
                drive.drive(0, 0, 1.0);
                activeTest = "ROT: drive(0, 0, +1.0)";
            } else {
                drive.drive(0, 0, 0);
                activeTest = "NONE (release to stop)";
            }

            if (gamepad1.dpad_up) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }

            drive.update();

            telemetry.addLine("=== SIGN TEST ===");
            telemetry.addData("Active", activeTest);
            telemetry.addLine("");
            telemetry.addData("X (inches)", "%.2f", x);
            telemetry.addData("Y (inches)", "%.2f", y);
            telemetry.addData("Heading (deg)", "%.2f", h);
            telemetry.addLine("");
            telemetry.addLine("A = forward test  (drive fwd=-0.3)");
            telemetry.addLine("B = strafe test   (drive str=+0.3)");
            telemetry.addLine("X = rotation test (drive rot=+0.3)");
            telemetry.addLine("DPAD UP = reset pose");
            telemetry.addLine("");
            telemetry.addLine("RECORD: which direction robot moves");
            telemetry.addLine("        and which value changes (+/-)");
            telemetry.update();
        }
    }
}
