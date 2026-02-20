package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Autonomous(name = "Strafe Turn Test", group = "Test")
public class ForwardTurnTest extends LinearOpMode {

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

        AutoDrive auto = new AutoDrive(drive, pinpoint, this);

        telemetry.addLine("Strafe + Turn Test Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();
        if (!drive.homeAllModules(this)) return;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // Strafe right 12 inches
        auto.strafe(24);
        if (!opModeIsActive()) return;

        // Turn 45° CCW
        auto.turnTo(45);
        if (!opModeIsActive()) return;

        // Done
        telemetry.addLine("DONE");
        telemetry.update();
        auto.stop();

        while (opModeIsActive()) {
            idle();
        }
    }
}
