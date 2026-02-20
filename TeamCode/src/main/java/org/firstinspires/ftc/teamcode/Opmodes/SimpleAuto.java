package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Autonomous(name = "Simple Auto", group = "Auto")
@Configurable
public class SimpleAuto extends LinearOpMode {

    public static double FORWARD_INCHES = 24.0;
    public static double SHOOT_HEADING = 0.0;
    public static double SPINUP_MS = 1500;

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        SwerveDrive drive = new SwerveDrive(hardwareMap);
        Outtake outtake = new Outtake(hardware);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(6.34, 6.64, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        AutoDrive auto = new AutoDrive(drive, pinpoint, this);

        telemetry.addLine("Simple Auto Ready");
        telemetry.update();

        waitForStart();

        // Home
        telemetry.addLine("Homing...");
        telemetry.update();
        if (!drive.homeAllModules(this)) return;
        pinpoint.recalibrateIMU();
        sleep(500);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();

        // Drive forward
        auto.forward(FORWARD_INCHES);
        if (!opModeIsActive()) return;

        // Turn to shoot heading
        auto.turnTo(SHOOT_HEADING);
        if (!opModeIsActive()) return;

        // Spin up outtake
        outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < SPINUP_MS) {
            outtake.update();
            telemetry.addData("State", "SPINNING UP");
            telemetry.update();
        }

        // Shoot
        outtake.rampShoot(true);
        outtake.update();
        sleep(500);

        // Done
        outtake.stop();
        auto.stop();
    }
}
