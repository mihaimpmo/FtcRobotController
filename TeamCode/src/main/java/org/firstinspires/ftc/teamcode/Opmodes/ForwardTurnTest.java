package org.firstinspires.ftc.teamcode.Opmodes;

import static org.firstinspires.ftc.teamcode.Opmodes.SimpleAuto.SPINUP_MS;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Autonomous(name = "Swerve Auto", group = "Auto")
public class ForwardTurnTest extends LinearOpMode {

    private static final double BRAKE_POWER = -0.3;
    private static final long BRAKE_MS = 300;
    private static final long SPINDOWN_MS = 1700; // rest of 2 sec after braking

    private Hardware hardware;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        SwerveDrive drive = new SwerveDrive(hardwareMap);
        Outtake outtake = new Outtake(hardware);
        Intake intake = new Intake(hardware);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(6.64, 6.34, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        AutoDrive auto = new AutoDrive(drive, pinpoint, this);

        telemetry.addLine("Swerve Auto Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();
        if (!drive.homeAllModules(this)) return;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // Drive forward 84 inches
        auto.forward(84);
        if (!opModeIsActive()) return;

        // Rotate 30 degrees
        auto.turnTo(30);
        if (!opModeIsActive()) return;

        // === Shot 1 ===
        spinUp(outtake);
        if (!opModeIsActive()) return;
        shootAndBrake(outtake);
        if (!opModeIsActive()) return;

        // === Shot 2 ===
        spinUp(outtake);
        if (!opModeIsActive()) return;
        shootAndBrake(outtake);
        if (!opModeIsActive()) return;

        // Activate intake for 1 sec to load ball 3
        intake.Start(0.9);
        sleep(1000);
        intake.Stop();
        if (!opModeIsActive()) return;

        // === Shot 3 ===
        spinUp(outtake);
        if (!opModeIsActive()) return;
        shootAndBrake(outtake);
        if (!opModeIsActive()) return;

        // Done
        outtake.stop();
        auto.stop();

        while (opModeIsActive()) {
            idle();
        }
    }

    private void spinUp(Outtake outtake) {
        outtake.setTargetRPM(OuttakeConstants.TARGET_RPM+500);
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < SPINUP_MS) {
            outtake.update();
            telemetry.addData("State", "SPINNING UP");
            telemetry.update();
        }
    }

    private void shootAndBrake(Outtake outtake) {
        // Toggle ramp to SHOOT
        outtake.rampShoot(true);
        outtake.update();
        sleep(1000);
        // Lower ramp back to IDLE
        outtake.rampShoot(false); // reset edge detection
        outtake.rampShoot(true);  // toggle back to IDLE
        outtake.update();
        sleep(300); // wait for ramp to physically lower

        // Brake flywheel with negative current
        outtake.stop();
        outtake.update();
        hardware.outtakeHardware.WheelMotor1.setPower(BRAKE_POWER);
        hardware.outtakeHardware.WheelMotor2.setPower(BRAKE_POWER);
        sleep(BRAKE_MS);
        hardware.outtakeHardware.WheelMotor1.setPower(0);
        hardware.outtakeHardware.WheelMotor2.setPower(0);
        sleep(SPINDOWN_MS);
    }
}
