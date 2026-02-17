package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Swerve Teleop", group = "Drive")
public class SwerveTeleop extends LinearOpMode {
    private Hardware hardware;
    private SwerveDrive drive;
    private Outtake outtake;
    private Intake intake;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);

        telemetry.addLine("Robot-Centric Swerve Drive Ready");
        telemetry.addLine("Press START to home modules and begin");
        telemetry.update();

        waitForStart();

        // Phase 1: Home all modules (find limit switches)
        telemetry.addLine("Homing swerve modules...");
        telemetry.update();

        boolean homingSuccess = drive.homeAllModules(this);

        if (!homingSuccess) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }

        // Phase 2: Steer all wheels to forward (angle 0) using offsets
        double alignTolerance = Math.toRadians(2.0);
        while (opModeIsActive() && !drive.allAtZero(alignTolerance)) {
            drive.steerAllToZero();
            telemetry.addLine("Aligning wheels to forward...");
            drive.logDetailed(telemetry);
            telemetry.update();
        }

        // Phase 3: Reset all encoders — current position becomes 0
        drive.resetAllEncoders();
        telemetry.addLine("Encoders reset. All modules at 0.");
        telemetry.update();
        sleep(200);

        // Phase 4: Drive
        while (opModeIsActive()) {

            if (gamepad2.cross) {
                if(gamepad2.left_bumper) outtake.setTargetRPM(OuttakeConstants.TARGET_RPM + 250);
                else outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
            } else {
                outtake.setTargetRPM(0);
            }
            outtake.rampShoot(gamepad2.dpad_up);
            outtake.update();

            if (gamepad2.square) {
                intake.Start(0.9);
            } else {
                intake.Stop();
            }

            if (gamepad2.dpad_up) {
                drive.zero();
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            if (Math.abs(forward) < ControlConstants.DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.DEADBAND) rotation = 0;

            drive.drive(forward, strafe, rotation);

            telemetry.addData("Fwd/Str/Rot", "%.1f / %.1f / %.1f", forward, strafe, rotation);
            drive.logDetailed(telemetry);
            telemetry.update();
        }
    }
}
