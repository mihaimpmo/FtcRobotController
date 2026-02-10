package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Diagnostic: Strafe Angles", group = "Test")
public class DiagnosticStrafe extends LinearOpMode {
    private SwerveDrive drive;

    @Override
    public void runOpMode() {
        drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("=== STRAFE ANGLE DIAGNOSTIC ===");
        telemetry.addLine("Push left stick RIGHT to test");
        telemetry.addLine("Expected: all modules at 270°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double rawX = gamepad1.left_stick_x;
            double rawY = gamepad1.left_stick_y;

            double leftStickX = rawX;
            double leftStickY = -rawY;

            double forward = leftStickY * 3.0;
            double strafe = -leftStickX * 3.0;
            double rotation = 0.0;

            telemetry.addLine("=== INPUTS ===");
            telemetry.addData("Raw Stick", "X=%.2f, Y=%.2f", rawX, rawY);
            telemetry.addData("After Invert", "X=%.2f, Y=%.2f", leftStickX, leftStickY);
            telemetry.addLine();

            telemetry.addLine("=== ROBOT COMMANDS ===");
            telemetry.addData("Forward (X)", "%.2f m/s", forward);
            telemetry.addData("Strafe (Y)", "%.2f m/s", strafe);
            telemetry.addData("Rotation", "%.2f rad/s", rotation);
            telemetry.addLine();

            if (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1) {
                drive.drive(forward, strafe, rotation, false);
            }

            telemetry.addLine("=== MODULE STATES ===");
            drive.addTelemetry(telemetry);

            telemetry.addLine();
            telemetry.addLine("EXPECTED for RIGHT strafe:");
            telemetry.addLine("forward=0, strafe=-3.0");
            telemetry.addLine("All modules → 270°");

            telemetry.update();
        }
    }
}
