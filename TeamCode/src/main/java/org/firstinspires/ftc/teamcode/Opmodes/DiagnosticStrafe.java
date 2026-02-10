package org.firstinspires.ftc.teamcode.Opmodes;

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
        telemetry.addLine("Expected: all modules at 270\u00b0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double rawX = gamepad1.left_stick_x;
            double rawY = gamepad1.left_stick_y;

            double leftStickX = rawX;
            double leftStickY = -rawY;

            double forward = leftStickY;
            double strafe = -leftStickX;
            double rotation = 0.0;

            telemetry.addLine("=== INPUTS ===");
            telemetry.addData("Raw Stick", "X=%.2f, Y=%.2f", rawX, rawY);
            telemetry.addData("After Invert", "X=%.2f, Y=%.2f", leftStickX, leftStickY);
            telemetry.addLine();

            telemetry.addLine("=== ROBOT COMMANDS ===");
            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Rotation", "%.2f", rotation);
            telemetry.addLine();

            if (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1) {
                drive.drive(forward, strafe, rotation);
            }

            telemetry.addLine("=== MODULE STATES ===");
            drive.log(telemetry);

            telemetry.addLine();
            telemetry.addLine("EXPECTED for RIGHT strafe:");
            telemetry.addLine("forward=0, strafe=-1.0");
            telemetry.addLine("All modules \u2192 270\u00b0");

            telemetry.update();
        }
    }
}
