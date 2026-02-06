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
        telemetry.addLine("Left Stick: Forward/Strafe");
        telemetry.addLine("Right Stick X: Rotation");
        telemetry.addLine("D-Pad Up: Reset modules to 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Outtake: Cross to spin, bumpers change speed
            if (gamepad2.cross) {
                if (gamepad2.left_bumper) {
                    outtake.setTargetRPM(1500);
                } else if (gamepad2.right_bumper) {
                    outtake.setTargetRPM(4000);
                } else {
                    outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                }
            } else {
                outtake.stop();
            }
            outtake.rampShoot(gamepad2.dpad_up);
            outtake.update();

            if (gamepad2.square) {
                intake.Start(0.9);
            } else {
                intake.Stop();
            }

            if (gamepad2.dpad_up) {
                drive.resetModulesToZero();
            }

            // Joystick inputs (inverted to match WPILib: +X forward, +Y left, +rot CCW)
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            if (Math.abs(forward) < ControlConstants.JOYSTICK_DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.JOYSTICK_DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.JOYSTICK_DEADBAND) rotation = 0;

            forward *= ControlConstants.MAX_DRIVE_SPEED;
            strafe *= ControlConstants.MAX_DRIVE_SPEED;
            rotation *= ControlConstants.MAX_ROTATION_SPEED;

            // OLD: Inconsistent optimize flag caused unpredictable flipping
            // boolean isRotating = Math.abs(rotation) > 0;
            // drive.drive(forward, strafe, rotation, !isRotating);

            // NEW: Use 3-arg drive() which defaults to optimize=false
            // Our manual ±90° clamping in SwerveModule handles optimization
            drive.drive(forward, strafe, rotation);

            telemetry.addData("Fwd/Str/Rot", "%.1f / %.1f / %.1f", forward, strafe, rotation);
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
