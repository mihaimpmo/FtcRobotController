package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Swerve Teleop (Angle)", group = "Drive")
public class SwerveTeleopAngle extends LinearOpMode {
    private Hardware hardware;

    private Outtake outtake;

    private Intake intake;

    private SwerveDrive drive;
    private double lastAngleDegrees = 0.0;

    @Override
    public void runOpMode() {
        hardware =new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);

        telemetry.addLine("Robot-Centric Swerve Drive Ready");
        telemetry.addLine("Press START to reset wheels to 0 and begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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
            outtake.rampShoot(gamepad2.triangle);
            outtake.update();

            if (gamepad2.square) {
                intake.Start(0.9);
            } else {
                intake.Stop();
            }



            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rotationInput = -gamepad1.right_stick_x;

            if (Math.abs(leftStickX) < ControlConstants.JOYSTICK_DEADBAND) leftStickX = 0;
            if (Math.abs(leftStickY) < ControlConstants.JOYSTICK_DEADBAND) leftStickY = 0;
            if (Math.abs(rotationInput) < ControlConstants.JOYSTICK_DEADBAND) rotationInput = 0;

            double magnitude = Math.sqrt(leftStickX * leftStickX + leftStickY * leftStickY);
            double joystickAngleRadians = Math.atan2(leftStickY, leftStickX);
            double joystickAngleDegrees = Math.toDegrees(joystickAngleRadians);
            if (joystickAngleDegrees < 0) joystickAngleDegrees += 360;

            double robotAngleDegrees = joystickAngleDegrees - 90.0;
            if (robotAngleDegrees < 0) robotAngleDegrees += 360;

            if (magnitude > ControlConstants.JOYSTICK_DEADBAND) {
                lastAngleDegrees = robotAngleDegrees;
            }

            double forward = leftStickY * ControlConstants.MAX_DRIVE_SPEED;
            double strafe = -leftStickX * ControlConstants.MAX_DRIVE_SPEED;
            double rotation = rotationInput * ControlConstants.MAX_ROTATION_SPEED;

            drive.drive(forward, strafe, rotation, true);

            telemetry.addData("Angle", "%.0f\u00b0", lastAngleDegrees);
            telemetry.addData("Fwd/Str/Rot", "%.1f / %.1f / %.1f", forward, strafe, rotation);
            drive.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
