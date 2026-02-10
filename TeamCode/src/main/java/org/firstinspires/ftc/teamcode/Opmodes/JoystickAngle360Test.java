package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Joystick Angle 360 Test", group = "Swerve")
public class JoystickAngle360Test extends LinearOpMode {

    private double lastAngleDegrees = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;

            if (Math.sqrt(leftStickX * leftStickX + leftStickY * leftStickY) > 0.1) {
                double angleRadians = Math.atan2(leftStickX, leftStickY);

                double angleDegrees = Math.toDegrees(angleRadians);

                if (angleDegrees < 0) {
                    angleDegrees += 360;
                }
                lastAngleDegrees = angleDegrees;
            }

            telemetry.addData("Left Stick X", "%.3f", leftStickX);
            telemetry.addData("Left Stick Y", "%.3f", leftStickY);
            telemetry.addData("Joystick Angle (0-360)", "%.1f", lastAngleDegrees);
            telemetry.update();
        }
    }
}
