package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    public ChassisHardware hardware;
    public Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {


        hardware = new ChassisHardware(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hardware.backLeftMotor.setPower(backLeftPower);
            hardware.frontLeftMotor.setPower(frontLeftPower);
            hardware.backRightMotor.setPower(backRightPower);
            hardware.frontRightMotor.setPower(frontRightPower);

            if (camera.hasAprilTag()) {
                telemetry.addData("AprilTag X (m)", camera.getAprilTagX());
                telemetry.addData("AprilTag Z (m)", camera.getAprilTagZ());
                telemetry.addData("Distance (m)", camera.getAprilTagDistance());
            } else {
                telemetry.addLine("No AprilTag detected");
            }

            telemetry.update();
        }
    }
}
