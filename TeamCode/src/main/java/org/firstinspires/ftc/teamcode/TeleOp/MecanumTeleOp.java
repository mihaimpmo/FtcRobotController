package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Hardware.LimelightHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
@TeleOp(name = "Limelight debug")
public class MecanumTeleOp extends LinearOpMode {
    public ChassisHardware hardware;
    public LimelightHardware limelightHardware;
    public Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {


        hardware = new ChassisHardware(hardwareMap);
        limelightHardware = new LimelightHardware(hardwareMap);
        camera = new Camera(limelightHardware);

        waitForStart();

        while (opModeIsActive()) {
            camera.update();
            telemetry.update();
            telemetry.addData("tx: ", camera.getTx());
            telemetry.addData("ty: ", camera.getTy());
            telemetry.addData("tid: ", camera.getTid());
            telemetry.addData("distance: ", camera.hdistance(camera.getTy()));



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
        }
    }
}

