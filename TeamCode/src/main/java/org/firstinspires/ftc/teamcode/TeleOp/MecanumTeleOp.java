package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Hardware.LimelightHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

import java.util.List;


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
            if (limelightHardware.limelight == null) {
                telemetry.addData("ERROR", "Limelight not found in hardwareMap");
                telemetry.update();
                sleep(500);
                continue;
            }
            LLStatus status = limelightHardware.limelight.getStatus();
            telemetry.addLine("=== STATUS ===");
            if (status != null) {
                telemetry.addData("Pipeline", status.getPipelineIndex());
                telemetry.addData("Pipeline Type", status.getPipelineType());
                telemetry.addData("Temp (C)", "%.1f", status.getTemp());
                telemetry.addData("FPS", "%.0f", status.getFps());
            } else {
                telemetry.addData("Status", "null");
            }
            // Result info
            LLResult result = limelightHardware.limelight.getLatestResult();
            telemetry.addLine("=== RESULT ===");
            if (result == null) {
                telemetry.addData("Result", "null");
            } else {
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("tx", "%.2f", result.getTx());
                telemetry.addData("ty", "%.2f", result.getTy());
                telemetry.addData("ta", "%.2f", result.getTa());
                telemetry.addData("Staleness (ms)", "%.1f", result.getStaleness());
                telemetry.addData("Pipeline Index", result.getPipelineIndex());
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetry.addLine("=== APRILTAGS ===");
                telemetry.addData("Tags found", fiducials.size());
                for (LLResultTypes.FiducialResult f : fiducials) {
                    telemetry.addData("  Tag " + f.getFiducialId(),
                            "tx=%.1f ty=%.1f ta=%.1f",
                            f.getTargetXDegrees(), f.getTargetYDegrees(), f.getTargetArea());
                    Pose3D rp = f.getRobotPoseTargetSpace();
                    if (rp != null) {
                        Position p = rp.getPosition();
                        telemetry.addData("    RobotPose", "(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
                    }
                }
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
}
