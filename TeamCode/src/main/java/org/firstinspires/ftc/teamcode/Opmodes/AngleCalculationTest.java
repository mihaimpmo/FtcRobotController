package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

@TeleOp(name = "Test: Angle Calculation", group = "Test")
public class AngleCalculationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("=== KINEMATICS TEST ===");
        telemetry.addLine("Testing pure strafe commands");
        telemetry.update();

        waitForStart();

        double[][] pos = DriveConstants.WHEEL_POS;
        String[] names = {"FL", "FR", "BL", "BR"};

        while (opModeIsActive()) {
            telemetry.addLine("=== MODULE POSITIONS ===");
            for (int i = 0; i < 4; i++) {
                telemetry.addData(names[i], "(%.3f, %.3f)", pos[i][0], pos[i][1]);
            }
            telemetry.addLine();

            // Test 1: Strafe right (fwd=0, str=-3, rot=0)
            double fwd = 0, str = -3.0, rot = 0;
            telemetry.addLine("=== TEST 1: STRAFE RIGHT ===");
            telemetry.addLine("Command: fwd=0, str=-3.0, rot=0");
            telemetry.addLine("Expected: all modules at 270\u00b0 (-90\u00b0)");
            telemetry.addLine("Actual:");
            for (int i = 0; i < 4; i++) {
                double vx = fwd + (-rot * pos[i][1]);
                double vy = str + ( rot * pos[i][0]);
                double angle = Math.toDegrees(Math.atan2(vy, vx));
                double speed = Math.sqrt(vx * vx + vy * vy);
                telemetry.addData(names[i], "%.1f\u00b0 @ %.2f", angle, speed);
            }
            telemetry.addLine();

            // Test 2: Strafe left (fwd=0, str=3, rot=0)
            fwd = 0; str = 3.0; rot = 0;
            telemetry.addLine("=== TEST 2: STRAFE LEFT ===");
            telemetry.addLine("Command: fwd=0, str=+3.0, rot=0");
            telemetry.addLine("Expected: all modules at 90\u00b0");
            telemetry.addLine("Actual:");
            for (int i = 0; i < 4; i++) {
                double vx = fwd + (-rot * pos[i][1]);
                double vy = str + ( rot * pos[i][0]);
                double angle = Math.toDegrees(Math.atan2(vy, vx));
                double speed = Math.sqrt(vx * vx + vy * vy);
                telemetry.addData(names[i], "%.1f\u00b0 @ %.2f", angle, speed);
            }
            telemetry.addLine();

            // Test 3: Diagonal (fwd=3, str=-3, rot=0)
            fwd = 3.0; str = -3.0; rot = 0;
            telemetry.addLine("=== TEST 3: DIAGONAL ===");
            telemetry.addLine("Command: fwd=3.0, str=-3.0, rot=0");
            telemetry.addLine("Expected: all modules at 315\u00b0 (-45\u00b0)");
            telemetry.addLine("Actual:");
            for (int i = 0; i < 4; i++) {
                double vx = fwd + (-rot * pos[i][1]);
                double vy = str + ( rot * pos[i][0]);
                double angle = Math.toDegrees(Math.atan2(vy, vx));
                double speed = Math.sqrt(vx * vx + vy * vy);
                telemetry.addData(names[i], "%.1f\u00b0 @ %.2f", angle, speed);
            }

            telemetry.update();
            sleep(100);
        }
    }
}
