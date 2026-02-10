package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

@TeleOp(name = "Test: Angle Calculation", group = "Test")
public class AngleCalculationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                DriveConstants.FL_POSITION,
                DriveConstants.FR_POSITION,
                DriveConstants.BL_POSITION,
                DriveConstants.BR_POSITION
        );

        telemetry.addLine("=== KINEMATICS TEST ===");
        telemetry.addLine("Testing pure strafe commands");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("=== MODULE POSITIONS ===");
            telemetry.addData("FL", "(%.3f, %.3f)",
                DriveConstants.FL_POSITION.getX(), DriveConstants.FL_POSITION.getY());
            telemetry.addData("FR", "(%.3f, %.3f)",
                DriveConstants.FR_POSITION.getX(), DriveConstants.FR_POSITION.getY());
            telemetry.addData("BL", "(%.3f, %.3f)",
                DriveConstants.BL_POSITION.getX(), DriveConstants.BL_POSITION.getY());
            telemetry.addData("BR", "(%.3f, %.3f)",
                DriveConstants.BR_POSITION.getX(), DriveConstants.BR_POSITION.getY());
            telemetry.addLine();

            ChassisSpeeds strafeRight = new ChassisSpeeds(0, -3.0, 0);
            SwerveModuleState[] statesRight = kinematics.toSwerveModuleStates(strafeRight);

            telemetry.addLine("=== TEST 1: STRAFE RIGHT ===");
            telemetry.addLine("Command: vx=0, vy=-3.0, omega=0");
            telemetry.addLine("Expected: all modules at 270° (-90°)");
            telemetry.addLine("Actual:");
            for (int i = 0; i < 4; i++) {
                String name = new String[]{"FL", "FR", "BL", "BR"}[i];
                double angleDeg = Math.toDegrees(statesRight[i].angle.getRadians());
                double speed = statesRight[i].speedMetersPerSecond;
                telemetry.addData(name, "%.1f° @ %.2f m/s", angleDeg, speed);
            }
            telemetry.addLine();

            ChassisSpeeds strafeLeft = new ChassisSpeeds(0, 3.0, 0);
            SwerveModuleState[] statesLeft = kinematics.toSwerveModuleStates(strafeLeft);

            telemetry.addLine("=== TEST 2: STRAFE LEFT ===");
            telemetry.addLine("Command: vx=0, vy=+3.0, omega=0");
            telemetry.addLine("Expected: all modules at 90°");
            telemetry.addLine("Actual:");
            for (int i = 0; i < 4; i++) {
                String name = new String[]{"FL", "FR", "BL", "BR"}[i];
                double angleDeg = Math.toDegrees(statesLeft[i].angle.getRadians());
                double speed = statesLeft[i].speedMetersPerSecond;
                telemetry.addData(name, "%.1f° @ %.2f m/s", angleDeg, speed);
            }
            telemetry.addLine();

            ChassisSpeeds diagonal = new ChassisSpeeds(3.0, -3.0, 0);
            SwerveModuleState[] statesDiag = kinematics.toSwerveModuleStates(diagonal);

            telemetry.addLine("=== TEST 3: DIAGONAL ===");
            telemetry.addLine("Command: vx=3.0, vy=-3.0, omega=0");
            telemetry.addLine("Expected: all modules at 315° (-45°)");
            telemetry.addLine("Actual:");
            for (int i = 0; i < 4; i++) {
                String name = new String[]{"FL", "FR", "BL", "BR"}[i];
                double angleDeg = Math.toDegrees(statesDiag[i].angle.getRadians());
                double speed = statesDiag[i].speedMetersPerSecond;
                telemetry.addData(name, "%.1f° @ %.2f m/s", angleDeg, speed);
            }

            telemetry.update();
            sleep(100);
        }
    }
}
