package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.Utils.ToggleButton;

@TeleOp(name = "Swerve Teleop", group = "Drive")
public class SwerveTeleop extends LinearOpMode {
    private Hardware hardware;
    private SwerveDrive drive;
    private Outtake outtake;
    private Intake intake;

    private final ToggleButton outtakeToggle = new ToggleButton();
    private boolean outtakeFast = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);

        telemetry.addLine("Swerve Drive Ready");
        telemetry.addLine("Press START to home modules and begin");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing swerve modules...");
        telemetry.update();

        boolean homingSuccess = drive.homeAllModules(this);

        if (!homingSuccess) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }

        long lastLoopTime = System.nanoTime();
        while (opModeIsActive()) {
            long currentTime = System.nanoTime();
            double loopTimeMs = (currentTime - lastLoopTime) / 1e6;
            lastLoopTime = currentTime;

            outtakeToggle.update(gamepad2.cross);

            if (gamepad2.right_bumper && !lastRightBumper) outtakeFast = true;
            if (gamepad2.left_bumper && !lastLeftBumper) outtakeFast = false;
            lastRightBumper = gamepad2.right_bumper;
            lastLeftBumper = gamepad2.left_bumper;

            if (outtakeToggle.isToggled()) {
                double rpm = outtakeFast
                        ? OuttakeConstants.TARGET_RPM + OuttakeConstants.DEFAULT_TO_MAX
                        : OuttakeConstants.TARGET_RPM;
                outtake.setTargetRPM(rpm);
            } else {
                outtake.setTargetRPM(0);
            }
            outtake.rampShoot(gamepad2.dpad_up);
            outtake.update();

            if (gamepad2.square) {
                intake.Start(0.9);
            } else if(!gamepad2.square&&!gamepad2.circle) {
                intake.Stop();
            }

            else if(gamepad2.circle) {
                intake.Start(-0.9);
            } else {
                intake.Stop();
            }
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            if (Math.abs(forward) < ControlConstants.DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.DEADBAND) rotation = 0;

            drive.drive(forward * 5.0, strafe * 5.0, rotation * 5.0);
            drive.update();

            telemetry.addData("Loop Time", "%.1f ms (%.0f Hz)", loopTimeMs, 1000.0 / loopTimeMs);
            telemetry.addData("Fwd/Str/Rot", "%.1f / %.1f / %.1f", forward, strafe, rotation);
            drive.logDetailed(telemetry);
            intake.log(telemetry);
            outtake.log(telemetry);
            telemetry.update();
        }
    }
}
