package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Leave", group = "Drive")
public class Leave extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("Leave Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();
        if (!drive.homeAllModules(this)) {
            telemetry.addLine("Homing Failed!");
            telemetry.update();
            sleep(2000);
            return;
        }

        telemetry.addLine("Driving Forward for 2 Seconds...");
        telemetry.update();
        
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 2000) {
            // Forward is negative in the drive method based on existing conventions
            drive.drive(-0.4, 0, 0);
            drive.update();

            drive.logDetailed(telemetry);
            telemetry.update();
        }

        drive.drive(0, 0, 0);
        drive.update();
        telemetry.addLine("Done");
        telemetry.update();
        sleep(1000);
    }
}
