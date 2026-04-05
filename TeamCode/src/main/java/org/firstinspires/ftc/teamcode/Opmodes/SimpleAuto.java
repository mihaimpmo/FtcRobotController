package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.AutoActions;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Autonomous(name = "Simple Auto", group = "Auto")
public class SimpleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        AutoActions autoActions = new AutoActions(this, drive, pinpoint);

        autoActions.configurePinpoint();
        autoActions.setStartPose(0.0, 0.0, 0.0);

        telemetry.addLine("Simple Auto Ready");
        telemetry.addLine("Pinpoint configured on hardware name: pinpoint");
        telemetry.addLine("Press START to home modules and run");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        boolean homed = autoActions.homeDrive();
        telemetry.addData("Homing", homed ? "ok" : "failed");
        telemetry.update();

        if (!homed) {
            sleep(1500);
            return;
        }

        autoActions.forward(60.0);
        autoActions.pause(250);
        autoActions.strafeLeft(30.0);
        autoActions.pause(250);
        autoActions.turnBy(90.0);
        autoActions.pause(250);
        autoActions.forward(30.0);
        autoActions.stop();
    }
}
