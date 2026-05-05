package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.AutoHandler;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Autonomous(name = "AutoTest3", group = "Test")
public class AutoTest3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive drive = new SwerveDrive(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        AutoHandler auto = new AutoHandler(drive, pinpoint, this, this.telemetry);
        pinpoint.setOffsets(AutoHandler.PINPOINT_X_OFFSET_MM, AutoHandler.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        waitForStart();
        boolean homingSuccess = drive.homeAllModules(this);

        if (!homingSuccess) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }

            auto.set0();
            auto.rotate(90);

    }
}
