package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Configurable
@Autonomous(name = "Auto10", group = "Auto")
public class SimpleAuto3 extends LinearOpMode {
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;


    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerve = new SwerveDrive(this.hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        //AutoActions actions = new AutoActions(swerve, pinpoint, this);
        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        waitForStart();
        boolean homingSuccess = swerve.homeAllModules(this);

        if (!homingSuccess) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }
        //CURSOR//

        //CURSOR//
        //or calling drive once with drive(0, 0, 0);
        swerve.drive(0, 0, 0);

        sleep(250);
        double meters = 0.5;
        double rawForward = -0.5;
        double rawStrafe = 0;
        double rawRotation = 0;
        double forward = Math.pow(rawForward, 3);
        double strafe = Math.pow(rawStrafe, 3);
        double rotation = Math.pow(rawRotation, 3);
        while(opModeIsActive() && getXMeters(pinpoint) < meters) {
            swerve.drive(forward*5.0, strafe*5.0, rotation*5.0);
            swerve.update();
            telemetry.addData("X", getXMeters(pinpoint));
            telemetry.addData("Y", getYMeters(pinpoint));
            telemetry.addData("Heading", getHeadingDegrees(pinpoint));
            telemetry.addData("Remaining", meters - getXMeters(pinpoint));
            swerve.logDetailed(telemetry);
            telemetry.update();
        }
        swerve.drive(0, 0, 0);
        swerve.update();
    }
    public Pose2D getPose(GoBildaPinpointDriver pinpoint) {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public double getXMeters(GoBildaPinpointDriver pinpoint) {
        return getPose(pinpoint).getX(DistanceUnit.METER);
    }

    public double getYMeters(GoBildaPinpointDriver pinpoint) {
        return getPose(pinpoint).getY(DistanceUnit.METER);
    }

    public double getHeadingDegrees(GoBildaPinpointDriver pinpoint) {
        return getPose(pinpoint).getHeading(AngleUnit.DEGREES);
    }
}
