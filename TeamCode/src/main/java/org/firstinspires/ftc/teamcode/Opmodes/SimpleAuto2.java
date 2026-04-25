package org.firstinspires.ftc.teamcode.Opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.AutoActions;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Configurable
@Autonomous(name = "Auto5", group = "Auto")
public class SimpleAuto2 extends LinearOpMode {
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;
    GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerve = new SwerveDrive(this.hardwareMap);
        //GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        //AutoActions actions = new AutoActions(swerve, pinpoint, this);
        pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        waitForStart();
        boolean homingSuccess = swerve.homeAllModules(this);
        sleep(5000);

        if (!homingSuccess) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }
        double meters = 0.5;
        double rawForward = -0.5;
        double rawStrafe = 0;
        double rawRotation = 0;
        double forward = Math.pow(rawForward, 3);
        double strafe = Math.pow(rawStrafe, 3);
        double rotation = Math.pow(rawRotation, 3);
        while(getXMeters() < meters) {
            swerve.drive(forward*5.0, strafe*5.0, rotation*5.0);
            swerve.update();
            telemetry.addData("X", getXMeters());
            telemetry.addData("Y", getYMeters());
            telemetry.addData("Heading", getHeadingDegrees());
            telemetry.addData("Remaining", meters - getXMeters());
            telemetry.update();
        }
    }
    public Pose2D getPose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public double getXMeters() {
        return getPose().getX(DistanceUnit.METER);
    }

    public double getYMeters() {
        return getPose().getY(DistanceUnit.METER);
    }

    public double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }
}
