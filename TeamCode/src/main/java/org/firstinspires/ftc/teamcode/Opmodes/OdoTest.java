package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "OdoTest", group = "Test")
public class OdoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Pose2D pose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        while(opModeIsActive()) {
            telemetry.addData("x", pose.getX(DistanceUnit.CM));
            telemetry.addData("y", pose.getY(DistanceUnit.CM));
            telemetry.addData("heading", pose.getHeading(AngleUnit.DEGREES));
        }
    }
}
