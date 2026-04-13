package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.AutoConstants;

@TeleOp(name = "OdoTest", group = "Test")
public class OdoTest extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(AutoConstants.PINPOINT_X_OFFSET_MM, AutoConstants.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();

        telemetry.addLine("Pinpoint ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            Pose2D pos = odo.getPosition();

            double x = pos.getX(DistanceUnit.MM);
            double y = pos.getY(DistanceUnit.MM);
            double heading = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addData("X (mm)", "%.2f", x);
            telemetry.addData("Y (mm)", "%.2f", y);
            telemetry.addData("Heading (deg)", "%.2f", heading);

            telemetry.update();
        }
    }
}