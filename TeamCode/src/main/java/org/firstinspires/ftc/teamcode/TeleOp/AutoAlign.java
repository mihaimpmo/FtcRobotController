package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Hardware.ChassisHardware;
import org.firstinspires.ftc.teamcode.Hardware.LimelightHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.PIDControl;
import org.firstinspires.ftc.teamcode.Utils.Constants;

@Autonomous
public class AutoAlign extends LinearOpMode {
    public static Constants constants;
    public ChassisHardware hardware;
    public LimelightHardware limelightHardware;
    public Camera camera;
    public PIDControl pidControl;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new ChassisHardware(hardwareMap);
        limelightHardware = new LimelightHardware(hardwareMap);
        camera = new Camera(limelightHardware);
        pidControl = new PIDControl(constants.kp_turn, constants.ki_turn, constants.kd_turn);
        waitForStart();

        while(opModeIsActive()) {
            camera.update();
            //tag is not seen
            if(!camera.hasTarget()) {
                while(!camera.hasTarget()) {
                    camera.update();
                    //turnRobot(turnPower);
                }
            }
            //tag is seen, but the robot isn't correctly aligned
            else if (Math.abs(camera.getTx()) > 1.0) {
                double turnPower = pidControl.calculate(camera.getTx(), 0);
                //turnRobot(turnPower);
            }
            //tag is seen, yaw is ok, but strafe isn't correct
            else {

                if (camera.getTx() > 1.0) {
                    //strafeLeft(0.3);
                } else if (camera.getTx() < -1.0) {
                    //strafeRight(0.3);
                }
            }

        }
    }

}
