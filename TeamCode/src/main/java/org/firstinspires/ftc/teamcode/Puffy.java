package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleopmotor")
public class Puffy extends LinearOpMode {
    Motoare motoare;
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            motoare.runFor(hw.frontRight,2, 5000);
        }
    }
}
