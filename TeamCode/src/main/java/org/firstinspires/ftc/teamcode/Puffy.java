package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Teleopmotor")
public class Puffy extends LinearOpMode {
    Motoare motoare;
    @Override
    public void runOpMode() throws InterruptedException {
        //Hardware hw = new Hardware(hardwareMap);
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        motoare = new Motoare();
        waitForStart();
        motoare.runFor(frontRight,2, 5000);
//        while(opModeIsActive()) {
//
//
//        }
    }
}
