package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Autonomous(name = "tiganiepistisugipula", group = "pula")
public class tiganieambulanta extends LinearOpMode {

    private Hardware hardware;
    private SwerveDrive drive;
    private Outtake outtake;
    private Intake intake;

    @Override
    public void runOpMode() {

        hardware = new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);

        waitForStart();

        if (!opModeIsActive()) return;

        long startTime = System.currentTimeMillis();

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < 1500) {

            drive.drive(0, 1.0, 0.0);
        }

        drive.drive(0.0, 0.0, 0.0);
        drive.hold();
    }
}

