package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Custom Quadrature Encoder Threaded", group = "Test")
public class EncoderOpMode extends LinearOpMode {

    private static final int CPR = 112; //counts per rotation

    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel pinA = hardwareMap.get(DigitalChannel.class, "Channel1");
        DigitalChannel pinB = hardwareMap.get(DigitalChannel.class, "Channel2");

        pinA.setMode(DigitalChannel.Mode.INPUT);
        pinB.setMode(DigitalChannel.Mode.INPUT);

        QuadratureEncoder encoder = new QuadratureEncoder(pinA, pinB, CPR);

        telemetry.addLine("Quadrature encoder ready");
        telemetry.addData("Initial State", EncoderUtil.stateToString(encoder.getState()));
        telemetry.update();

        waitForStart();

        encoder.start();

        while (opModeIsActive()) {
            telemetry.addData("Count", encoder.getCount());
            telemetry.addData("Angle", "%.2f", encoder.getAngleDeg());

            telemetry.addData("Velocity (counts/s)", "%.2f", encoder.getVelocityCountsPerSec());
            telemetry.addData("Velocity (deg/s)", "%.2f", encoder.getVelocityDegPerSec());

            telemetry.addData("Last Error Vel (counts/s)", "%.2f", encoder.getLastErrorVelocityCountsPerSec());
            telemetry.addData("Last Error Vel (deg/s)", "%.2f", encoder.getLastErrorVelocityDegPerSec());

            telemetry.addData("Min Error Vel (counts/s)", "%.2f", encoder.getMinErrorVelocityCountsPerSec());
            telemetry.addData("Min Error Vel (deg/s)", "%.2f", encoder.getMinErrorVelocityDegPerSec());

            telemetry.addData("A", encoder.getA() ? 1 : 0);
            telemetry.addData("B", encoder.getB() ? 1 : 0);
            telemetry.addData("State", EncoderUtil.stateToString(encoder.getState()));

            telemetry.addData("Errors", encoder.getErrorCount());
            telemetry.addData("Last Invalid", encoder.getLastInvalidTransitionString());
            telemetry.addData("Direction", encoder.getLastDirectionString());

            telemetry.update();
            idle();
        }

        encoder.stop();
    }
}