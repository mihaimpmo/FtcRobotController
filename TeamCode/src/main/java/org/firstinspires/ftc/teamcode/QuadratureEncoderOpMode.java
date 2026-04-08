package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Quadrature Encoder", group = "Test")
public class QuadratureEncoderOpMode extends LinearOpMode {

    private static final double COUNTS_PER_REV = 240.0; // 60 magneti * 4x

    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel encA = hardwareMap.get(DigitalChannel.class, "encA");
        DigitalChannel encB = hardwareMap.get(DigitalChannel.class, "encB");

        encA.setMode(DigitalChannel.Mode.INPUT);
        encB.setMode(DigitalChannel.Mode.INPUT);

        QuadratureEncoderReader encoder = new QuadratureEncoderReader(encA, encB, COUNTS_PER_REV);

        telemetry.addLine("Encoder initialized");
        telemetry.update();

        waitForStart();

        encoder.start();

        while (opModeIsActive()) {
            EncoderSnapshot s = encoder.getSnapshot();

            telemetry.addData("A", s.a ? 1 : 0);
            telemetry.addData("B", s.b ? 1 : 0);
            telemetry.addData("Counts", s.counts);
            telemetry.addData("Angle (deg)", "%.2f", s.angleDeg);
            telemetry.addData("Speed (counts/s)", "%.2f", s.velocityCountsPerSec);
            telemetry.addData("Speed (deg/s)", "%.2f", s.velocityDegPerSec);
            telemetry.addData("Errors", s.errorCount);
            telemetry.addData("Last invalid", QuadratureUtil.transitionToString(s.lastInvalidFrom, s.lastInvalidTo));
            telemetry.addData("Direction", QuadratureUtil.directionToString(s.lastDirection));
            telemetry.update();

            sleep(20);
        }

        encoder.stop();
    }
}