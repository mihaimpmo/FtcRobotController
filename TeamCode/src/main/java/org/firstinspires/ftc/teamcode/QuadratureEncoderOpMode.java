package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.io.IOException;

@TeleOp(name = "Quadrature Encoder CSV Event+Sample", group = "Test")
public class QuadratureEncoderOpMode extends LinearOpMode {

    private static final double COUNTS_PER_REV = 240.0;

    // 5 ms = bun pentru sample
    private static final long SAMPLE_INTERVAL_NS = 5_000_000L;

    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel encA = hardwareMap.get(DigitalChannel.class, "encA");
        DigitalChannel encB = hardwareMap.get(DigitalChannel.class, "encB");

        encA.setMode(DigitalChannel.Mode.INPUT);
        encB.setMode(DigitalChannel.Mode.INPUT);

        EncoderCsvLogger logger = new EncoderCsvLogger("/sdcard/FIRST/encoder_log.csv");

        try {
            logger.start();
        } catch (IOException e) {
            telemetry.addLine("Nu am putut porni loggerul CSV");
            telemetry.update();
            sleep(1500);
            return;
        }

        QuadratureEncoderReader encoder = new QuadratureEncoderReader(
                encA,
                encB,
                COUNTS_PER_REV,
                logger,
                SAMPLE_INTERVAL_NS
        );

        telemetry.addLine("Encoder + CSV logger ready");
        telemetry.addLine("Log: /sdcard/FIRST/encoder_log.csv");
        telemetry.addData("Sample interval ms", SAMPLE_INTERVAL_NS / 1_000_000.0);
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
            telemetry.addData("Logger failed", encoder.hasLoggerFailed());
            telemetry.addData("Dropped log writes", encoder.getDroppedLogWrites());
            telemetry.update();

            sleep(20);
        }

        encoder.stop();
    }
}