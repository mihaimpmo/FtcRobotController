package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.io.IOException;

public class QuadratureEncoderReader implements Runnable {

    private final DigitalChannel chA;
    private final DigitalChannel chB;
    private final double countsPerRev;
    private final EncoderCsvLogger logger;
    private final long sampleIntervalNs;

    private volatile boolean running = false;
    private Thread worker;

    private volatile int counts = 0;
    private volatile boolean aState = false;
    private volatile boolean bState = false;
    private volatile double angleDeg = 0.0;

    private volatile double velocityCountsPerSec = 0.0;
    private volatile double velocityDegPerSec = 0.0;

    private volatile long errorCount = 0;
    private volatile int lastInvalidFrom = -1;
    private volatile int lastInvalidTo = -1;
    private volatile int lastDirection = 0;

    private volatile boolean loggerFailed = false;
    private volatile long droppedLogWrites = 0;

    private int prevState;
    private long lastVelocityTimeNs;
    private int lastVelocityCounts;
    private long lastSampleLogNs;

    public QuadratureEncoderReader(
            DigitalChannel chA,
            DigitalChannel chB,
            double countsPerRev,
            EncoderCsvLogger logger,
            long sampleIntervalNs
    ) {
        this.chA = chA;
        this.chB = chB;
        this.countsPerRev = countsPerRev;
        this.logger = logger;
        this.sampleIntervalNs = sampleIntervalNs;

        boolean a = chA.getState();
        boolean b = chB.getState();

        this.aState = a;
        this.bState = b;
        this.prevState = QuadratureUtil.encodeState(a, b);

        long now = System.nanoTime();
        this.lastVelocityTimeNs = now;
        this.lastVelocityCounts = 0;
        this.lastSampleLogNs = now;
    }

    public void start() {
        if (running) return;

        running = true;
        worker = new Thread(this, "QuadratureReader");
        worker.setDaemon(true);
        worker.start();
    }

    public void stop() {
        running = false;

        if (worker != null) {
            try {
                worker.join(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        if (logger != null) {
            logger.close();
        }
    }

    @Override
    public void run() {
        while (running) {
            boolean a = chA.getState();
            boolean b = chB.getState();

            aState = a;
            bState = b;

            int currState = QuadratureUtil.encodeState(a, b);
            long nowNs = System.nanoTime();

            boolean stateChanged = (currState != prevState);

            if (stateChanged) {
                int oldState = prevState;
                int delta = QuadratureUtil.transitionDelta(oldState, currState);

                if (delta != 0) {
                    counts += delta;
                    angleDeg = counts * 360.0 / countsPerRev;
                    lastDirection = (delta > 0) ? 1 : -1;
                } else {
                    errorCount++;
                    lastInvalidFrom = oldState;
                    lastInvalidTo = currState;
                }

                long dtNs = nowNs - lastVelocityTimeNs;
                if (dtNs >= 1_000_000L) { // 1 ms minim pentru update viteză
                    int dc = counts - lastVelocityCounts;
                    double dtSec = dtNs * 1e-9;

                    velocityCountsPerSec = dc / dtSec;
                    velocityDegPerSec = velocityCountsPerSec * 360.0 / countsPerRev;

                    lastVelocityCounts = counts;
                    lastVelocityTimeNs = nowNs;
                }

                logEventSafely(
                        nowNs,
                        oldState,
                        currState,
                        delta,
                        counts,
                        angleDeg,
                        velocityCountsPerSec,
                        velocityDegPerSec,
                        a,
                        b,
                        errorCount,
                        lastDirection
                );

                prevState = currState;
            }

            if (nowNs - lastSampleLogNs >= sampleIntervalNs) {
                logSampleSafely(
                        nowNs,
                        currState,
                        counts,
                        angleDeg,
                        velocityCountsPerSec,
                        velocityDegPerSec,
                        a,
                        b,
                        errorCount,
                        lastDirection
                );
                lastSampleLogNs = nowNs;
            }

            Thread.yield();
        }
    }

    private void logEventSafely(
            long timestampNs,
            int prevState,
            int currState,
            int delta,
            int counts,
            double angleDeg,
            double velocityCountsPerSec,
            double velocityDegPerSec,
            boolean a,
            boolean b,
            long errorCount,
            int lastDirection
    ) {
        if (logger == null || loggerFailed) return;

        try {
            logger.logEvent(
                    timestampNs,
                    prevState,
                    currState,
                    delta,
                    counts,
                    angleDeg,
                    velocityCountsPerSec,
                    velocityDegPerSec,
                    a,
                    b,
                    errorCount,
                    lastDirection
            );
        } catch (IOException e) {
            loggerFailed = true;
            droppedLogWrites++;
        }
    }

    private void logSampleSafely(
            long timestampNs,
            int currState,
            int counts,
            double angleDeg,
            double velocityCountsPerSec,
            double velocityDegPerSec,
            boolean a,
            boolean b,
            long errorCount,
            int lastDirection
    ) {
        if (logger == null || loggerFailed) return;

        try {
            logger.logSample(
                    timestampNs,
                    currState,
                    counts,
                    angleDeg,
                    velocityCountsPerSec,
                    velocityDegPerSec,
                    a,
                    b,
                    errorCount,
                    lastDirection
            );
        } catch (IOException e) {
            loggerFailed = true;
            droppedLogWrites++;
        }
    }

    public void reset() {
        counts = 0;
        angleDeg = 0.0;
        velocityCountsPerSec = 0.0;
        velocityDegPerSec = 0.0;
        errorCount = 0;
        lastInvalidFrom = -1;
        lastInvalidTo = -1;
        lastDirection = 0;
        loggerFailed = false;
        droppedLogWrites = 0;

        boolean a = chA.getState();
        boolean b = chB.getState();

        aState = a;
        bState = b;
        prevState = QuadratureUtil.encodeState(a, b);

        long now = System.nanoTime();
        lastVelocityCounts = 0;
        lastVelocityTimeNs = now;
        lastSampleLogNs = now;
    }

    public EncoderSnapshot getSnapshot() {
        return new EncoderSnapshot(
                counts,
                aState,
                bState,
                angleDeg,
                velocityCountsPerSec,
                velocityDegPerSec,
                errorCount,
                lastInvalidFrom,
                lastInvalidTo,
                lastDirection
        );
    }

    public boolean hasLoggerFailed() {
        return loggerFailed;
    }

    public long getDroppedLogWrites() {
        return droppedLogWrites;
    }
}