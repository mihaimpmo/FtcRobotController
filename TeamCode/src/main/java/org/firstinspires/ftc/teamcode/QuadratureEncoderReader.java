package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class QuadratureEncoderReader implements Runnable {

    private final DigitalChannel chA;
    private final DigitalChannel chB;
    private final double countsPerRev;

    private volatile boolean running = false;
    private Thread worker;

    // stare live
    private volatile int counts = 0;
    private volatile boolean aState = false;
    private volatile boolean bState = false;
    private volatile double angleDeg = 0.0;

    private volatile double velocityCountsPerSec = 0.0;
    private volatile double velocityDegPerSec = 0.0;

    private volatile long errorCount = 0;
    private volatile int lastInvalidFrom = -1;
    private volatile int lastInvalidTo = -1;
    private volatile int lastDirection = 0; // -1, 0, +1

    private int prevState;
    private long lastVelocityTimeNs;
    private int lastVelocityCounts;

    public QuadratureEncoderReader(DigitalChannel chA, DigitalChannel chB, double countsPerRev) {
        this.chA = chA;
        this.chB = chB;
        this.countsPerRev = countsPerRev;

        boolean a = chA.getState();
        boolean b = chB.getState();

        this.aState = a;
        this.bState = b;
        this.prevState = QuadratureUtil.encodeState(a, b);
        this.lastVelocityTimeNs = System.nanoTime();
        this.lastVelocityCounts = 0;
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
                worker.join(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
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

            if (currState != prevState) {
                int delta = QuadratureUtil.transitionDelta(prevState, currState);

                if (delta != 0) {
                    counts += delta;
                    angleDeg = counts * 360.0 / countsPerRev;
                    lastDirection = delta > 0 ? 1 : -1;
                } else {
                    errorCount++;
                    lastInvalidFrom = prevState;
                    lastInvalidTo = currState;
                }

                prevState = currState;
            }

            long now = System.nanoTime();
            long dtNs = now - lastVelocityTimeNs;

            if (dtNs >= 10_000_000L) { // ~10ms
                int dc = counts - lastVelocityCounts;
                double dt = dtNs * 1e-9;

                velocityCountsPerSec = dc / dt;
                velocityDegPerSec = velocityCountsPerSec * 360.0 / countsPerRev;

                lastVelocityCounts = counts;
                lastVelocityTimeNs = now;
            }

            Thread.yield();
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

        boolean a = chA.getState();
        boolean b = chB.getState();
        aState = a;
        bState = b;
        prevState = QuadratureUtil.encodeState(a, b);

        lastVelocityCounts = 0;
        lastVelocityTimeNs = System.nanoTime();
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
}