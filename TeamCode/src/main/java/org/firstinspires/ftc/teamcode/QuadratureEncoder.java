package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class QuadratureEncoder implements Runnable {

    private final DigitalChannel pinA;
    private final DigitalChannel pinB;
    private final int cpr;

    private volatile boolean running = false;
    private Thread worker;

    private volatile long count = 0;
    private volatile int state = 0;
    private volatile boolean a = false;
    private volatile boolean b = false;

    private volatile double angleDeg = 0.0;
    private volatile double velocityCountsPerSec = 0.0;
    private volatile double velocityDegPerSec = 0.0;

    private volatile long errorCount = 0;
    private volatile int lastInvalidFrom = -1;
    private volatile int lastInvalidTo = -1;
    private volatile int lastDirection = 0; // +1, -1, 0

    // viteza la ultima eroare
    private volatile double lastErrorVelocityCountsPerSec = 0.0;
    private volatile double lastErrorVelocityDegPerSec = 0.0;

    // viteza minima la care a aparut o eroare
    private volatile double minErrorVelocityCountsPerSec = Double.POSITIVE_INFINITY;
    private volatile double minErrorVelocityDegPerSec = Double.POSITIVE_INFINITY;

    private int lastState;
    private long lastVelocityTimeNs;
    private long lastVelocityCount;

    public QuadratureEncoder(DigitalChannel pinA, DigitalChannel pinB, int cpr) {
        this.pinA = pinA;
        this.pinB = pinB;
        this.cpr = cpr;

        boolean startA = pinA.getState();
        boolean startB = pinB.getState();

        this.a = startA;
        this.b = startB;
        this.state = EncoderUtil.encodeState(startA, startB);
        this.lastState = this.state;

        this.lastVelocityTimeNs = System.nanoTime();
        this.lastVelocityCount = 0;
    }

    public void start() {
        if (running) return;

        running = true;
        worker = new Thread(this, "QuadratureEncoderThread");
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
    }

    @Override
    public void run() {
        while (running) {
            boolean currentA = pinA.getState();
            boolean currentB = pinB.getState();

            a = currentA;
            b = currentB;

            int currentState = EncoderUtil.encodeState(currentA, currentB);
            state = currentState;

            if (currentState != lastState) {
                int delta = EncoderUtil.transitionDelta(lastState, currentState);

                if (delta == 1) {
                    count++;
                    lastDirection = 1;
                } else if (delta == -1) {
                    count--;
                    lastDirection = -1;
                } else {
                    errorCount++;
                    lastInvalidFrom = lastState;
                    lastInvalidTo = currentState;

                    // salveaza viteza la ultima eroare
                    lastErrorVelocityCountsPerSec = velocityCountsPerSec;
                    lastErrorVelocityDegPerSec = velocityDegPerSec;

                    // viteza minima la care a aparut eroarea
                    double absCountsVel = Math.abs(velocityCountsPerSec);
                    double absDegVel = Math.abs(velocityDegPerSec);

                    if (absCountsVel < minErrorVelocityCountsPerSec) {
                        minErrorVelocityCountsPerSec = absCountsVel;
                    }

                    if (absDegVel < minErrorVelocityDegPerSec) {
                        minErrorVelocityDegPerSec = absDegVel;
                    }
                }

                angleDeg = (count * 360.0) / cpr;

                long nowNs = System.nanoTime();
                long dtNs = nowNs - lastVelocityTimeNs;

                // actualizare viteză la ~5 ms
                if (dtNs >= 5_000_000L) {
                    long dCount = count - lastVelocityCount;
                    double dtSec = dtNs / 1e9;

                    velocityCountsPerSec = dCount / dtSec;
                    velocityDegPerSec = velocityCountsPerSec * 360.0 / cpr;

                    lastVelocityCount = count;
                    lastVelocityTimeNs = nowNs;
                }

                lastState = currentState;
            }

            Thread.yield();
        }
    }

    public long getCount() {
        return count;
    }

    public int getState() {
        return state;
    }

    public boolean getA() {
        return a;
    }

    public boolean getB() {
        return b;
    }

    public double getAngleDeg() {
        return angleDeg;
    }

    public double getVelocityCountsPerSec() {
        return velocityCountsPerSec;
    }

    public double getVelocityDegPerSec() {
        return velocityDegPerSec;
    }

    public double getLastErrorVelocityCountsPerSec() {
        return lastErrorVelocityCountsPerSec;
    }

    public double getLastErrorVelocityDegPerSec() {
        return lastErrorVelocityDegPerSec;
    }

    public double getMinErrorVelocityCountsPerSec() {
        if (Double.isInfinite(minErrorVelocityCountsPerSec)) {
            return 0.0;
        }
        return minErrorVelocityCountsPerSec;
    }

    public double getMinErrorVelocityDegPerSec() {
        if (Double.isInfinite(minErrorVelocityDegPerSec)) {
            return 0.0;
        }
        return minErrorVelocityDegPerSec;
    }

    public long getErrorCount() {
        return errorCount;
    }

    public String getLastInvalidTransitionString() {
        if (lastInvalidFrom < 0 || lastInvalidTo < 0) {
            return "none";
        }
        return EncoderUtil.stateToString(lastInvalidFrom) + " -> " +
                EncoderUtil.stateToString(lastInvalidTo);
    }

    public String getLastDirectionString() {
        if (lastDirection > 0) return "CW";
        if (lastDirection < 0) return "CCW";
        return "none";
    }
}