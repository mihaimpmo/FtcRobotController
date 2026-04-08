package org.firstinspires.ftc.teamcode;

public class EncoderSnapshot {
    public final int counts;
    public final boolean a;
    public final boolean b;
    public final double angleDeg;
    public final double velocityCountsPerSec;
    public final double velocityDegPerSec;
    public final long errorCount;
    public final int lastInvalidFrom;
    public final int lastInvalidTo;
    public final int lastDirection;

    public EncoderSnapshot(
            int counts,
            boolean a,
            boolean b,
            double angleDeg,
            double velocityCountsPerSec,
            double velocityDegPerSec,
            long errorCount,
            int lastInvalidFrom,
            int lastInvalidTo,
            int lastDirection
    ) {
        this.counts = counts;
        this.a = a;
        this.b = b;
        this.angleDeg = angleDeg;
        this.velocityCountsPerSec = velocityCountsPerSec;
        this.velocityDegPerSec = velocityDegPerSec;
        this.errorCount = errorCount;
        this.lastInvalidFrom = lastInvalidFrom;
        this.lastInvalidTo = lastInvalidTo;
        this.lastDirection = lastDirection;
    }
}