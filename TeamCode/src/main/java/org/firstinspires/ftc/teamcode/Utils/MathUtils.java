package org.firstinspires.ftc.teamcode.Utils;

public class MathUtils {
    private static final double TWO_PI = 2.0 * Math.PI;

    /** Normalizes angle to [-180, 180] degrees */
    public static double normalizeAngleDegrees(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }

    /** Normalizes angle to [-π, π] radians */
    public static double normalizeAngleRadians(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) angle += TWO_PI;
        if (angle > Math.PI) angle -= TWO_PI;
        return angle;
    }

    /** Shortest angular distance from 'from' to 'to' in [-π, π] radians (360° period) */
    public static double shortestAngularDistance(double from, double to) {
        return normalizeAngleRadians(to - from);
    }

    /**
     * Shortest angular distance with 180° (π) period.
     * For 2:1 gear ratio where +90° and -90° are the SAME physical position.
     * Returns error in [-π/2, π/2] radians.
     */
    public static double shortestAngularDistance90(double fromRad, double toRad) {
        double diff = toRad - fromRad;
        // Wrap to [-π/2, π/2] with π period
        diff = diff % Math.PI;
        if (diff > Math.PI / 2) diff -= Math.PI;
        if (diff < -Math.PI / 2) diff += Math.PI;
        return diff;
    }

    /** Normalizes angle to [0, 2π) radians */
    public static double normalizeAngle0To2Pi(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) angle += TWO_PI;
        return angle;
    }

    /** Wraps angle to [-180, 180] degrees */
    public static double wrap180(double angleDegrees) {
        double wrapped = angleDegrees % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }

    /**
     * Wraps angle to [-90, +90] degrees with 180° period.
     * For 2:1 gear ratio where encoder only gives 180° unique info.
     * This ensures measured angle stays in same range as target (±90°).
     */
    public static double wrap90(double angleDegrees) {
        double wrapped = angleDegrees % 180.0;
        if (wrapped >= 90.0) wrapped -= 180.0;
        if (wrapped <= -90.0) wrapped += 180.0;
        return wrapped;
    }
}
