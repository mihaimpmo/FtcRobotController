package org.firstinspires.ftc.teamcode.Utils;

public class MathUtils {
    private static final double TWO_PI = 2.0 * Math.PI;

    public static double wrapDeg(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }

    public static double wrapRad(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) angle += TWO_PI;
        if (angle > Math.PI) angle -= TWO_PI;
        return angle;
    }

    public static double angleDiff(double from, double to) {
        return wrapRad(to - from);
    }

    public static double angleDiff90(double fromRad, double toRad) {
        double diff = toRad - fromRad;
        diff = diff % Math.PI;
        if (diff > Math.PI / 2) diff -= Math.PI;
        if (diff < -Math.PI / 2) diff += Math.PI;
        return diff;
    }

    public static double wrap2Pi(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) angle += TWO_PI;
        return angle;
    }

    public static double wrap180(double angleDegrees) {
        double wrapped = angleDegrees % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }

    public static double wrap90(double angleDegrees) {
        double wrapped = angleDegrees % 180.0;
        if (wrapped >= 90.0) wrapped -= 180.0;
        if (wrapped <= -90.0) wrapped += 180.0;
        return wrapped;
    }
}
