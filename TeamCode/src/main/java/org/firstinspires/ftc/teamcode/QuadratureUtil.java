package org.firstinspires.ftc.teamcode;

public final class QuadratureUtil {

    private QuadratureUtil() { }

    // index = (prev << 2) | curr
    private static final int[] QUAD_TABLE = {
            0, -1, +1,  0,
            +1,  0,  0, -1,
            -1,  0,  0, +1,
            0, +1, -1,  0
    };

    public static int encodeState(boolean a, boolean b) {
        return ((a ? 1 : 0) << 1) | (b ? 1 : 0);
    }

    public static int transitionDelta(int prevState, int currState) {
        return QUAD_TABLE[(prevState << 2) | currState];
    }

    public static String stateToString(int state) {
        switch (state) {
            case 0: return "00";
            case 1: return "01";
            case 2: return "10";
            case 3: return "11";
            default: return "--";
        }
    }

    public static String transitionToString(int from, int to) {
        if (from < 0 || to < 0) return "none";
        return stateToString(from) + " -> " + stateToString(to);
    }

    public static String directionToString(int dir) {
        if (dir > 0) return "forward";
        if (dir < 0) return "reverse";
        return "unknown";
    }
}