package org.firstinspires.ftc.teamcode;

public final class EncoderUtil {

    private EncoderUtil() {
    }

    public static int encodeState(boolean a, boolean b) {
        int ai = a ? 1 : 0;
        int bi = b ? 1 : 0;
        return (ai << 1) | bi;
    }

    public static int transitionDelta(int lastState, int currentState) {
        // sens 1
        if ((lastState == 0 && currentState == 1) ||
                (lastState == 1 && currentState == 3) ||
                (lastState == 3 && currentState == 2) ||
                (lastState == 2 && currentState == 0)) {
            return 1;
        }

        // sens 2
        if ((lastState == 0 && currentState == 2) ||
                (lastState == 2 && currentState == 3) ||
                (lastState == 3 && currentState == 1) ||
                (lastState == 1 && currentState == 0)) {
            return -1;
        }

        return 0;
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
}