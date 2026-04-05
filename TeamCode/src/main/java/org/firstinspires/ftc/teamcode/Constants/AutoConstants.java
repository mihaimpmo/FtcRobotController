package org.firstinspires.ftc.teamcode.Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoConstants {
    private AutoConstants() {}

    // Pinpoint pod offsets relative to the robot tracking point.
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;

    // Motion tuning.
    public static double MAX_TRANSLATION_SPEED = 2.5;
    public static double MAX_ROTATION_SPEED = 1.8;
    public static double MIN_TRANSLATION_SPEED = 0.18;
    public static double MIN_ROTATION_SPEED = 0.22;

    public static double DRIVE_P = 0.12;
    public static double STRAFE_P = 0.12;
    public static double CROSS_AXIS_P = 0.05;
    public static double HEADING_P = 0.035;
    public static double TURN_P = 0.025;

    public static double DECEL_DISTANCE_CM = 25.0;
    public static double DECEL_ANGLE_DEG = 18.0;

    public static double POSITION_TOLERANCE_CM = 1.5;
    public static double CROSS_AXIS_TOLERANCE_CM = 1.25;
    public static double HEADING_TOLERANCE_DEG = 2.0;

    public static long SETTLE_MS = 150;
    public static long LOOP_TIMEOUT_MS = 5000;

    // Sign conventions. Keep these tunable because the current swerve implementation
    // may need sign flips compared to the Pinpoint coordinate frame.
    public static double FORWARD_COMMAND_SIGN = -1.0;
    public static double STRAFE_COMMAND_SIGN = -1.0;
    public static double ROTATION_COMMAND_SIGN = -1.0;
}
