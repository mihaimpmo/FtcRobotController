package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SteeringConstants {
    public static final double SERVO_TO_WHEEL_RATIO = 2.0;

    // Rev Through Bore encoder: 8192 ticks per encoder shaft revolution
    public static final int ENCODER_TICKS_PER_REV = 8192;
    // Ticks per wheel revolution (encoder on servo shaft, 2:1 to wheel)
    public static final int TICKS_PER_WHEEL_REV = (int) (ENCODER_TICKS_PER_REV * SERVO_TO_WHEEL_RATIO);

    // Encoder motor port names in the hardware map
    // These are the expansion hub motor ports where each Rev Through Bore encoder is plugged in.
    // FL, FR, BR share ports with intake/outtake motors; BL has a dedicated port.
    public static final String FL_ENCODER_NAME = "IM1";
    public static final String FR_ENCODER_NAME = "w2";
    public static final String BL_ENCODER_NAME = "bl_enc";
    public static final String BR_ENCODER_NAME = "w1";

    // Which encoder ports are shared with another motor (intake/outtake)
    public static final boolean FL_ENCODER_SHARED = true;
    public static final boolean FR_ENCODER_SHARED = true;
    public static final boolean BL_ENCODER_SHARED = false;
    public static final boolean BR_ENCODER_SHARED = true;

    // Per-module tick offsets: ticks from limit-switch home to "wheels forward"
    // Set these by running the SwerveCalibration opmode
    public static int FL_TICK_OFFSET = 2501;
    public static int FR_TICK_OFFSET = -306;
    public static int BL_TICK_OFFSET = 257;
    public static int BR_TICK_OFFSET = 2879;

    // Limit switch hardware map names
    public static final String FL_SWITCH_NAME = "flSwitch";
    public static final String FR_SWITCH_NAME = "frSwitch";
    public static final String BL_SWITCH_NAME = "blSwitch";
    public static final String BR_SWITCH_NAME = "brSwitch";

    // Homing — dual-stage (fast approach, back off, slow approach)
    public static double HOMING_FAST_POWER = 0.6;
    public static double HOMING_SLOW_POWER = 0.2;
    public static double HOMING_BACKOFF_POWER = -0.3;
    public static int HOMING_BACKOFF_MS = 300;
    public static int HOMING_TIMEOUT_MS = 10000;
    // Limit switch polarity: false = pressed for active-low, true = pressed for active-high
    public static boolean LIMIT_SWITCH_ACTIVE_STATE = false;

    // Steering PD
    public static double STEER_P = 0.8;
    public static double STEER_I = 0.0;
    public static double STEER_D = 0;

    public static double STEERING_DEADBAND_RADIANS = 0.01;
    public static double MIN_SERVO_POWER = 0.05;
    public static double STATIC_FRICTION_COMPENSATION = 0.0;
}
