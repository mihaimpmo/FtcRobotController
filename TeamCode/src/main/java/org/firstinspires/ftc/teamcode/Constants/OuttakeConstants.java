package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config

public class OuttakeConstants {
    public static final double RAMP_MAX = 355;
    public static final double RAMP_MIN = 0;
    public static final double RAMP_IDLE = 92.5;
    public static final double RAMP_SHOOT = 127.5;

    public static final double MAX_MOTOR_RPM = 6000.0;
    public static final double MOTOR_TICKS_PER_REV = 28.0;

    public static final double TARGET_RPM = 3200;

    public static final double VELOCITY_P = 0.0002;
    public static final double VELOCITY_I = 0.00001;
    public static final double VELOCITY_D = 0.00001;
    public static final double VELOCITY_FF = 1.0 / ((MAX_MOTOR_RPM / 60.0) * MOTOR_TICKS_PER_REV);
    public static final double MAX_INTEGRAL = 0.1;

    public static final double RPM_TOLERANCE = 15.0;
    public static final double MIN_POWER = 0.05;

}
