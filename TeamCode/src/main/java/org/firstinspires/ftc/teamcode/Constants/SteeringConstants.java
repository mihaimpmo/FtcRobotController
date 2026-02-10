package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SteeringConstants {
    public static final double SERVO_TO_WHEEL_RATIO = 2.0;

    public static final double MIN_VOLTAGE = 0.0;
    public static final double MAX_VOLTAGE = 3.3;

    public static final double FL_VOLTAGE_OFFSET = 2.964;
    public static final double FR_VOLTAGE_OFFSET = 3.110;
    public static final double BL_VOLTAGE_OFFSET = 0.982;
    public static final double BR_VOLTAGE_OFFSET = 2.980;

    public static double STEER_P = 0.65;
    public static double STEER_I = 0.0;

    public static double STEER_D = 0.0;

    public static double STEERING_DEADBAND_RADIANS = 0.01;
    public static double MIN_SERVO_POWER = 0.01;
    public static double STATIC_FRICTION_COMPENSATION = 0.03;

    public static double ENCODER_FILTER_ALPHA = 0;
}
