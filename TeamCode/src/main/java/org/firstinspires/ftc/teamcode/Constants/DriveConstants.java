package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {
    public static double WHEELBASE_METERS = 0.385;
    public static double TRACK_WIDTH_METERS = 0.385;
    public static double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    public static double DRIVE_GEAR_RATIO = 13.7;
    public static final double MOTOR_TICKS_PER_REV = 384.5;
    public static final double TICKS_PER_METER = (MOTOR_TICKS_PER_REV * DRIVE_GEAR_RATIO) / WHEEL_CIRCUMFERENCE_METERS;

    public static double DRIVE_FF = 1.0;
}
