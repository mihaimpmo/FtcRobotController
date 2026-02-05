package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

@Config
public class DriveConstants {
    // Robot geometry
    public static double WHEELBASE_METERS = 0.385;
    public static double TRACK_WIDTH_METERS = 0.385;

    // Module positions - +X forward, +Y left
    // FL-FR at front, BL-BR at back (Y signs flipped for correct spin direction)
    public static final Translation2d BL_POSITION = new Translation2d(-0.1925, 0.1925);   // front-left
    public static final Translation2d BR_POSITION = new Translation2d(-0.1925, -0.1925);    // front-right
    public static final Translation2d FR_POSITION = new Translation2d(-0.1925, -0.1925);  // back-left
    public static final Translation2d FL_POSITION = new Translation2d(-0.1925, 0.1925);   // back-right

    // Wheel specs
    public static double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    // Motor specs
    public static double DRIVE_GEAR_RATIO = 13.7;
    public static final double MOTOR_TICKS_PER_REV = 384.5;
    public static final double TICKS_PER_METER =
            (MOTOR_TICKS_PER_REV * DRIVE_GEAR_RATIO) / WHEEL_CIRCUMFERENCE_METERS;

    // Speed limits
    public static double MAX_SPEED_METERS_PER_SECOND = 4.0;
    public static double MAX_ANGULAR_VELOCITY = 4 * Math.PI;

    // Drive control (feedforward only, no velocity PID)
    public static double DRIVE_P = 0.0;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0.0;
    public static double DRIVE_FF = 1.0;
}
