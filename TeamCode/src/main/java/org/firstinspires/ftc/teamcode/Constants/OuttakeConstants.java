package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class OuttakeConstants {
    public static double RAMP_MAX = 355;
    public static  double RAMP_MIN = 50;
    public static  double RAMP_IDLE = 95;
    public static double RAMP_SHOOT = 120;

    public static  double MAX_MOTOR_RPM = 6000.0;
    public static  double MOTOR_TICKS_PER_REV = 28.0;

    public static double TARGET_RPM = 3900;
    public static double DEFAULT_TO_MAX = 1000;

    public static  double VELOCITY_FF = 1.0 / ((MAX_MOTOR_RPM / 60.0) * MOTOR_TICKS_PER_REV);
    public static  double MIN_POWER = 0.05;

}
