package org.firstinspires.ftc.teamcode.Utils;

public class Constants {
    public double h1=0.185; //april tag height
    public double h2=0.48; //camera height on the robot
    public double a=0; //camera angle relative to the Ox axis
    public double pi=3.14159265358979323846; //pi
    public double kp_turn=0;
    public double ki_turn=0;
    public double kd_turn=0;
    public double deg2rad(double x) {
        return x*pi/180;
    }

}
