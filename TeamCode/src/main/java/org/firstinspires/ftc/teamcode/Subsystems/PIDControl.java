package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl
{
    private double Kp;
    private double Ki;
    private double Kd;
    private double maxOut=0;
    private double sp=0;
    private double pv=0;
    private double out=0;
    private double err=0;
    private double perr=0;
    private double di=0;
    private double dv=0;
    private double dT=0;
    ElapsedTime tm;
    public PIDControl(double pid_Kp, double pid_Ki, double pid_Kd)
    {
        Kp = pid_Kp;
        Ki = pid_Ki;
        Kd = pid_Kd;
        tm = new ElapsedTime();
        tm.startTime();
        tm.reset();
    }

    void setPID( double pid_Kp, double pid_Ki, double pid_Kd )
    {
        Kp = pid_Kp;
        Ki = pid_Ki;
        Kd = pid_Kd;
    }
    void setSp(double pid_sp){ sp = pid_sp; };
    void setPv( double pid_pv ){ pv = pid_pv;};
    void setMaxOut( double pid_MaxOut ){ maxOut = pid_MaxOut; };

    double calulate()
    {
        dT = tm.milliseconds()/1E3;
        tm.reset();
        // Error
        err = sp - pv;
        // Integral term
        di += err * dT;
        // Derivative term
        dv = (err - perr )/dT;
        perr = err;
        // Calculate total output
        out = ( Kp * err ) + ( Ki*di ) + (Kd * dv );
        // Apply limiter
        out = Math.max( -1, Math.min(out,1));
        return out;
    }

    public double calculate(double pid_sp, double pid_pv)
    {
        sp = pid_sp; pv = pid_pv;
        dT = tm.milliseconds()/1E3;
        tm.reset();
        // Error
        err = sp - pv;
        // Integral term
        di += err * dT;
        // Derivative term
        dv = (err - perr )/dT;
        perr = err;
        // Calculate total output
        out = ( Kp * err ) + ( Ki*di ) + (Kd * dv );
        // Apply limiter
        //out = Math.max( -1, Math.min(out,1));
        return out;
    }
}