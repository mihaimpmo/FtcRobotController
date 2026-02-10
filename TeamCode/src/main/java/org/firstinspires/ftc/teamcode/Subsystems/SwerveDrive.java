package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;

public class SwerveDrive {
    public final SwerveModule fl, fr, bl, br;

    public SwerveDrive(HardwareMap hardwareMap) {
        fl = createModule(hardwareMap, "fl", SteeringConstants.FL_VOLTAGE_OFFSET, true, true);
        fr = createModule(hardwareMap, "fr", SteeringConstants.FR_VOLTAGE_OFFSET, true, true);

        bl = createModule(hardwareMap, "bl", SteeringConstants.BL_VOLTAGE_OFFSET, true, true);
        br = createModule(hardwareMap, "br", SteeringConstants.BR_VOLTAGE_OFFSET, true, true);
    }

    private SwerveModule createModule(
            HardwareMap hardwareMap,
            String name,
            double voltageOffset,
            boolean driveInverted,
            boolean steerInverted
    ) {
        return new SwerveModule(
                hardwareMap.get(DcMotorEx.class, name),
                hardwareMap.get(CRServo.class, name + "_servo"),
                new AxonEncoder(
                        hardwareMap.get(AnalogInput.class, name + "_enc"),
                        voltageOffset
                ),
                driveInverted,
                steerInverted,
                name.toUpperCase()
        );
    }

    public void drive(double fwd, double str, double rot) {
        double[][] pos = DriveConstants.WHEEL_POS;
        SwerveModule[] mod = {fl, fr, bl, br};

        // Scale rot so full stick = full motor speed
        double wheelDist = Math.sqrt(pos[0][0] * pos[0][0] + pos[0][1] * pos[0][1]);
        rot /= wheelDist;

        // When rotating, disable translation to prevent flip-causing combined angles
        if (Math.abs(rot) > 0.01) {
            fwd = 0;
            str = 0;
        }
        if (Math.abs(fwd)> 0.01 || Math.abs(str) > 0.01){
            rot =0;
        }

        double max = 0;
        double[] ang = new double[4];
        double[] spd = new double[4];

        for (int i = 0; i < 4; i++) {
            double vx = fwd + (-rot * pos[i][1]);
            double vy = str + ( rot * pos[i][0]);
            ang[i] = Math.atan2(vy, vx);
            spd[i] = Math.sqrt(vx * vx + vy * vy);
            if (spd[i] > max) max = spd[i];
        }
        if (max > 1.0) for (int i = 0; i < 4; i++) spd[i] /= max;
        for (int i = 0; i < 4; i++) mod[i].set(ang[i], spd[i]);
    }

    public void hold() {
        fl.hold();
        fr.hold();
        bl.hold();
        br.hold();
    }

    public void zero() {
        fl.set(0, 0);
        fr.set(0, 0);
        bl.set(0, 0);
        br.set(0, 0);
    }

    public void log(Telemetry telemetry) {
        fl.log(telemetry);
        fr.log(telemetry);
        bl.log(telemetry);
        br.log(telemetry);
    }
}
