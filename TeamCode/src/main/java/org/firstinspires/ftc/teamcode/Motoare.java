package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Motoare {
    void power(DcMotor m, double power) {
        m.setPower(power);
    }
    void stop(DcMotor m) {
        m.setPower(0);
    }
    void runFor(DcMotor m, double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.time() < time) {
            m.setPower(power);
        }
        m.setPower(0);
    }
}
