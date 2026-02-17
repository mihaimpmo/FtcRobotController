package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeHardware {
    private static final String Wheel_1 = "w1";
    private static final String Wheel_2 = "w2";
    private static final String Ramp_servo = "rs";

    public final DcMotorEx WheelMotor1;
    public final DcMotorEx WheelMotor2;
    public Servo RampServo;

    public OuttakeHardware(HardwareMap hardwareMap) {
        WheelMotor1 = hardwareMap.get(DcMotorEx.class, Wheel_1);
        WheelMotor2 = hardwareMap.get(DcMotorEx.class, Wheel_2);
        RampServo = hardwareMap.get(Servo.class, Ramp_servo);

        WheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // RUN_WITHOUT_ENCODER: encoder pins on these ports are used by swerve steering encoders,
        // so we cannot use encoder-based velocity control for the outtake motors.
        WheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
