package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoCfg {
    private Servo servo;
    private enum ServoState { servoReady, servoMove }
    private String gIndex;
    public ServoState state = ServoState.servoReady;
    public double actPos;
    public double trgPos;
    private double prevPos;
    private double defSpeed;
    private double minRange;
    private double maxRange;
    private final double IN_WINDOW = 2;
    private ElapsedTime tm;

    ServoCfg(Servo armServo, String rcGIndex, double definedSpeed) {
        servo = armServo;
        gIndex = rcGIndex;
        defSpeed = definedSpeed;
        tm = new ElapsedTime();
        tm.startTime();
    }

    public ServoCfg(Servo armServo, double definedSpeed) {
        servo = armServo;
        gIndex = "";
        defSpeed = definedSpeed;
        tm = new ElapsedTime();
        tm.startTime();
    }

    public void setRange(double in_minRange, double in_maxRange) {
        minRange = in_minRange;
        maxRange = in_maxRange;
    }

    public double mapRange(double inValue) {
        return ((inValue - minRange) * 1) / (maxRange - minRange) + 0;
    }

    double getActPos() {
        return (servo.getPosition() * (maxRange - minRange)) + minRange;
    }

    public void moveTo(double newPosition) {
        servo.setPosition(mapRange(newPosition));
        trgPos = newPosition;
        state = ServoState.servoMove;
        tm.reset();
    }

    public void moveTo(String newPosition) {
        if (newPosition == null || newPosition == "NOP") return;
        double numPosition = Double.parseDouble(newPosition);
        servo.setPosition(mapRange(numPosition));
        trgPos = numPosition;
        state = ServoState.servoMove;
        tm.reset();
    }

    public void execute() {
        if (state == ServoState.servoMove) {
            double dt = tm.milliseconds() / 1000;
            if (dt > Math.abs(trgPos - prevPos) / defSpeed) {
                prevPos = trgPos;
                state = ServoState.servoReady;
            }
        }
    }

    public void stop() {
        servo.setPosition(servo.getPosition());
    }

    public String getIndex() { return gIndex; }
    public boolean isReady() { return state == ServoState.servoReady; }
    public boolean notReady() { return state != ServoState.servoReady; }

    public boolean inPosition(double chkPosition) {
        return Math.abs(chkPosition - getActPos()) < IN_WINDOW;
    }
}
