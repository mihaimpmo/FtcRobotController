package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

public class RevThroughBoreEncoder {
    private final DcMotorEx encoderMotor;
    private int tickOffset;
    private boolean inverted = false;


    public RevThroughBoreEncoder(DcMotorEx encoderMotor) {
        this(encoderMotor, false);
    }


    public RevThroughBoreEncoder(DcMotorEx encoderMotor, boolean sharedPort) {
        this.encoderMotor = encoderMotor;
        if (!sharedPort) {
            this.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Always ensure RUN_WITHOUT_ENCODER so the hub's motor controller doesn't
        // use the encoder pins for velocity PID (which corrupts steering readings)
        this.encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Record initial position as baseline — homing will set the real zero
        this.tickOffset = encoderMotor.getCurrentPosition();
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public int getRawTicks() {
        int ticks = encoderMotor.getCurrentPosition();
        return inverted ? -ticks : ticks;
    }


    public void setHomePosition(int calibratedTickOffset) {

        this.tickOffset = getRawTicks() - calibratedTickOffset;
    }

    public int getPositionTicks() {
        return getRawTicks() - tickOffset;
    }
    public double getWheelAngleRad() {
        return (getPositionTicks() / (double) SteeringConstants.TICKS_PER_WHEEL_REV) * 2.0 * Math.PI;
    }
}
