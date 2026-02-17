package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

public class RevThroughBoreEncoder {
    private final DcMotorEx encoderMotor;
    private int tickOffset;

    /**
     * Create an encoder on a dedicated (empty) motor port.
     * Resets the encoder count to 0 on construction.
     */
    public RevThroughBoreEncoder(DcMotorEx encoderMotor) {
        this(encoderMotor, false);
    }

    /**
     * @param encoderMotor  the motor port the encoder is plugged into
     * @param sharedPort    true if this port is also driving a motor (outtake/intake).
     *                      When true, we only read the encoder — we do NOT reset it
     *                      or change the motor's run mode.
     */
    public RevThroughBoreEncoder(DcMotorEx encoderMotor, boolean sharedPort) {
        this.encoderMotor = encoderMotor;
        if (!sharedPort) {
            this.encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Record initial position as baseline — homing will set the real zero
        this.tickOffset = encoderMotor.getCurrentPosition();
    }

    public int getRawTicks() {
        return encoderMotor.getCurrentPosition();
    }

    /**
     * Set the encoder zero point during homing.
     * After the limit switch triggers, call this with the calibrated tick offset
     * so that 0 ticks = wheels pointing forward.
     */
    public void setHomePosition(int calibratedTickOffset) {
        this.tickOffset = getRawTicks() + calibratedTickOffset;
    }

    public int getPositionTicks() {
        return getRawTicks() - tickOffset;
    }
    public double getWheelAngleRad() {
        return (getPositionTicks() / (double) SteeringConstants.TICKS_PER_WHEEL_REV) * 2.0 * Math.PI;
    }
}
