package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.ServoCfg;

public class Outtake {

    private Hardware hardware;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private ServoCfg ramp;

    private enum RampState { IDLE, SHOOT }
    private RampState rampState = RampState.IDLE;
    private boolean lastShootButton = false;

    private double targetRPM = 0.0;
    private double motorPower = 0.0;
    private boolean isActive = false;

    public Outtake(Hardware hardware) {
        this.hardware = hardware;
        this.motor1 = hardware.outtakeHardware.WheelMotor1;
        this.motor2 = hardware.outtakeHardware.WheelMotor2;

        ramp = new ServoCfg(hardware.outtakeHardware.RampServo, 2);
        ramp.setRange(OuttakeConstants.RAMP_MIN, OuttakeConstants.RAMP_MAX);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0, Math.min(rpm, OuttakeConstants.MAX_MOTOR_RPM));
        isActive = targetRPM > 0;
    }

    public void update() {
        if (!isActive) {
            motor1.setPower(0);
            motor2.setPower(0);
            motorPower = 0;
            ramp.execute();
            return;
        }

        // Open-loop feedforward only — encoder pins are shared with swerve steering encoders,
        // so motor velocity feedback is not available.
        double targetTPS = (targetRPM / 60.0) * OuttakeConstants.MOTOR_TICKS_PER_REV;
        motorPower = targetTPS * OuttakeConstants.VELOCITY_FF;
        motorPower = Math.max(0, Math.min(motorPower, 1.0));
        if (motorPower > 0 && motorPower < OuttakeConstants.MIN_POWER) {
            motorPower = OuttakeConstants.MIN_POWER;
        }

        motor1.setPower(motorPower);
        motor2.setPower(motorPower);

        ramp.execute();
    }

    public void rampShoot(boolean shootButton) {
        boolean pressed = shootButton && !lastShootButton;
        lastShootButton = shootButton;

        if (!pressed) return;

        if (rampState == RampState.IDLE) {
            ramp.moveTo(OuttakeConstants.RAMP_SHOOT);
            rampState = RampState.SHOOT;
        } else {
            ramp.moveTo(OuttakeConstants.RAMP_IDLE);
            rampState = RampState.IDLE;
        }
    }

    public void stop() {
        isActive = false;
        targetRPM = 0;
        motor1.setPower(0);
        motor2.setPower(0);
        rampState = RampState.IDLE;
        ramp.moveTo(OuttakeConstants.RAMP_IDLE);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getMotorPower() {
        return motorPower;
    }

    public boolean isActive() {
        return isActive;
    }

    public String getRampState() {
        return rampState.toString();
    }

    public boolean isRampReady() {
        return rampState == RampState.IDLE;
    }

    public void resetRamp() {
        rampState = RampState.IDLE;
        ramp.moveTo(OuttakeConstants.RAMP_IDLE);
    }

    public void log(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addLine("--- Outtake ---");
        telemetry.addData("  Target RPM", "%.0f", targetRPM);
        telemetry.addData("  Motor Power", "%.2f", motorPower);
        telemetry.addData("  Ramp State", "%s", rampState);
    }
}
