package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.ServoCfg;

public class Outtake {
    private Hardware hardware;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private PIDController pidController1;
    private PIDController pidController2;
    private ServoCfg ramp;

    private enum RampState { IDLE, SHOOTING, HOLDING }
    private RampState rampState = RampState.IDLE;
    private boolean lastShootButton = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_HOLD_TIME = 0.5;

    private double targetRPM = 0.0;
    private double currentRPM1 = 0.0;
    private double currentRPM2 = 0.0;
    private double motorPower1 = 0.0;
    private double motorPower2 = 0.0;
    private boolean isActive = false;

    public Outtake(Hardware hardware) {
        this.hardware = hardware;
        this.motor1 = hardware.outtakeHardware.WheelMotor1;
        this.motor2 = hardware.outtakeHardware.WheelMotor2;

        ramp = new ServoCfg(hardware.outtakeHardware.RampServo, 2);
        ramp.setRange(OuttakeConstants.RAMP_MIN, OuttakeConstants.RAMP_MAX);
        ramp.moveTo(OuttakeConstants.RAMP_IDLE);  // Initialize to IDLE position

        this.pidController1 = new PIDController(
            OuttakeConstants.VELOCITY_P,
            OuttakeConstants.VELOCITY_I,
            OuttakeConstants.VELOCITY_D
        );

        this.pidController2 = new PIDController(
            OuttakeConstants.VELOCITY_P,
            OuttakeConstants.VELOCITY_I,
            OuttakeConstants.VELOCITY_D
        );
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.max(0, Math.min(rpm, OuttakeConstants.MAX_MOTOR_RPM));
        this.isActive = (this.targetRPM > 0);
    }

    public void update() {
        if (!isActive) {
            motor1.setPower(0);
            motor2.setPower(0);
            motorPower1 = 0;
            motorPower2 = 0;
            currentRPM1 = 0;
            currentRPM2 = 0;
            return;
        }

        double targetTPS = (targetRPM / 60.0) * OuttakeConstants.MOTOR_TICKS_PER_REV;
        double feedforward = targetTPS * OuttakeConstants.VELOCITY_FF;

        // Motor 1
        double velocityTPS1 = motor1.getVelocity();
        currentRPM1 = (velocityTPS1 / OuttakeConstants.MOTOR_TICKS_PER_REV) * 60.0;
        motorPower1 = pidController1.calculate(targetRPM, currentRPM1) + feedforward;
        motorPower1 = Math.max(-1.0, Math.min(motorPower1, 1.0));
        if (motorPower1 > 0 && motorPower1 < OuttakeConstants.MIN_POWER) {
            motorPower1 = OuttakeConstants.MIN_POWER;
        }
        motor1.setPower(motorPower1);

        // Motor 2
        double velocityTPS2 = motor2.getVelocity();
        currentRPM2 = (velocityTPS2 / OuttakeConstants.MOTOR_TICKS_PER_REV) * 60.0;
        motorPower2 = pidController2.calculate(targetRPM, currentRPM2) + feedforward;
        motorPower2 = Math.max(-1.0, Math.min(motorPower2, 1.0));
        if (motorPower2 > 0 && motorPower2 < OuttakeConstants.MIN_POWER) {
            motorPower2 = OuttakeConstants.MIN_POWER;
        }
        motor2.setPower(motorPower2);

        ramp.execute();
    }

    /** Auto-shoot: button press → SHOOT → hold → return to IDLE */
    public void rampShoot(boolean shootButton) {
        boolean buttonPressed = shootButton && !lastShootButton;
        lastShootButton = shootButton;

        switch (rampState) {
            case IDLE:
                if (buttonPressed) {
                    ramp.moveTo(OuttakeConstants.RAMP_SHOOT);
                    rampState = RampState.SHOOTING;
                }
                break;
            case SHOOTING:
                if (ramp.isReady() && isAtTargetSpeed()) {
                    shootTimer.reset();
                    rampState = RampState.HOLDING;
                }
                break;
            case HOLDING:
                if (shootTimer.seconds() >= SHOOT_HOLD_TIME) {
                    ramp.moveTo(OuttakeConstants.RAMP_IDLE);
                    rampState = RampState.IDLE;
                }
                break;
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

    public boolean isAtTargetSpeed() {
        boolean motor1AtTarget = Math.abs(targetRPM - currentRPM1) < OuttakeConstants.RPM_TOLERANCE;
        boolean motor2AtTarget = Math.abs(targetRPM - currentRPM2) < OuttakeConstants.RPM_TOLERANCE;
        return motor1AtTarget && motor2AtTarget;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM1() {
        return currentRPM1;
    }

    public double getCurrentRPM2() {
        return currentRPM2;
    }

    public double getAverageRPM() {
        return (currentRPM1 + currentRPM2) / 2.0;
    }

    public double getMotorPower1() {
        return motorPower1;
    }

    public double getMotorPower2() {
        return motorPower2;
    }

    public double getError1() {
        return targetRPM - currentRPM1;
    }

    public double getError2() {
        return targetRPM - currentRPM2;
    }

    public boolean isActive() {
        return isActive;
    }

    public String getRampState() { return rampState.toString(); }
    public boolean isRampReady() { return rampState == RampState.IDLE; }

    public void resetRamp() {
        rampState = RampState.IDLE;
        ramp.moveTo(OuttakeConstants.RAMP_IDLE);
    }
}
