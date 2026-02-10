package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class SwerveModule {
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AxonEncoder encoder;
    private final boolean driveInverted;
     private final boolean steerInverted;
    private final String moduleName;

    private double targetAngle;
    private double heldAngle;
    private double lastError = 0.0;
    private long lastTime = System.nanoTime();

    public SwerveModule(
            DcMotorEx driveMotor,
            CRServo steerServo,
            AxonEncoder encoder,
            boolean driveInverted,
            boolean steerInverted,
            String moduleName
    ) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.encoder = encoder;
        this.driveInverted = driveInverted;
        this.steerInverted = steerInverted;
        this.moduleName = moduleName;

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        heldAngle = encoder.getWheelAngle();
        targetAngle = heldAngle;

        steerServo.setPower(0);
    }

    public double getAngle() {
        return encoder.getWheelAngle();
    }

    public void set(double angle, double speed) {
        if (Math.abs(speed) < 0.001) {
            angle = heldAngle;
        } else {
            heldAngle = angle;
        }

        double rawTargetRad = MathUtils.wrapRad(angle);
        double currentAngle = getAngle();

        final double RIGHT_ANGLE = Math.PI / 2;
        final double NEAR_90_TOLERANCE = Math.toRadians(5.0);

        double targetRad = rawTargetRad;
        boolean motorFlipped = false;

        boolean nearPositive90 = Math.abs(rawTargetRad - RIGHT_ANGLE) < NEAR_90_TOLERANCE;
        boolean nearNegative90 = Math.abs(rawTargetRad + RIGHT_ANGLE) < NEAR_90_TOLERANCE;

        if (nearPositive90 || nearNegative90) {
            double pos90 = RIGHT_ANGLE;
            double neg90 = -RIGHT_ANGLE;

            while (pos90 - currentAngle > Math.PI) pos90 -= 2 * Math.PI;
            while (pos90 - currentAngle < -Math.PI) pos90 += 2 * Math.PI;
            while (neg90 - currentAngle > Math.PI) neg90 -= 2 * Math.PI;
            while (neg90 - currentAngle < -Math.PI) neg90 += 2 * Math.PI;

            if (Math.abs(pos90 - currentAngle) <= Math.abs(neg90 - currentAngle)) {
                targetRad = pos90;
                if (nearNegative90) motorFlipped = true;
            } else {
                targetRad = neg90;
                if (nearPositive90) motorFlipped = true;
            }
        } else {
            if (targetRad > RIGHT_ANGLE) {
                targetRad -= Math.PI;
                motorFlipped = true;
            } else if (targetRad <= -RIGHT_ANGLE) {
                targetRad += Math.PI;
                motorFlipped = true;
            }

            while (targetRad - currentAngle > Math.PI) targetRad -= 2 * Math.PI;
            while (targetRad - currentAngle < -Math.PI) targetRad += 2 * Math.PI;
        }

        if (motorFlipped) speed = -speed;

        double nearestTarget = targetRad;

        this.targetAngle = angle;

        double error = nearestTarget - currentAngle;

        double cosineScale = Math.cos(error);
        double drivePower = speed * DriveConstants.DRIVE_FF * cosineScale;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double steerPower;
        if (Math.abs(error) < SteeringConstants.STEERING_DEADBAND_RADIANS) {
            steerPower = (error == 0) ? SteeringConstants.MIN_SERVO_POWER
                    : Math.signum(error) * SteeringConstants.MIN_SERVO_POWER;
        } else {
            steerPower = error * SteeringConstants.STEER_P + derivative * SteeringConstants.STEER_D;
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        drivePower = Math.max(-1.0, Math.min(1.0, drivePower));
        if (driveInverted) drivePower *= -1.0;
        driveMotor.setPower(drivePower);
    }

    public void hold() {
        driveMotor.setPower(0.0);
        double holdPower = steerInverted ? -SteeringConstants.MIN_SERVO_POWER
                : SteeringConstants.MIN_SERVO_POWER;
        steerServo.setPower(holdPower);
    }

    public void log(Telemetry telemetry) {
        double currentUnwrapped = getAngle();
        double currentWrapped = MathUtils.wrapDeg(Math.toDegrees(currentUnwrapped));
        double target = targetAngle;
        double targetWrapped = MathUtils.wrapDeg(Math.toDegrees(target));
        telemetry.addData(moduleName, "%.0f\u00b0 (%.0f\u00b0) \u2192 %.0f\u00b0",
                currentWrapped, Math.toDegrees(currentUnwrapped), targetWrapped);
    }
}
