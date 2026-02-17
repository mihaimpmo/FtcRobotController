package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class SwerveModule {
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final RevThroughBoreEncoder encoder;
    private final DigitalChannel limitSwitch;
    private final boolean driveInverted;
    private final boolean steerInverted;
    private final String moduleName;
    private final int calibratedTickOffset;

    private enum HomingStage { IDLE, FAST_APPROACH, BACKOFF, SLOW_APPROACH, DONE }

    private double targetAngle;
    private double heldAngle;
    private double lastAngle = 0.0;
    private long lastTime = System.nanoTime();
    private boolean homed = false;

    private HomingStage homingStage = HomingStage.IDLE;
    private long backoffStartTime;

    public SwerveModule(
            DcMotorEx driveMotor,
            CRServo steerServo,
            RevThroughBoreEncoder encoder,
            DigitalChannel limitSwitch,
            boolean driveInverted,
            boolean steerInverted,
            String moduleName,
            int calibratedTickOffset
    ) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.encoder = encoder;
        this.limitSwitch = limitSwitch;
        this.driveInverted = driveInverted;
        this.steerInverted = steerInverted;
        this.moduleName = moduleName;
        this.calibratedTickOffset = calibratedTickOffset;

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (limitSwitch != null) {
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        // Not homed yet — default to 0
        heldAngle = 0;
        targetAngle = 0;
        steerServo.setPower(0);
    }

    /**
     * Check if the limit switch is currently triggered.
     */
    public boolean isLimitSwitchPressed() {
        if (limitSwitch == null) return false;
        return limitSwitch.getState() == SteeringConstants.LIMIT_SWITCH_ACTIVE_STATE;
    }

    /**
     * Start dual-stage homing: fast approach → back off → slow approach.
     * Called by SwerveDrive's simultaneous homing loop.
     * Follow up with updateHoming() each loop iteration.
     */
    public void startHoming() {
        homingStage = HomingStage.FAST_APPROACH;
        double power = SteeringConstants.HOMING_FAST_POWER;
        if (steerInverted) power = -power;
        steerServo.setPower(power);
    }

    /**
     * Advance the homing state machine. Call each loop iteration.
     * Returns true when homing is complete.
     */
    public boolean updateHoming() {
        switch (homingStage) {
            case FAST_APPROACH:
                if (isLimitSwitchPressed()) {
                    // Hit the switch — back off
                    double backoff = SteeringConstants.HOMING_BACKOFF_POWER;
                    if (steerInverted) backoff = -backoff;
                    steerServo.setPower(backoff);
                    backoffStartTime = System.currentTimeMillis();
                    homingStage = HomingStage.BACKOFF;
                }
                break;

            case BACKOFF:
                if (System.currentTimeMillis() - backoffStartTime >= SteeringConstants.HOMING_BACKOFF_MS) {
                    // Done backing off — slow approach
                    double slow = SteeringConstants.HOMING_SLOW_POWER;
                    if (steerInverted) slow = -slow;
                    steerServo.setPower(slow);
                    homingStage = HomingStage.SLOW_APPROACH;
                }
                break;

            case SLOW_APPROACH:
                if (isLimitSwitchPressed()) {
                    finishHoming();
                    homingStage = HomingStage.DONE;
                }
                break;

            case DONE:
                return true;

            default:
                break;
        }
        return homingStage == HomingStage.DONE;
    }

    /**
     * Finish homing: stop servo, set encoder zero, update held angle.
     */
    public void finishHoming() {
        steerServo.setPower(0);
        encoder.setHomePosition(calibratedTickOffset);
        homed = true;
        heldAngle = encoder.getWheelAngleRad();
        targetAngle = heldAngle;
    }

    /**
     * Home this module standalone (for test opmodes).
     * Dual-stage: fast approach, back off, slow approach.
     * Returns true if homing succeeded, false if timed out.
     */
    public boolean homeModule(LinearOpMode opMode) {
        if (limitSwitch == null) {
            homed = true;
            heldAngle = encoder.getWheelAngleRad();
            targetAngle = heldAngle;
            return true;
        }

        long startTime = System.currentTimeMillis();
        startHoming();

        while (opMode.opModeIsActive()) {
            if (updateHoming()) return true;
            if (System.currentTimeMillis() - startTime > SteeringConstants.HOMING_TIMEOUT_MS) {
                steerServo.setPower(0);
                return false;
            }
        }

        steerServo.setPower(0);
        return false;
    }

    public boolean isHomed() {
        return homed;
    }

    public double getAngle() {
        return encoder.getWheelAngleRad();
    }

    public int getRawTicks() {
        return encoder.getRawTicks();
    }

    public int getPositionTicks() {
        return encoder.getPositionTicks();
    }

    public void set(double angle, double speed) {
        if (Math.abs(speed) < 0.001) {
            angle = heldAngle;
        } else {
            heldAngle = angle;
        }

        double currentAngle = getAngle();
        double target = angle;

        // Unwrap target to be closest to current angle (continuous rotation)
        while (target - currentAngle > Math.PI) target -= 2 * Math.PI;
        while (target - currentAngle < -Math.PI) target += 2 * Math.PI;

        // Motor flip optimization: if >90° away, reverse motor and flip 180°
        double error = target - currentAngle;
        boolean motorFlipped = false;
        if (Math.abs(error) > Math.PI / 2) {
            target -= Math.signum(error) * Math.PI;
            motorFlipped = true;
            error = target - currentAngle;
        }

        if (motorFlipped) speed = -speed;

        this.targetAngle = angle;

        // Cosine scaling: reduce drive power when module is misaligned
        double cosineScale = Math.cos(error);
        double drivePower = speed * DriveConstants.DRIVE_FF * cosineScale;

        // PD controller — derivative on measurement to avoid derivative kick
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (dt > 0) ? -(currentAngle - lastAngle) / dt : 0;
        lastAngle = currentAngle;

        double steerPower = error * SteeringConstants.STEER_P + derivative * SteeringConstants.STEER_D;
        // Apply minimum power floor to overcome static friction
        if (Math.abs(error) > SteeringConstants.STEERING_DEADBAND_RADIANS) {
            if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
                steerPower = Math.signum(steerPower) * SteeringConstants.MIN_SERVO_POWER;
            }
        }
        steerPower = Math.max(-1.0, Math.min(1.0, steerPower));

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        drivePower = Math.max(-1.0, Math.min(1.0, drivePower));
        if (driveInverted) drivePower *= -1.0;
        driveMotor.setPower(drivePower);
    }

    /**
     * Steer to a target angle without driving. Uses the PD controller.
     */
    public void steerToAngle(double angleRad) {
        driveMotor.setPower(0);
        double currentAngle = getAngle();
        double error = angleRad - currentAngle;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (dt > 0) ? -(currentAngle - lastAngle) / dt : 0;
        lastAngle = currentAngle;

        double steerPower = error * SteeringConstants.STEER_P + derivative * SteeringConstants.STEER_D;
        if (Math.abs(error) > SteeringConstants.STEERING_DEADBAND_RADIANS) {
            if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
                steerPower = Math.signum(steerPower) * SteeringConstants.MIN_SERVO_POWER;
            }
        }
        steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        heldAngle = angleRad;
        targetAngle = angleRad;
    }

    /**
     * Check if the module is within tolerance of a target angle.
     */
    public boolean isAtAngle(double angleRad, double toleranceRad) {
        double error = angleRad - getAngle();
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return Math.abs(error) < toleranceRad;
    }

    /**
     * Reset encoder so current position becomes 0.
     */
    public void resetEncoder() {
        encoder.setHomePosition(0);
        heldAngle = 0;
        targetAngle = 0;
        lastAngle = 0;
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

    public void logDetailed(Telemetry telemetry) {
        double currentRad = getAngle();
        double currentDeg = MathUtils.wrapDeg(Math.toDegrees(currentRad));
        double targetDeg = MathUtils.wrapDeg(Math.toDegrees(targetAngle));
        double errorDeg = Math.toDegrees(targetAngle - currentRad);
        telemetry.addLine("--- " + moduleName + " ---");
        telemetry.addData("  Angle", "%.1f\u00b0 \u2192 %.1f\u00b0", currentDeg, targetDeg);
        telemetry.addData("  Error", "%.2f\u00b0", errorDeg);
        telemetry.addData("  Raw Ticks", "%d", getRawTicks());
        telemetry.addData("  Position Ticks", "%d", getPositionTicks());
        telemetry.addData("  Homed", "%s", homed);
    }
}
