package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Utils.EMA;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

/**
 * Axon encoder for swerve steering. Outputs 0-3.3V for 360° SERVO rotation.
 * With 2:1 gear ratio: 360° servo = 180° wheel.
 *
 * Encoder reads SERVO, so we divide by 2 to get wheel angle.
 * Supports continuous (unwrapped) angle tracking.
 */
public class AxonEncoder {
    private final AnalogInput encoder;
    private final double voltageOffset;
    private final double angleOffsetDegrees;
    private final EMA emaFilter;

    // Continuous angle tracking (encoder reads servo, divide by 2 for wheel)
    private double prevRawServoDeg = 0.0;
    private double unwrappedServoDeg = 0.0;
    private boolean firstUpdate = true;

    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.voltageOffset = voltageOffset;
        this.angleOffsetDegrees = voltageToWheelAngle(voltageOffset);
        this.emaFilter = new EMA(SteeringConstants.ENCODER_FILTER_ALPHA);

        // Initialize continuous tracking immediately
        initializeContinuousTracking();
    }

    private void initializeContinuousTracking() {
        double voltage = encoder.getVoltage();
        // Encoder reads servo (0-3.3V = 0-360° servo)
        double rawServoDeg = (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0 - angleOffsetDegrees;
        rawServoDeg = MathUtils.wrap180(rawServoDeg);
        prevRawServoDeg = rawServoDeg;
        unwrappedServoDeg = rawServoDeg;
        firstUpdate = false;
    }

    private double voltageToWheelAngle(double voltage) {
        voltage = Math.max(0.0, Math.min(voltage, SteeringConstants.MAX_VOLTAGE));
        return (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0;
    }

    private double normalizeAngle0To360(double angleDegrees) {
        angleDegrees = angleDegrees % 360.0;
        if (angleDegrees < 0) angleDegrees += 360.0;
        return angleDegrees;
    }

    public double getAngleDegrees() {
        double currentVoltage = encoder.getVoltage();
        double rawWheelAngle = voltageToWheelAngle(currentVoltage);
        double calibratedAngle = rawWheelAngle - angleOffsetDegrees;
        double normalizedAngle = normalizeAngle0To360(calibratedAngle);

        if (SteeringConstants.ENCODER_FILTER_ALPHA > 0) {
            return emaFilter.update(normalizedAngle);
        }
        return normalizedAngle;
    }

    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    public double getRawVoltage() {
        return encoder.getVoltage();
    }

    public double getRawServoAngle() {
        double voltage = Math.max(0.0, Math.min(encoder.getVoltage(), SteeringConstants.MAX_VOLTAGE));
        return (voltage / SteeringConstants.MAX_VOLTAGE) * 720.0;
    }

    /** Returns calibrated wheel angle in [-180, 180] for continuous tracking */
    public double getRawAngleDegrees180() {
        double voltage = encoder.getVoltage();
        double rawAngle = ((voltage / 3.3) * 360.0) - 180.0;
        double offsetAngle = ((voltageOffset / 3.3) * 360.0) - 180.0;
        double calibrated = rawAngle - offsetAngle;
        return MathUtils.wrap180(calibrated);
    }

    /**
     * Returns the CONTINUOUS (unwrapped) wheel angle in RADIANS.
     * Encoder reads servo, divided by 2 for wheel angle (2:1 gear ratio).
     *
     * This value can be ANY number: -500°, 0°, 720°, etc.
     * It tracks the true physical position continuously.
     */
    public double getUnwrappedWheelAngleRad() {
        double voltage = encoder.getVoltage();
        // Encoder reads servo (0-3.3V = 0-360° servo)
        double rawServoDeg = (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0 - angleOffsetDegrees;
        rawServoDeg = MathUtils.wrap180(rawServoDeg);

        if (firstUpdate) {
            prevRawServoDeg = rawServoDeg;
            unwrappedServoDeg = rawServoDeg;
            firstUpdate = false;
        }

        // Unwrap delta to avoid jumps at ±180° boundary
        double delta = rawServoDeg - prevRawServoDeg;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        unwrappedServoDeg += delta;
        prevRawServoDeg = rawServoDeg;

        // 2:1 gear ratio: wheel = servo / 2
        double wheelAngleDeg = unwrappedServoDeg / 2.0;
        return Math.toRadians(wheelAngleDeg);
    }

    /**
     * Returns the CONTINUOUS (unwrapped) wheel angle in DEGREES.
     */
    public double getUnwrappedWheelAngleDeg() {
        return Math.toDegrees(getUnwrappedWheelAngleRad());
    }
}
