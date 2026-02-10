package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Utils.EMA;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class AxonEncoder {
    private final AnalogInput encoder;
    private final double voltageOffset;
    private final double angleOffsetDegrees;
    private final EMA emaFilter;

    private double prevRawServoDeg = 0.0;
    private double unwrappedServoDeg = 0.0;
    private boolean firstUpdate = true;

    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.voltageOffset = voltageOffset;
        this.angleOffsetDegrees = voltageToWheelAngle(voltageOffset);
        this.emaFilter = new EMA(SteeringConstants.ENCODER_FILTER_ALPHA);

        initializeContinuousTracking();
    }

    private void initializeContinuousTracking() {
        double voltage = encoder.getVoltage();
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

    public double getRawAngleDegrees180() {
        double voltage = encoder.getVoltage();
        double rawAngle = ((voltage / 3.3) * 360.0) - 180.0;
        double offsetAngle = ((voltageOffset / 3.3) * 360.0) - 180.0;
        double calibrated = rawAngle - offsetAngle;
        return MathUtils.wrap180(calibrated);
    }

    public double getUnwrappedWheelAngleRad() {
        double voltage = encoder.getVoltage();
        double rawServoDeg = (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0 - angleOffsetDegrees;
        rawServoDeg = MathUtils.wrap180(rawServoDeg);

        if (firstUpdate) {
            prevRawServoDeg = rawServoDeg;
            unwrappedServoDeg = rawServoDeg;
            firstUpdate = false;
        }

        double delta = rawServoDeg - prevRawServoDeg;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        unwrappedServoDeg += delta;
        prevRawServoDeg = rawServoDeg;

        double wheelAngleDeg = unwrappedServoDeg / 2.0;
        return Math.toRadians(wheelAngleDeg);
    }

    public double getUnwrappedWheelAngleDeg() {
        return Math.toDegrees(getUnwrappedWheelAngleRad());
    }
}
