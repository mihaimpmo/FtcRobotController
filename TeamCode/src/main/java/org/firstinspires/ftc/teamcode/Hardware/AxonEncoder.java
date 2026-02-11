package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class AxonEncoder {
    private final AnalogInput encoder;
    private final double angleOffsetDegrees;

    private double previousWrappedAngleDeg;
    private double continuousAngleDeg;

    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.angleOffsetDegrees = (voltageOffset / SteeringConstants.MAX_VOLTAGE) * 360.0;

        double wrappedAngleDeg = readWrappedAngleDeg();
        this.previousWrappedAngleDeg = wrappedAngleDeg;
        this.continuousAngleDeg = wrappedAngleDeg;
    }

    private double readWrappedAngleDeg() {
        double voltage = encoder.getVoltage();
        double angleDeg = (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0 - angleOffsetDegrees;
        return MathUtils.wrap180(angleDeg);
    }

    public double getWheelAngleRad() {
        double wrappedAngleDeg = readWrappedAngleDeg();

        double delta = wrappedAngleDeg - previousWrappedAngleDeg;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        continuousAngleDeg += delta;
        previousWrappedAngleDeg = wrappedAngleDeg;

        double wheelAngleDeg = continuousAngleDeg / SteeringConstants.SERVO_TO_WHEEL_RATIO;
        return Math.toRadians(wheelAngleDeg);
    }
}
