package org.firstinspires.ftc.teamcode.kinematics;

import org.firstinspires.ftc.teamcode.Utils.MathUtils;

/**
 * Robot heading / wheel angle in radians (same conventions as WPILib FTCLib bindings).
 */
public final class Rotation2d {
    private final double radians;

    public Rotation2d(double radians) {
        this.radians = radians;
    }

    public double getRadians() {
        return radians;
    }

    public Rotation2d plus(Rotation2d other) {
        return new Rotation2d(MathUtils.wrapRad(radians + other.radians));
    }

    /**
     * Difference {@code this - other}, wrapped to [-π, π].
     */
    public Rotation2d minus(Rotation2d other) {
        return new Rotation2d(MathUtils.wrapRad(radians - other.radians));
    }

    public Rotation2d rotateBy(Rotation2d delta) {
        return plus(delta);
    }
}
