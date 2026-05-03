package org.firstinspires.ftc.teamcode.kinematics;

/**
 * Wheel command: speed (m/s) and azimuth. Matches WPILib {@code SwerveModuleState} behavior.
 */
public final class SwerveModuleState {
    public double speedMetersPerSecond;
    public Rotation2d angle;

    public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
    }

    /**
     * Chooses equivalent angle within ±90° of {@code currentAngle}, flipping drive direction if needed.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getRadians()) > Math.PI / 2.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(new Rotation2d(Math.PI))
            );
        }
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
}
