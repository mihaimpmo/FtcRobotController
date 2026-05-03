package org.firstinspires.ftc.teamcode.kinematics;

/**
 * Four-module swerve inverse kinematics (+x forward, +y left, +omega CCW).
 */
public final class SwerveDriveKinematics {
    private final Translation2d[] moduleTranslations;

    public SwerveDriveKinematics(Translation2d fl, Translation2d fr, Translation2d bl, Translation2d br) {
        moduleTranslations = new Translation2d[]{fl, fr, bl, br};
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        double vx = chassisSpeeds.vxMetersPerSecond;
        double vy = chassisSpeeds.vyMetersPerSecond;
        double omega = chassisSpeeds.omegaRadiansPerSecond;

        for (int i = 0; i < 4; i++) {
            Translation2d p = moduleTranslations[i];
            double mx = vx - omega * p.getY();
            double my = vy + omega * p.getX();
            double speed = Math.hypot(mx, my);
            double angle = Math.atan2(my, mx);
            states[i] = new SwerveModuleState(speed, new Rotation2d(angle));
        }
        return states;
    }

    /**
     * Scales all wheel speeds so the largest magnitude does not exceed {@code attainableMaxSpeed}.
     */
    public static void normalizeWheelSpeeds(SwerveModuleState[] moduleStates, double attainableMaxSpeed) {
        double max = 0.0;
        for (SwerveModuleState s : moduleStates) {
            max = Math.max(max, Math.abs(s.speedMetersPerSecond));
        }
        if (max <= 1e-9 || max <= attainableMaxSpeed) {
            return;
        }
        double scale = attainableMaxSpeed / max;
        for (SwerveModuleState s : moduleStates) {
            s.speedMetersPerSecond *= scale;
        }
    }
}
