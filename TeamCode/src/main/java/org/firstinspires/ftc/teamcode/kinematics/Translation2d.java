package org.firstinspires.ftc.teamcode.kinematics;

/**
 * Module position relative to the robot center: +x forward, +y left (WPILib / FTCLib).
 */
public final class Translation2d {
    private final double x;
    private final double y;

    public Translation2d(double xMeters, double yMeters) {
        this.x = xMeters;
        this.y = yMeters;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
