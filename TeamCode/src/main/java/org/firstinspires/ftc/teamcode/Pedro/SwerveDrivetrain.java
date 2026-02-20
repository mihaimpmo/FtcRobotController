package org.firstinspires.ftc.teamcode.Pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@Configurable
public class SwerveDrivetrain extends Drivetrain {

    private final SwerveDrive swerveDrive;
    private final SwerveDriveKinematics kinematics;
    private final VoltageSensor voltageSensor;

    public static double X_VELOCITY = 50.0;
    public static double Y_VELOCITY = 50.0;

    private double currentXVelocity = X_VELOCITY;
    private double currentYVelocity = Y_VELOCITY;

    // Debug values from last calculateDrive call
    public double dbgPathX, dbgPathY, dbgCorrX, dbgCorrY, dbgHeadX, dbgHeadY;
    public double dbgFieldTx, dbgFieldTy, dbgRobotX, dbgRobotY, dbgOmega;
    public double dbgVx, dbgVy;

    public SwerveDrivetrain(HardwareMap hardwareMap) {
        this.swerveDrive = new SwerveDrive(hardwareMap);
        this.kinematics = swerveDrive.getKinematics();
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Core Pedro bridge: convert 3 field-relative vectors into swerve module commands.
     *
     * @param correctivePower translational correction vector (field-relative)
     * @param headingPower    heading correction vector (field-relative, direction encodes rotation)
     * @param pathingPower    feedforward pathing vector (field-relative)
     * @param robotHeading    current robot heading in radians
     * @return double[8]: [fl_angle, fl_speed, fr_angle, fr_speed, bl_angle, bl_speed, br_angle, br_speed]
     */
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double scale = getMaxPowerScaling();

        // Clamp all vectors to max power scaling
        Vector corrective = clampVector(correctivePower, scale);
        Vector heading = clampVector(headingPower, scale);
        Vector pathing = clampVector(pathingPower, scale);

        // Extract rotation from heading vector via cross product with robot's forward unit vector
        // Robot forward in field frame = (cos(robotHeading), sin(robotHeading))
        double forwardX = Math.cos(robotHeading);
        double forwardY = Math.sin(robotHeading);
        double headX = heading.getXComponent();
        double headY = heading.getYComponent();
        // Cross product: forward x heading = forwardX*headY - forwardY*headX
        double omega = forwardX * headY - forwardY * headX;

        double fieldTx = corrective.getXComponent() + pathing.getXComponent();
        double fieldTy = corrective.getYComponent() + pathing.getYComponent();

        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double robotX = fieldTx * cos - fieldTy * sin;
        double robotY = fieldTx * sin + fieldTy * cos;

        dbgPathX = pathing.getXComponent(); dbgPathY = pathing.getYComponent();
        dbgCorrX = corrective.getXComponent(); dbgCorrY = corrective.getYComponent();
        dbgHeadX = heading.getXComponent(); dbgHeadY = heading.getYComponent();
        dbgFieldTx = fieldTx; dbgFieldTy = fieldTy;
        dbgRobotX = robotX; dbgRobotY = robotY;
        dbgOmega = omega;
        dbgVx = -robotX; dbgVy = -robotY;

        ChassisSpeeds speeds = new ChassisSpeeds(-robotX, -robotY, omega);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);

        // Encode as [fl_angle, fl_speed, fr_angle, fr_speed, bl_angle, bl_speed, br_angle, br_speed]
        // Kinematics order matches constructor: FL=0, FR=1, BL=2, BR=3
        return new double[]{
                states[0].angle.getRadians(), states[0].speedMetersPerSecond,
                states[1].angle.getRadians(), states[1].speedMetersPerSecond,
                states[2].angle.getRadians(), states[2].speedMetersPerSecond,
                states[3].angle.getRadians(), states[3].speedMetersPerSecond
        };
    }

    /**
     * Apply the 8-value drive command to the swerve modules.
     */
    @Override
    public void runDrive(double[] drivePowers) {
        if (drivePowers.length < 8) return;

        swerveDrive.fl.setTarget(drivePowers[0], drivePowers[1]);
        swerveDrive.fr.setTarget(drivePowers[2], drivePowers[3]);
        swerveDrive.bl.setTarget(drivePowers[4], drivePowers[5]);
        swerveDrive.br.setTarget(drivePowers[6], drivePowers[7]);

        swerveDrive.update();
    }

    @Override
    public void updateConstants() {
        currentXVelocity = X_VELOCITY;
        currentYVelocity = Y_VELOCITY;
    }

    @Override
    public void breakFollowing() {
        swerveDrive.hold();
    }

    @Override
    public void startTeleopDrive() {
        // No special setup needed for teleop
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        // No special setup needed for teleop
    }

    @Override
    public double xVelocity() {
        return currentXVelocity;
    }

    @Override
    public double yVelocity() {
        return currentYVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) {
        currentXVelocity = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        currentYVelocity = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public void logDebug(Telemetry telemetry) {
        telemetry.addData("Pedro Path", "X=%.3f Y=%.3f", dbgPathX, dbgPathY);
        telemetry.addData("Pedro Corr", "X=%.3f Y=%.3f", dbgCorrX, dbgCorrY);
        telemetry.addData("Pedro Head", "X=%.3f Y=%.3f", dbgHeadX, dbgHeadY);
        telemetry.addData("Field T", "X=%.3f Y=%.3f", dbgFieldTx, dbgFieldTy);
        telemetry.addData("Robot T", "X=%.3f Y=%.3f", dbgRobotX, dbgRobotY);
        telemetry.addData("ChassisSpeeds", "vx=%.3f vy=%.3f omega=%.3f", dbgVx, dbgVy, dbgOmega);
    }

    @Override
    public String debugString() {
        return String.format("SwerveDrivetrain | vx=%.1f vy=%.1f voltage=%.1f",
                currentXVelocity, currentYVelocity, getVoltage());
    }

    /**
     * Clamp a vector's magnitude to the given max, preserving direction.
     */
    private Vector clampVector(Vector v, double max) {
        if (v.getMagnitude() > max) {
            Vector clamped = v.copy();
            clamped.setMagnitude(max);
            return clamped;
        }
        return v;
    }
}
