package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
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

    private SwerveModuleState desiredState = new SwerveModuleState();
    private Rotation2d lastNonZeroAngle; // initialized in constructor to current position
    private double lastError = 0.0;
    private long lastTime = System.nanoTime();
    private final boolean steerInvertedStored; // Store for hold() to use

    // Continuous angle tracking (fixes 0°/360° wrapping)
    private double prevRawAngleDeg = 0.0;
    private double unwrappedAngleDeg = 0.0;
    private boolean firstAngleUpdate = true;

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
        this.steerInvertedStored = steerInverted; // Store for hold()
        this.moduleName = moduleName;

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize angle tracking immediately to prevent random spin on first setDesiredState()
        // This reads the current encoder position and uses it as the starting point
        double initialAngle = encoder.getRawAngleDegrees180();
        prevRawAngleDeg = initialAngle;
        unwrappedAngleDeg = initialAngle;
        firstAngleUpdate = false;  // Mark as already initialized

        // Initialize lastNonZeroAngle to current position (clamped to ±90° wheel angle)
        double wheelAngleDeg = initialAngle / 2.0;  // 2:1 gear ratio
        double clampedDeg = Math.max(-90.0, Math.min(90.0, MathUtils.wrap180(wheelAngleDeg)));
        lastNonZeroAngle = new Rotation2d(Math.toRadians(clampedDeg));

        // Initialize desiredState to current position so it doesn't try to go to 0° on first call
        desiredState = new SwerveModuleState(0, lastNonZeroAngle);

        // Set servo to neutral on init (0 power = stop, encoder still works with Axon)
        steerServo.setPower(0);
    }

    public double getSteeringAngle() {
        double rawServoDeg = encoder.getRawAngleDegrees180();

        if (firstAngleUpdate) {
            prevRawAngleDeg = rawServoDeg;
            unwrappedAngleDeg = rawServoDeg;
            firstAngleUpdate = false;
        }

        // Unwrap delta to avoid jumps at boundary
        double delta = rawServoDeg - prevRawAngleDeg;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        unwrappedAngleDeg += delta;
        prevRawAngleDeg = rawServoDeg;

        // 2:1 gear ratio: wheel = servo / 2
        double wheelAngleDeg = unwrappedAngleDeg / 2.0;
        double wrappedDeg = MathUtils.wrap180(wheelAngleDeg);

        // Clamp measured angle to [-90°, +90°] - same range as target
        // With 2:1 ratio, this is the valid operating range
        if (wrappedDeg > 90.0) wrappedDeg = 90.0;
        if (wrappedDeg < -90.0) wrappedDeg = -90.0;

        return Math.toRadians(wrappedDeg);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Default to optimize=false - FTCLib optimize fails due to 2:1 encoder overlap
        // (90° and 270° servo positions output same voltage)
        setDesiredState(desiredState, false);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        // When stopped, hold last direction instead of snapping to 0°
        boolean isStationaryDefault =
                Math.abs(desiredState.speedMetersPerSecond) < 0.001 &&
                Math.abs(desiredState.angle.getRadians()) < 0.001;

        SwerveModuleState stateToUse = isStationaryDefault
                ? new SwerveModuleState(0, lastNonZeroAngle)
                : desiredState;

        SwerveModuleState stateToSet = optimize
                ? SwerveModuleState.optimize(stateToUse, new Rotation2d(getSteeringAngle()))
                : stateToUse;

        // Cache optimized angle for hold behavior
        if (!isStationaryDefault && Math.abs(desiredState.speedMetersPerSecond) > 0.001) {
            lastNonZeroAngle = stateToSet.angle;
        }

        double targetRad = MathUtils.normalizeAngleRadians(stateToSet.angle.getRadians());
        double speed = stateToSet.speedMetersPerSecond;



        if (targetRad > Math.PI / 2) {
            targetRad -= Math.PI;
            speed = -speed;
        } else if (targetRad < -Math.PI / 2) {
            targetRad += Math.PI;
            speed = -speed;
        }

        this.desiredState = stateToSet;

        double currentAngle = getSteeringAngle();

        double error = MathUtils.shortestAngularDistance(currentAngle, targetRad);

        // Cosine compensation: reduce drive power when wheel isn't aligned
        // Prevents wheel from "fighting" - driving hard while still turning
        double cosineScale = Math.cos(error);  // 1.0 at 0°, 0 at 90°
        double drivePower = speed * DriveConstants.DRIVE_FF * cosineScale;

        // Calculate derivative for PD control
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double steerPower;
        if (Math.abs(error) < SteeringConstants.STEERING_DEADBAND_RADIANS) {
            // FIX: Use directional power even in deadband to prevent drift
            // If error is exactly 0, default to positive (arbitrary but consistent)
            steerPower = (error == 0) ? SteeringConstants.MIN_SERVO_POWER
                                      : Math.signum(error) * SteeringConstants.MIN_SERVO_POWER;
        } else {
            // PD control
            steerPower = error * SteeringConstants.STEER_P + derivative * SteeringConstants.STEER_D;
          //  steerPower += Math.signum(steerPower) * SteeringConstants.STATIC_FRICTION_COMPENSATION;
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));

            /*if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
                steerPower = Math.signum(error) * SteeringConstants.MIN_SERVO_POWER;
            }*/
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        drivePower = Math.max(-1.0, Math.min(1.0, drivePower));
        if (driveInverted) drivePower *= -1.0;
        driveMotor.setPower(drivePower);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(0.0, new Rotation2d(getSteeringAngle()));
    }

    public void hold() {
        driveMotor.setPower(0.0);
        // FIX: Apply steerInverted consistently with setDesiredState()
        double holdPower = steerInvertedStored ? -SteeringConstants.MIN_SERVO_POWER
                                               : SteeringConstants.MIN_SERVO_POWER;
        steerServo.setPower(holdPower);
    }

    public void addTelemetry(Telemetry telemetry) {
        double current = getSteeringAngle();
        double target = desiredState.angle.getRadians();
        double err = Math.toDegrees(MathUtils.shortestAngularDistance(current, target));
        telemetry.addData(moduleName, "%.1f° → %.1f° (err: %.1f°)",
                Math.toDegrees(current), Math.toDegrees(target), err);
    }
}
