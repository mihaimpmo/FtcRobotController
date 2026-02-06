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

        // Initialize lastNonZeroAngle to current wheel position (encoder handles tracking)
        lastNonZeroAngle = new Rotation2d(encoder.getUnwrappedWheelAngleRad());

        // Initialize desiredState to current position so it doesn't try to go to 0° on first call
        desiredState = new SwerveModuleState(0, lastNonZeroAngle);

        // Set servo to neutral on init (0 power = stop, encoder still works with Axon)
        steerServo.setPower(0);
    }

    /**
     * Returns the CONTINUOUS (unwrapped) wheel angle in radians.
     * This can be any value: -500°, 0°, 720°, etc.
     * Tracked continuously by AxonEncoder to avoid the 2:1 ambiguity problem.
     */
    public double getSteeringAngle() {
        return encoder.getUnwrappedWheelAngleRad();
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

        double rawTargetRad = MathUtils.normalizeAngleRadians(stateToSet.angle.getRadians());
        double speed = stateToSet.speedMetersPerSecond;
        double currentAngle = getSteeringAngle();  // UNWRAPPED (continuous)

        // Force target into canonical range (-90°, +90°] to keep all modules in sync
        final double RIGHT_ANGLE = Math.PI / 2;  // 90°
        final double NEAR_90_TOLERANCE = Math.toRadians(5.0);  // Within 5° of ±90°

        double targetRad = rawTargetRad;
        boolean motorFlipped = false;

        // Check if target is near ±90° (special case for strafe)
        boolean nearPositive90 = Math.abs(rawTargetRad - RIGHT_ANGLE) < NEAR_90_TOLERANCE;
        boolean nearNegative90 = Math.abs(rawTargetRad + RIGHT_ANGLE) < NEAR_90_TOLERANCE;

        if (nearPositive90 || nearNegative90) {
            // Special handling for strafe: pick nearest ±90° equivalent to current position
            // This prevents 180° rotation when switching strafe directions
            double pos90 = RIGHT_ANGLE;
            double neg90 = -RIGHT_ANGLE;

            // Find nearest 360° equivalent of each
            while (pos90 - currentAngle > Math.PI) pos90 -= 2 * Math.PI;
            while (pos90 - currentAngle < -Math.PI) pos90 += 2 * Math.PI;
            while (neg90 - currentAngle > Math.PI) neg90 -= 2 * Math.PI;
            while (neg90 - currentAngle < -Math.PI) neg90 += 2 * Math.PI;

            // Pick the closer one
            if (Math.abs(pos90 - currentAngle) <= Math.abs(neg90 - currentAngle)) {
                targetRad = pos90;
                // If original target was -90° but we're using +90°, flip motor
                if (nearNegative90) {
                    motorFlipped = true;
                }
            } else {
                targetRad = neg90;
                // If original target was +90° but we're using -90°, flip motor
                if (nearPositive90) {
                    motorFlipped = true;
                }
            }
        } else {
            // Normal case: clamp to (-90°, +90°]
            if (targetRad > RIGHT_ANGLE) {
                targetRad -= Math.PI;
                motorFlipped = true;
            } else if (targetRad <= -RIGHT_ANGLE) {
                targetRad += Math.PI;
                motorFlipped = true;
            }

            // Find nearest 360° equivalent
            while (targetRad - currentAngle > Math.PI) targetRad -= 2 * Math.PI;
            while (targetRad - currentAngle < -Math.PI) targetRad += 2 * Math.PI;
        }

        if (motorFlipped) {
            speed = -speed;
        }

        double nearestTarget = targetRad;

        this.desiredState = stateToSet;

        // Error to nearest equivalent target (will be ≤180°)
        double error = nearestTarget - currentAngle;

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
        double currentUnwrapped = getSteeringAngle();
        double currentWrapped = MathUtils.normalizeAngleDegrees(Math.toDegrees(currentUnwrapped));
        double target = desiredState.angle.getRadians();
        double targetWrapped = MathUtils.normalizeAngleDegrees(Math.toDegrees(target));
        telemetry.addData(moduleName, "%.0f° (%.0f°) → %.0f°",
                currentWrapped, Math.toDegrees(currentUnwrapped), targetWrapped);
    }
}
