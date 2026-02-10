package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;

public class SwerveDrive {
    public final SwerveModule fl, fr, bl, br;
    private final SwerveDriveKinematics kinematics;

    public SwerveDrive(HardwareMap hardwareMap) {
        kinematics = new SwerveDriveKinematics(
                DriveConstants.FL_POSITION,
                DriveConstants.FR_POSITION,
                DriveConstants.BL_POSITION,
                DriveConstants.BR_POSITION
        );

        fl = createModule(hardwareMap, "fl", SteeringConstants.FL_VOLTAGE_OFFSET, true, true);
        fr = createModule(hardwareMap, "fr", SteeringConstants.FR_VOLTAGE_OFFSET, true, true);
        bl = createModule(hardwareMap, "bl", SteeringConstants.BL_VOLTAGE_OFFSET, true, true);
        br = createModule(hardwareMap, "br", SteeringConstants.BR_VOLTAGE_OFFSET, true, true);
    }

    private SwerveModule createModule(
            HardwareMap hardwareMap,
            String name,
            double voltageOffset,
            boolean driveInverted,
            boolean steerInverted
    ) {
        return new SwerveModule(
                hardwareMap.get(DcMotorEx.class, name),
                hardwareMap.get(CRServo.class, name + "_servo"),
                new AxonEncoder(
                        hardwareMap.get(AnalogInput.class, name + "_enc"),
                        voltageOffset
                ),
                driveInverted,
                steerInverted,
                name.toUpperCase()
        );
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        drive(xSpeed, ySpeed, rotSpeed, false);
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean optimize) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        fl.setDesiredState(moduleStates[0], optimize);
        fr.setDesiredState(moduleStates[1], optimize);
        bl.setDesiredState(moduleStates[2], optimize);
        br.setDesiredState(moduleStates[3], optimize);
    }

    private SwerveModuleState scaleSpeed(SwerveModuleState state, double scale) {
        return new SwerveModuleState(state.speedMetersPerSecond * scale, state.angle);
    }

    public void hold() {
        fl.hold();
        fr.hold();
        bl.hold();
        br.hold();
    }

    public void resetModulesToZero() {
        SwerveModuleState zeroState = new SwerveModuleState(0.0, new Rotation2d(0));
        fl.setDesiredState(zeroState, false);
        fr.setDesiredState(zeroState, false);
        bl.setDesiredState(zeroState, false);
        br.setDesiredState(zeroState, false);
    }

    public void addTelemetry(Telemetry telemetry) {
        fl.addTelemetry(telemetry);
        fr.addTelemetry(telemetry);
        bl.addTelemetry(telemetry);
        br.addTelemetry(telemetry);
    }
}