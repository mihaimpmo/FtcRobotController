package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;

public class SwerveDrive {
    public final SwerveModule fl, fr, bl, br;
    private final SwerveDriveKinematics kinematics;

    public SwerveDrive(HardwareMap hardwareMap) {
        fl = createModule(hardwareMap, "fl",
                SteeringConstants.FL_ENCODER_NAME, SteeringConstants.FL_ENCODER_SHARED,
                SteeringConstants.FL_SWITCH_NAME, SteeringConstants.FL_TICK_OFFSET, 
                false, true, false); 
        fr = createModule(hardwareMap, "fr",
                SteeringConstants.FR_ENCODER_NAME, SteeringConstants.FR_ENCODER_SHARED,
                SteeringConstants.FR_SWITCH_NAME, SteeringConstants.FR_TICK_OFFSET,
                true, true, true);
        bl = createModule(hardwareMap, "bl",
                SteeringConstants.BL_ENCODER_NAME, SteeringConstants.BL_ENCODER_SHARED,
                SteeringConstants.BL_SWITCH_NAME, SteeringConstants.BL_TICK_OFFSET, 
                false, false, false);
        br = createModule(hardwareMap, "br",
                SteeringConstants.BR_ENCODER_NAME, SteeringConstants.BR_ENCODER_SHARED,
                SteeringConstants.BR_SWITCH_NAME, SteeringConstants.BR_TICK_OFFSET, 
                true, true, true);

        // FTCLib kinematics: +x = forward, +y = left
        double halfWB = DriveConstants.WHEELBASE_METERS / 2.0;
        double halfTW = DriveConstants.TRACK_WIDTH_METERS / 2.0;
        kinematics = new SwerveDriveKinematics(
                new Translation2d(halfWB, halfTW),    // FL
                new Translation2d(halfWB, -halfTW),   // FR
                new Translation2d(-halfWB, halfTW),   // BL
                new Translation2d(-halfWB, -halfTW)   // BR
        );
    }

    private SwerveModule createModule(
            HardwareMap hardwareMap,
            String name,
            String encoderName,
            boolean encoderShared,
            String switchName,
            int tickOffset,
            boolean driveInverted,
            boolean encoderInverted,
            boolean steerInverted
    ) {
        RevThroughBoreEncoder encoder = new RevThroughBoreEncoder(
                hardwareMap.get(DcMotorEx.class, encoderName), encoderShared
        );
        encoder.setInverted(encoderInverted);

        return new SwerveModule(
                hardwareMap.get(DcMotorEx.class, name),
                hardwareMap.get(CRServo.class, name + "_servo"),
                encoder,
                hardwareMap.get(DigitalChannel.class, switchName),
                driveInverted,
                steerInverted,
                name.toUpperCase(),
                tickOffset
        );
    }

    /**
     * Home all 4 modules simultaneously with dual-stage homing.
     */
    public boolean homeAllModules(LinearOpMode opMode) {
        SwerveModule[] modules = {fl, fr, bl, br};
        boolean[] done = new boolean[4];
        long startTime = System.currentTimeMillis();

        for (int i = 0; i < 4; i++) {
            if (modules[i].isLimitSwitchPressed()) {
                modules[i].finishHoming();
                done[i] = true;
            } else {
                modules[i].startHoming();
            }
        }

        while (opMode.opModeIsActive()) {
            boolean allDone = true;
            for (int i = 0; i < 4; i++) {
                if (done[i]) continue;
                if (modules[i].updateHoming()) {
                    done[i] = true;
                } else {
                    allDone = false;
                }
            }
            if (allDone) break;

            if (System.currentTimeMillis() - startTime > SteeringConstants.HOMING_TIMEOUT_MS) {
                for (int i = 0; i < 4; i++) {
                    if (!done[i]) modules[i].stop();
                }
                return false;
            }
        }

        return true;
    }


    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void drive(double fwd, double str, double rot) {
        SwerveModule[] modules = {fl, fr, bl, br};

        if (Math.abs(fwd) < 0.01 && Math.abs(str) < 0.01 && Math.abs(rot) < 0.01) {
            for (SwerveModule m : modules) m.setTarget(m.getTargetAngle(), 0);
            return;
        }

        // FTCLib uses vx (forward), vy (left), omega (CCW positive)
        ChassisSpeeds speeds = new ChassisSpeeds(fwd, str, rot);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);

        for (int i = 0; i < 4; i++) {
            modules[i].setTarget(states[i].angle.getRadians(), states[i].speedMetersPerSecond);
        }
    }

    /**
     * Run PID and write hardware for all modules. Call every loop iteration.
     */
    public void update() {
        fl.update();
        fr.update();
        bl.update();
        br.update();
    }

    public void hold() {
        fl.hold();
        fr.hold();
        bl.hold();
        br.hold();
    }

    public void log(Telemetry telemetry) {
        fl.log(telemetry);
        fr.log(telemetry);
        bl.log(telemetry);
        br.log(telemetry);
    }

    public void logDetailed(Telemetry telemetry) {
        fl.logDetailed(telemetry);
        fr.logDetailed(telemetry);
        bl.logDetailed(telemetry);
        br.logDetailed(telemetry);
    }
}
