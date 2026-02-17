package org.firstinspires.ftc.teamcode.Subsystems;

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

    public SwerveDrive(HardwareMap hardwareMap) {
        fl = createModule(hardwareMap, "fl",
                SteeringConstants.FL_ENCODER_NAME, SteeringConstants.FL_ENCODER_SHARED,
                SteeringConstants.FL_SWITCH_NAME, SteeringConstants.FL_TICK_OFFSET, true, true);
        fr = createModule(hardwareMap, "fr",
                SteeringConstants.FR_ENCODER_NAME, SteeringConstants.FR_ENCODER_SHARED,
                SteeringConstants.FR_SWITCH_NAME, SteeringConstants.FR_TICK_OFFSET, true, true);
        bl = createModule(hardwareMap, "bl",
                SteeringConstants.BL_ENCODER_NAME, SteeringConstants.BL_ENCODER_SHARED,
                SteeringConstants.BL_SWITCH_NAME, SteeringConstants.BL_TICK_OFFSET, true, true);
        br = createModule(hardwareMap, "br",
                SteeringConstants.BR_ENCODER_NAME, SteeringConstants.BR_ENCODER_SHARED,
                SteeringConstants.BR_SWITCH_NAME, SteeringConstants.BR_TICK_OFFSET, true, true);
    }

    private SwerveModule createModule(
            HardwareMap hardwareMap,
            String name,
            String encoderName,
            boolean encoderShared,
            String switchName,
            int tickOffset,
            boolean driveInverted,
            boolean steerInverted
    ) {
        return new SwerveModule(
                hardwareMap.get(DcMotorEx.class, name),
                hardwareMap.get(CRServo.class, name + "_servo"),
                new RevThroughBoreEncoder(
                        hardwareMap.get(DcMotorEx.class, encoderName), encoderShared
                ),
                hardwareMap.get(DigitalChannel.class, switchName),
                driveInverted,
                steerInverted,
                name.toUpperCase(),
                tickOffset
        );
    }

    /**
     * Home all 4 modules simultaneously with dual-stage homing.
     * Call after waitForStart().
     * Each module independently runs: fast approach → back off → slow approach.
     * Returns true if all modules homed successfully.
     */
    public boolean homeAllModules(LinearOpMode opMode) {
        SwerveModule[] modules = {fl, fr, bl, br};
        boolean[] done = new boolean[4];
        long startTime = System.currentTimeMillis();

        // Start all modules homing (or finish immediately if already at switch)
        for (int i = 0; i < 4; i++) {
            if (modules[i].isLimitSwitchPressed()) {
                modules[i].finishHoming();
                done[i] = true;
            } else {
                modules[i].startHoming();
            }
        }

        // Each module advances its own state machine independently
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
                    if (!done[i]) {
                        modules[i].hold();
                    }
                }
                return false;
            }
        }

        return true;
    }

    public void drive(double fwd, double str, double rot) {
        double[][] pos = DriveConstants.WHEEL_POS;
        SwerveModule[] mod = {fl, fr, bl, br};

        // Scale rot so full stick = full motor speed
        double wheelDist = Math.sqrt(pos[0][0] * pos[0][0] + pos[0][1] * pos[0][1]);
        rot /= wheelDist;

        // When rotating, disable translation to prevent flip-causing combined angles
        if (Math.abs(rot) > 0.01) {
            fwd = 0;
            str = 0;
        }
        if (Math.abs(fwd)> 0.01 || Math.abs(str) > 0.01){
            rot =0;
        }

        double max = 0;
        double[] ang = new double[4];
        double[] spd = new double[4];

        for (int i = 0; i < 4; i++) {
            double vx = fwd + (-rot * pos[i][1]);
            double vy = str + ( rot * pos[i][0]);
            ang[i] = Math.atan2(vy, vx);
            spd[i] = Math.sqrt(vx * vx + vy * vy);
            if (spd[i] > max) max = spd[i];
        }
        if (max > 1.0) for (int i = 0; i < 4; i++) spd[i] /= max;
        for (int i = 0; i < 4; i++) mod[i].set(ang[i], spd[i]);
    }

    /**
     * Steer all wheels to angle 0 (forward) without driving.
     */
    public void steerAllToZero() {
        fl.steerToAngle(0);
        fr.steerToAngle(0);
        bl.steerToAngle(0);
        br.steerToAngle(0);
    }

    /**
     * Check if all modules are within tolerance of angle 0.
     */
    public boolean allAtZero(double toleranceRad) {
        return fl.isAtAngle(0, toleranceRad)
                && fr.isAtAngle(0, toleranceRad)
                && bl.isAtAngle(0, toleranceRad)
                && br.isAtAngle(0, toleranceRad);
    }

    /**
     * Reset all encoders so current position = 0.
     */
    public void resetAllEncoders() {
        fl.resetEncoder();
        fr.resetEncoder();
        bl.resetEncoder();
        br.resetEncoder();
    }

    public void hold() {
        fl.hold();
        fr.hold();
        bl.hold();
        br.hold();
    }

    public void zero() {
        fl.set(0, 0);
        fr.set(0, 0);
        bl.set(0, 0);
        br.set(0, 0);
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
