package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Hardware.LimelightHardware;

import java.util.List;
import java.util.Locale;

public class Camera {
    private final LimelightHardware limelightHardware;
    private Constants constants;


    public Camera(LimelightHardware limelightHardware) {
        this.limelightHardware = limelightHardware;
        constants = new Constants();
    }
    private int targetTagId = -1;
    private double lastDistanceMeters = 0;
    private double lastHeightMeters = 0;
    private double lastTx = 0;
    private double lastTy = 0;
    private int lastTrackedTagId = -1;
    private boolean hasValidTarget = false;

    public void setTargetTagId(int tagId) {
        this.targetTagId = tagId;
    }
    public void update() {
        if (limelightHardware.limelight == null) {
            hasValidTarget = false;
            return;
        }

        LLResult result = limelightHardware.limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasValidTarget = false;
            return;
        }

        // Cache raw angle offsets
        lastTx = result.getTx();
        lastTy = result.getTy();


        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            hasValidTarget = false;
            return;
        }

        // Find the target tag (or use first visible if targetTagId is -1)
        LLResultTypes.FiducialResult targetFiducial = null;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (targetTagId == -1 || fiducial.getFiducialId() == targetTagId) {
                targetFiducial = fiducial;
                break;
            }
        }

        if (targetFiducial == null) {
            hasValidTarget = false;
            return;
        }

        lastTrackedTagId = targetFiducial.getFiducialId();

    }
    public boolean hasTarget() {
        return hasValidTarget;
    }
    public int getTrackedTagId() {
        return hasValidTarget ? lastTrackedTagId : -1;
    }
    public double getDistanceToGoalMeters() {
        return lastDistanceMeters;
    }
    public double getHeightToGoalMeters() {
        return lastHeightMeters;
    }
    public double getTx() {
        return lastTx;
    }
    public double getTy() {
        return lastTy;
    }
    public double getTID() {return targetTagId;}
    public void setPipeline(int pipeline) {
        if (limelightHardware.limelight != null) {
            limelightHardware.limelight.pipelineSwitch(pipeline);
        }
    }
    public boolean isConnected() {
        return limelightHardware.limelight != null;
    }
    public double hdistance(double ty) {
        return (constants.h2-constants.h1)/Math.tan(constants.deg2rad(constants.a+ty));
    }
}