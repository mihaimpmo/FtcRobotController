package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Hardware.LimelightHardware;
import java.util.List;

public class Camera {
    private final LimelightHardware limelightHardware;
    private final Constants constants;


    public Camera(LimelightHardware limelightHardware) {
        this.limelightHardware = limelightHardware;
        constants = new Constants();
    }
    private int targetTagId = -1;
    private double lastTx = 0;
    private double lastTy = 0;
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

        targetTagId = targetFiducial.getFiducialId();

    }
    public boolean hasTarget() {
        return hasValidTarget;
    }
    public double getTx() {
        return lastTx;
    } //get horizontal offset in degress
    public double getTy() {
        return lastTy;
    } //get vertical offset in degress
    public double getTID() {return targetTagId;} //get target id

    // Set the camera pipeline
    public void setPipeline(int pipeline) {
        if (limelightHardware.limelight != null) {
            limelightHardware.limelight.pipelineSwitch(pipeline);
        }
    }
    public boolean isConnected() {
        return limelightHardware.limelight != null;
    }

    // Get horizontal distance to target
    public double hdistance(double ty) {
        return (constants.h2-constants.h1)/Math.tan(constants.deg2rad(constants.a+ty));
    }
}