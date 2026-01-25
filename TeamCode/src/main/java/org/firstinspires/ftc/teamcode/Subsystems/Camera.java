package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Hardware.LimelightHardware;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class Camera {

    private final Limelight3A limelight;

    public Camera(LimelightHardware limelightHardware) {
        this.limelight = limelightHardware.limelight;
    }

    /** Există AprilTag vizibil */
    public boolean hasAprilTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return false;

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        return fiducials != null && !fiducials.isEmpty();
    }

    /** X = stânga / dreapta (m) */
    public double getAprilTagX() {
        Pose3D pose = getTagPose();
        if (pose == null) return 0;

        Position pos = pose.getPosition();
        return pos.x;
    }

    /** Z = distanța înainte (m) */
    public double getAprilTagZ() {
        Pose3D pose = getTagPose();
        if (pose == null) return 0;

        Position pos = pose.getPosition();
        return pos.z;
    }

    /** Distanță totală */
    public double getAprilTagDistance() {
        double x = getAprilTagX();
        double z = getAprilTagZ();
        return Math.sqrt(x * x + z * z);
    }

    /** Helper intern */
    private Pose3D getTagPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) return null;

        return fiducials
                .get(0)
                .getTargetPoseRobotSpace();
    }
}
