package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightHardware {
    public Limelight3A limelight;

    public LimelightHardware(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0); // Default AprilTag pipeline
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }
}
