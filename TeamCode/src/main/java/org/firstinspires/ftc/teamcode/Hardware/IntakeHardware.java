package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeHardware {
    private static final String intakeMotorName = "IM1";
    public final DcMotor IntakeMotor;

    public IntakeHardware(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class, intakeMotorName);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // RUN_WITHOUT_ENCODER: encoder pins on this port are used by FL swerve steering encoder
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
