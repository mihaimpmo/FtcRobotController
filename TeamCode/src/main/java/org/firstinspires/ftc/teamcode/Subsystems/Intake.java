package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Intake {
    private Hardware hardware;

    public Intake(Hardware hardware) {
        this.hardware = hardware;
    }

    public void Start(double power) {
        hardware.intakeHardware.IntakeMotor.setPower(power);
    }

    public void Stop() {
        hardware.intakeHardware.IntakeMotor.setPower(0);
    }

    public void log(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Intake Power", "%.2f", hardware.intakeHardware.IntakeMotor.getPower());
    }
}
