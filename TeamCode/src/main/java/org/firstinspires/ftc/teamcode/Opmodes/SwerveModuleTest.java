package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Module Test", group = "Test")
public class SwerveModuleTest extends LinearOpMode {
    private SwerveModule fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fl = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "fl"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "fl_enc"), SteeringConstants.FL_VOLTAGE_OFFSET),
                false, true, "FL"
        );

        fr = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "fr"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "fr_enc"), SteeringConstants.FR_VOLTAGE_OFFSET),
                false, true, "FR"
        );

        bl = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "bl"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "bl_enc"), SteeringConstants.BL_VOLTAGE_OFFSET),
                false, true, "BL"
        );

        br = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "br"),
                hardwareMap.get(CRServo.class, "br_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "br_enc"), SteeringConstants.BR_VOLTAGE_OFFSET),
                false, true, "BR"
        );

        waitForStart();

        double targetDegrees = 0;

        fl.set(0, 0);
        fr.set(0, 0);
        bl.set(0, 0);
        br.set(0, 0);

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) { targetDegrees += 5; sleep(100); }
            if (gamepad1.dpad_left) { targetDegrees -= 5; sleep(100); }
            if (gamepad1.right_bumper) { targetDegrees += 45; sleep(200); }
            if (gamepad1.left_bumper) { targetDegrees -= 45; sleep(200); }
            if (gamepad1.a) { targetDegrees = 0; sleep(200); }

            while (targetDegrees >= 360) targetDegrees -= 360;
            while (targetDegrees < 0) targetDegrees += 360;

            double targetRad = Math.toRadians(targetDegrees);
            fl.set(targetRad, 0);
            fr.set(targetRad, 0);
            bl.set(targetRad, 0);
            br.set(targetRad, 0);

            telemetry.addLine("Target: " + String.format("%.0f\u00b0", targetDegrees));
            telemetry.addLine("DPad L/R: \u00b15\u00b0 | Bumpers: \u00b145\u00b0 | A: Reset");
            telemetry.addLine();
            fl.log(telemetry);
            fr.log(telemetry);
            bl.log(telemetry);
            br.log(telemetry);
            telemetry.update();
        }
    }
}
