package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Module Test", group = "Test")
public class SwerveModuleTest extends LinearOpMode {
    private SwerveModule fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fl = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "fl"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.FL_ENCODER_NAME), SteeringConstants.FL_ENCODER_SHARED),
                hardwareMap.get(DigitalChannel.class, SteeringConstants.FL_SWITCH_NAME),
                false, true, "FL",
                SteeringConstants.FL_TICK_OFFSET
        );

        fr = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "fr"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.FR_ENCODER_NAME), SteeringConstants.FR_ENCODER_SHARED),
                hardwareMap.get(DigitalChannel.class, SteeringConstants.FR_SWITCH_NAME),
                false, true, "FR",
                SteeringConstants.FR_TICK_OFFSET
        );

        bl = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "bl"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.BL_ENCODER_NAME), SteeringConstants.BL_ENCODER_SHARED),
                hardwareMap.get(DigitalChannel.class, SteeringConstants.BL_SWITCH_NAME),
                false, true, "BL",
                SteeringConstants.BL_TICK_OFFSET
        );

        br = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "br"),
                hardwareMap.get(CRServo.class, "br_servo"),
                new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.BR_ENCODER_NAME), SteeringConstants.BR_ENCODER_SHARED),
                hardwareMap.get(DigitalChannel.class, SteeringConstants.BR_SWITCH_NAME),
                false, true, "BR",
                SteeringConstants.BR_TICK_OFFSET
        );

        telemetry.addLine("Swerve Module Test");
        telemetry.addLine("Press START to home and begin");
        telemetry.update();

        waitForStart();

        // Home all modules
        telemetry.addLine("Homing...");
        telemetry.update();
        fl.homeModule(this);
        fr.homeModule(this);
        bl.homeModule(this);
        br.homeModule(this);

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
