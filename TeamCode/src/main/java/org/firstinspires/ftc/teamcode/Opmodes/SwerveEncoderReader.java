package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;
@Disabled
@TeleOp(name = "Swerve Encoder Reader", group = "Test")
public class SwerveEncoderReader extends LinearOpMode {

    @Override
    public void runOpMode() {
        RevThroughBoreEncoder flEnc, frEnc, blEnc, brEnc;
        CRServo flServo, frServo, blServo, brServo;

        try {
            flEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.FL_ENCODER_NAME), SteeringConstants.FL_ENCODER_SHARED);
            frEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.FR_ENCODER_NAME), SteeringConstants.FR_ENCODER_SHARED);
            blEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.BL_ENCODER_NAME), SteeringConstants.BL_ENCODER_SHARED);
            brEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.BR_ENCODER_NAME), SteeringConstants.BR_ENCODER_SHARED);

            flServo = hardwareMap.get(CRServo.class, "fl_servo");
            frServo = hardwareMap.get(CRServo.class, "fr_servo");
            blServo = hardwareMap.get(CRServo.class, "bl_servo");
            brServo = hardwareMap.get(CRServo.class, "br_servo");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find all devices. Please check your configuration.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        telemetry.addLine("Swerve Encoder Reader (Rev Through Bore)");
        telemetry.addLine("Shows raw ticks and computed angles.");
        telemetry.addLine("No homing — just raw encoder data.");
        telemetry.update();

        waitForStart();

        // Hold servos with minimal power
        flServo.setPower(0.01);
        frServo.setPower(0.01);
        blServo.setPower(0.01);
        brServo.setPower(0.01);

        while (opModeIsActive()) {
            displayModule("FL", flEnc);
            displayModule("FR", frEnc);
            displayModule("BL", blEnc);
            displayModule("BR", brEnc);
            telemetry.update();
        }

        flServo.setPower(0);
        frServo.setPower(0);
        blServo.setPower(0);
        brServo.setPower(0);
    }

    private void displayModule(String name, RevThroughBoreEncoder encoder) {
        int rawTicks = encoder.getRawTicks();
        double encoderAngleDeg = ((rawTicks % SteeringConstants.ENCODER_TICKS_PER_REV) / (double) SteeringConstants.ENCODER_TICKS_PER_REV) * 360.0;
        if (encoderAngleDeg < 0) encoderAngleDeg += 360.0;
        double wheelAngleDeg = ((rawTicks % SteeringConstants.TICKS_PER_WHEEL_REV) / (double) SteeringConstants.TICKS_PER_WHEEL_REV) * 360.0;
        if (wheelAngleDeg < 0) wheelAngleDeg += 360.0;
        telemetry.addLine("--- " + name + " ---");
        telemetry.addData("  Raw Ticks", "%d", rawTicks);
        telemetry.addData("  Encoder Angle", "%.1f deg (per enc rev)", encoderAngleDeg);
        telemetry.addData("  Wheel Angle", "%.1f deg (per wheel rev)", wheelAngleDeg);
    }
}
