package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

/**
 * Test OpMode to diagnose angle calculation issues.
 * Shows raw voltages, calculated angles, and all intermediate values.
 * NO PID - just displays what the angle calculation is doing.
 */
@TeleOp(name = "Angle Calc Test", group = "Test")
public class AngleCalcTest extends LinearOpMode {

    private AnalogInput flEnc, frEnc, blEnc, brEnc;
    private CRServo flServo, frServo, blServo, brServo;
    private AxonEncoder flEncoder, frEncoder, blEncoder, brEncoder;

    // Continuous tracking variables (mirrors SwerveModule)
    private double[] prevRawAngleDeg = new double[4];
    private double[] unwrappedAngleDeg = new double[4];
    private boolean[] firstUpdate = {true, true, true, true};

    @Override
    public void runOpMode() {
        // Raw analog inputs
        flEnc = hardwareMap.get(AnalogInput.class, "fl_enc");
        frEnc = hardwareMap.get(AnalogInput.class, "fr_enc");
        blEnc = hardwareMap.get(AnalogInput.class, "bl_enc");
        brEnc = hardwareMap.get(AnalogInput.class, "br_enc");

        // Servos (to keep encoders powered)
        flServo = hardwareMap.get(CRServo.class, "fl_servo");
        frServo = hardwareMap.get(CRServo.class, "fr_servo");
        blServo = hardwareMap.get(CRServo.class, "bl_servo");
        brServo = hardwareMap.get(CRServo.class, "br_servo");

        // AxonEncoder instances
        flEncoder = new AxonEncoder(flEnc, SteeringConstants.FL_VOLTAGE_OFFSET);
        frEncoder = new AxonEncoder(frEnc, SteeringConstants.FR_VOLTAGE_OFFSET);
        blEncoder = new AxonEncoder(blEnc, SteeringConstants.BL_VOLTAGE_OFFSET);
        brEncoder = new AxonEncoder(brEnc, SteeringConstants.BR_VOLTAGE_OFFSET);

        telemetry.addLine("=== ANGLE CALCULATION TEST ===");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Keep servos powered for encoder readings
            flServo.setPower(0.01);
            frServo.setPower(0.01);
            blServo.setPower(0.01);
            brServo.setPower(0.01);

            // Get raw voltages
            double flV = flEnc.getVoltage();
            double frV = frEnc.getVoltage();
            double blV = blEnc.getVoltage();
            double brV = brEnc.getVoltage();

            // Get angles from AxonEncoder
            double flRaw180 = flEncoder.getRaw180();
            double frRaw180 = frEncoder.getRaw180();
            double blRaw180 = blEncoder.getRaw180();
            double brRaw180 = brEncoder.getRaw180();

            // Calculate wheel angles (mimics SwerveModule.getAngle())
            double flWheel = calcWheelAngle(0, flRaw180);
            double frWheel = calcWheelAngle(1, frRaw180);
            double blWheel = calcWheelAngle(2, blRaw180);
            double brWheel = calcWheelAngle(3, brRaw180);

            telemetry.addLine("=== RAW VOLTAGES ===");
            telemetry.addData("FL", "%.3fV (offset: %.3f)", flV, SteeringConstants.FL_VOLTAGE_OFFSET);
            telemetry.addData("FR", "%.3fV (offset: %.3f)", frV, SteeringConstants.FR_VOLTAGE_OFFSET);
            telemetry.addData("BL", "%.3fV (offset: %.3f)", blV, SteeringConstants.BL_VOLTAGE_OFFSET);
            telemetry.addData("BR", "%.3fV (offset: %.3f)", brV, SteeringConstants.BR_VOLTAGE_OFFSET);

            telemetry.addLine("");
            telemetry.addLine("=== RAW ANGLE (from encoder, [-180,180]) ===");
            telemetry.addData("FL", "%.1f°", flRaw180);
            telemetry.addData("FR", "%.1f°", frRaw180);
            telemetry.addData("BL", "%.1f°", blRaw180);
            telemetry.addData("BR", "%.1f°", brRaw180);

            telemetry.addLine("");
            telemetry.addLine("=== UNWRAPPED SERVO ANGLE ===");
            telemetry.addData("FL", "%.1f°", unwrappedAngleDeg[0]);
            telemetry.addData("FR", "%.1f°", unwrappedAngleDeg[1]);
            telemetry.addData("BL", "%.1f°", unwrappedAngleDeg[2]);
            telemetry.addData("BR", "%.1f°", unwrappedAngleDeg[3]);

            telemetry.addLine("");
            telemetry.addLine("=== WHEEL ANGLE (after /2 and wrap) ===");
            telemetry.addData("FL", "%.1f°", flWheel);
            telemetry.addData("FR", "%.1f°", frWheel);
            telemetry.addData("BL", "%.1f°", blWheel);
            telemetry.addData("BR", "%.1f°", brWheel);

            telemetry.addLine("");
            telemetry.addLine("=== CLAMPED WHEEL ANGLE (±90°) ===");
            telemetry.addData("FL", "%.1f°", clamp90(flWheel));
            telemetry.addData("FR", "%.1f°", clamp90(frWheel));
            telemetry.addData("BL", "%.1f°", clamp90(blWheel));
            telemetry.addData("BR", "%.1f°", clamp90(brWheel));

            telemetry.addLine("");
            telemetry.addLine("Rotate wheels by hand to test");
            telemetry.addLine("All wheels forward should show ~0°");
            telemetry.addLine("Clamped values stay within ±90°");

            telemetry.update();
        }

        // Stop servos
        flServo.setPower(0);
        frServo.setPower(0);
        blServo.setPower(0);
        brServo.setPower(0);
    }

    private double clamp90(double angleDeg) {
        if (angleDeg > 90.0) return 90.0;
        if (angleDeg < -90.0) return -90.0;
        return angleDeg;
    }

    /**
     * Mimics SwerveModule.getAngle() logic
     */
    private double calcWheelAngle(int index, double rawServoDeg) {
        if (firstUpdate[index]) {
            prevRawAngleDeg[index] = rawServoDeg;
            unwrappedAngleDeg[index] = rawServoDeg;
            firstUpdate[index] = false;
        }

        // Unwrap delta
        double delta = rawServoDeg - prevRawAngleDeg[index];
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        unwrappedAngleDeg[index] += delta;
        prevRawAngleDeg[index] = rawServoDeg;

        // 2:1 gear ratio
        double wheelAngleDeg = unwrappedAngleDeg[index] / 2.0;
        return MathUtils.wrap180(wheelAngleDeg);
    }
}
