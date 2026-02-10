package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Direction Test", group = "Diagnostic")
public class MotorDirectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor fl_motor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr_motor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor bl_motor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor br_motor = hardwareMap.get(DcMotor.class, "br");

        telemetry.addLine("Motor Direction Test Ready");
        telemetry.addLine("Use D-Pad to test each motor individually.");
        telemetry.addLine("D-Pad UP: Front-Left");
        telemetry.addLine("D-Pad RIGHT: Front-Right");
        telemetry.addLine("D-Pad DOWN: Back-Left");
        telemetry.addLine("D-Pad LEFT: Back-Right");
        telemetry.addLine("Press STOP to end.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            fl_motor.setPower(0);
            fr_motor.setPower(0);
            bl_motor.setPower(0);
            br_motor.setPower(0);

            String activeMotor = "None";

            if (gamepad1.dpad_up) {
                fl_motor.setPower(0.3);
                activeMotor = "Front-Left (fl)";
            } else if (gamepad1.dpad_right) {
                fr_motor.setPower(0.3);
                activeMotor = "Front-Right (fr)";
            } else if (gamepad1.dpad_down) {
                bl_motor.setPower(0.3);
                activeMotor = "Back-Left (bl)";
            } else if (gamepad1.dpad_left) {
                br_motor.setPower(0.3);
                activeMotor = "Back-Right (br)";
            }

            telemetry.addData("Active Motor", activeMotor);
            telemetry.addData("Instructions", "Observe the spin direction of the active wheel.");
            telemetry.update();
        }
    }
}
