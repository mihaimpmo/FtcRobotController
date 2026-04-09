package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Custom Quadrature Encoder", group = "Test")
public class Encoder extends LinearOpMode {

    private DigitalChannel pinA;
    private DigitalChannel pinB;

    private long count = 0;
    private int lastState = 0;

    // IMPORTANT: counts per revolution
    private static final int CPR = 112;   // modifică dacă e nevoie

    @Override
    public void runOpMode() throws InterruptedException {

        // Numele trebuie să fie exact cele din Robot Configuration
        pinA = hardwareMap.get(DigitalChannel.class, "Channel1");
        pinB = hardwareMap.get(DigitalChannel.class, "Channel2");

        pinA.setMode(DigitalChannel.Mode.INPUT);
        pinB.setMode(DigitalChannel.Mode.INPUT);

        // Citește starea inițială
        lastState = readState();

        telemetry.addLine("Quadrature encoder ready");
        telemetry.addData("Initial State", lastState);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateEncoder();

            double angle = (count * 360.0) / CPR;

            telemetry.addData("Count", count);
            telemetry.addData("Angle", angle);
            telemetry.addData("A", pinA.getState() ? 1 : 0);
            telemetry.addData("B", pinB.getState() ? 1 : 0);
            telemetry.addData("State", lastState);
            telemetry.update();

            // Foarte important:
            // fără sleep(), bucla rulează cât de repede poate
            // și maximizează șansele să nu ratezi tranziții.
            idle();
        }
    }

    private int readState() {
        int a = pinA.getState() ? 1 : 0;
        int b = pinB.getState() ? 1 : 0;
        return (a << 1) | b;
    }

    private void updateEncoder() {
        int currentState = readState();

        if (currentState == lastState) {
            return;
        }

        // Tabel de tranziții quadrature
        if ((lastState == 0 && currentState == 1) ||
                (lastState == 1 && currentState == 3) ||
                (lastState == 3 && currentState == 2) ||
                (lastState == 2 && currentState == 0)) {
            count++;   // CW
        }
        else if ((lastState == 0 && currentState == 2) ||
                (lastState == 2 && currentState == 3) ||
                (lastState == 3 && currentState == 1) ||
                (lastState == 1 && currentState == 0)) {
            count--;   // CCW
        }

        lastState = currentState;
    }
}