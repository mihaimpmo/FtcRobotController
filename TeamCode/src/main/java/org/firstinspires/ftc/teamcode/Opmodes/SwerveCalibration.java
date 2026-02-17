package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;

@TeleOp(name = "Swerve Calibration", group = "Calibration")
public class SwerveCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        RevThroughBoreEncoder flEnc, frEnc, blEnc, brEnc;
        DigitalChannel flLimit, frLimit, blLimit, brLimit;
        CRServo flServo, frServo, blServo, brServo;

        try {
            flEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.FL_ENCODER_NAME), SteeringConstants.FL_ENCODER_SHARED);
            frEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.FR_ENCODER_NAME), SteeringConstants.FR_ENCODER_SHARED);
            blEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.BL_ENCODER_NAME), SteeringConstants.BL_ENCODER_SHARED);
            brEnc = new RevThroughBoreEncoder(hardwareMap.get(DcMotorEx.class, SteeringConstants.BR_ENCODER_NAME), SteeringConstants.BR_ENCODER_SHARED);

            flLimit = hardwareMap.get(DigitalChannel.class, SteeringConstants.FL_SWITCH_NAME);
            frLimit = hardwareMap.get(DigitalChannel.class, SteeringConstants.FR_SWITCH_NAME);
            blLimit = hardwareMap.get(DigitalChannel.class, SteeringConstants.BL_SWITCH_NAME);
            brLimit = hardwareMap.get(DigitalChannel.class, SteeringConstants.BR_SWITCH_NAME);

            flServo = hardwareMap.get(CRServo.class, "fl_servo");
            frServo = hardwareMap.get(CRServo.class, "fr_servo");
            blServo = hardwareMap.get(CRServo.class, "bl_servo");
            brServo = hardwareMap.get(CRServo.class, "br_servo");

            for (DigitalChannel ch : new DigitalChannel[]{flLimit, frLimit, blLimit, brLimit}) {
                ch.setMode(DigitalChannel.Mode.INPUT);
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find all devices");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        telemetry.addLine("Swerve Calibration (Rev Through Bore)");
        telemetry.addLine("1. Press START to home all modules");
        telemetry.addLine("2. Manually align all wheels straight forward");
        telemetry.addLine("3. Press A to capture tick offsets");
        telemetry.update();

        waitForStart();

        // Phase 1: Home all modules simultaneously with offset=0 (find limit switches)
        telemetry.addLine("Homing all modules...");
        telemetry.update();

        homeAllModules(
                new CRServo[]{flServo, frServo, blServo, brServo},
                new RevThroughBoreEncoder[]{flEnc, frEnc, blEnc, brEnc},
                new DigitalChannel[]{flLimit, frLimit, blLimit, brLimit},
                new String[]{"FL", "FR", "BL", "BR"}
        );

        telemetry.addLine("Homing complete!");
        telemetry.addLine("Manually align all wheels straight forward.");
        telemetry.addLine("Press A to capture tick offsets.");
        telemetry.update();

        // Phase 2: User aligns wheels, capture offsets
        while (opModeIsActive()) {
            int flTicks = flEnc.getPositionTicks();
            int frTicks = frEnc.getPositionTicks();
            int blTicks = blEnc.getPositionTicks();
            int brTicks = brEnc.getPositionTicks();

            telemetry.addLine("--- Current Ticks (from home) ---");
            telemetry.addData("FL", "%d ticks", flTicks);
            telemetry.addData("FR", "%d ticks", frTicks);
            telemetry.addData("BL", "%d ticks", blTicks);
            telemetry.addData("BR", "%d ticks", brTicks);
            telemetry.addLine();
            telemetry.addLine("Align wheels forward, then press A");

            if (gamepad1.a) {
                telemetry.addLine();
                telemetry.addLine("--- Copy to SteeringConstants.java ---");
                telemetry.addData("FL_TICK_OFFSET", "%d", flTicks);
                telemetry.addData("FR_TICK_OFFSET", "%d", frTicks);
                telemetry.addData("BL_TICK_OFFSET", "%d", blTicks);
                telemetry.addData("BR_TICK_OFFSET", "%d", brTicks);
            }

            telemetry.update();
        }
    }

    private static final int STAGE_FAST = 0, STAGE_BACKOFF = 1, STAGE_SLOW = 2, STAGE_DONE = 3;

    private void homeAllModules(CRServo[] servos, RevThroughBoreEncoder[] encoders,
                                DigitalChannel[] switches, String[] names) {
        int[] stage = new int[4];
        long[] stageStart = new long[4];
        long startTime = System.currentTimeMillis();

        // Start all modules in fast approach
        for (int i = 0; i < 4; i++) {
            if (switches[i].getState() == SteeringConstants.LIMIT_SWITCH_ACTIVE_STATE) {
                // Already at switch — skip straight to done
                servos[i].setPower(0);
                encoders[i].setHomePosition(0);
                stage[i] = STAGE_DONE;
            } else {
                servos[i].setPower(SteeringConstants.HOMING_FAST_POWER);
                stage[i] = STAGE_FAST;
            }
        }

        while (opModeIsActive()) {
            boolean allDone = true;
            for (int i = 0; i < 4; i++) {
                if (stage[i] == STAGE_DONE) continue;
                allDone = false;

                boolean pressed = switches[i].getState() == SteeringConstants.LIMIT_SWITCH_ACTIVE_STATE;

                switch (stage[i]) {
                    case STAGE_FAST:
                        if (pressed) {
                            servos[i].setPower(SteeringConstants.HOMING_BACKOFF_POWER);
                            stageStart[i] = System.currentTimeMillis();
                            stage[i] = STAGE_BACKOFF;
                        }
                        break;
                    case STAGE_BACKOFF:
                        if (System.currentTimeMillis() - stageStart[i] >= SteeringConstants.HOMING_BACKOFF_MS) {
                            servos[i].setPower(SteeringConstants.HOMING_SLOW_POWER);
                            stage[i] = STAGE_SLOW;
                        }
                        break;
                    case STAGE_SLOW:
                        if (pressed) {
                            servos[i].setPower(0);
                            encoders[i].setHomePosition(0);
                            stage[i] = STAGE_DONE;
                        }
                        break;
                }
            }
            if (allDone) break;

            if (System.currentTimeMillis() - startTime > SteeringConstants.HOMING_TIMEOUT_MS) {
                for (int i = 0; i < 4; i++) {
                    if (stage[i] != STAGE_DONE) {
                        servos[i].setPower(0);
                        encoders[i].setHomePosition(0);
                        telemetry.addData(names[i], "HOMING TIMEOUT");
                    }
                }
                telemetry.update();
                break;
            }
        }
    }
}
