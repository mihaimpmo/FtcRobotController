package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "LeaveAuto", group = "Auto")
public class SimpleAuto extends Auto {

    @Override
    protected void runPath() {
        autoDrive.forward(24.0);
    }
}
