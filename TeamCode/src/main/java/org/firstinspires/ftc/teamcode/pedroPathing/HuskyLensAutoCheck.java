package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HuskyLens Auto Check", group = "Pedro Pathing")
public class HuskyLensAutoCheck extends HuskyLensPoseCheck {
    @Override
    protected boolean startOnOpModeStart() {
        return true;
    }
}
