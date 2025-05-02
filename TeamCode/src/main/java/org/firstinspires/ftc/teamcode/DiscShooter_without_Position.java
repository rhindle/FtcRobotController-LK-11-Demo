package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DiscShooter_without_Position", group = "")
//@Disabled
public class DiscShooter_without_Position extends DiscShooter_with_Position {

    @Override
    public void settingsOverride(){
        parts.useSlamra = false;
        parts.useAprilTag = false;
    }
}
