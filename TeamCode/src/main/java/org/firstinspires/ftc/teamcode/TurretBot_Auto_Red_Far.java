package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "4 TurretBot Red Far", group = "14273")
//@Disabled
public class TurretBot_Auto_Red_Far extends TurretBot_Auto_Blue_Near {

    @Override
    public void settingsOverride() {
        teamBlue = false;
        startNear = false;
    }
}
