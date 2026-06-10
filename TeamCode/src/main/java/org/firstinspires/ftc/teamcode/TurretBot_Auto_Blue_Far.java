package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "3 TurretBot Blue Far", group = "14273")
//@Disabled
public class TurretBot_Auto_Blue_Far extends TurretBot_Auto_Blue_Near {

    @Override
    public void settingsOverride() {
        teamBlue = true;
        startNear = false;
    }
}
