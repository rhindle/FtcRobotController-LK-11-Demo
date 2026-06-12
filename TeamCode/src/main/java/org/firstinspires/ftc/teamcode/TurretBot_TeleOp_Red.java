package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Misc;

@TeleOp(name = "3 TurretBot RED Alliance", group = "14273")
//@Disabled
public class TurretBot_TeleOp_Red extends TurretBot_TeleOp_AutoAlliance {

    @Override
    public void settingsOverride() {
        // team color?  better to hold from autonomous?
        teamBlue = false;
        if (teamBlue) TB_Misc.setAllianceBlue(); else TB_Misc.setAllianceRed();
    }

    @Override
    public void settingsOverrideParts() {
//        parts.drivetrain.accelControl = true;
//        parts.drivetrain.accelControlRamp = 125;
    }
}
