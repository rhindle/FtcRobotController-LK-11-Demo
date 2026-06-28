package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Misc;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "0 TurretBot Demo", group = "14273")
//@Disabled
public class TurretBot_TeleOp_Demo extends TurretBot_TeleOp_AutoAlliance {

    @Override
    public void settingsOverride() {
        parts.usePinpoint = true;
        parts.useLimeLight = true; //false
        parts.fieldStartPosition = new Position(0,0,180);  // center, facing back ball *** UPDATE THIS!!!! ***
        parts.speedMaximum = 1;
        parts.useForzaControls = false;
        TB_Misc.modeDemo = true;
        TB_Misc.speedFactor = (float)TB_Misc.demoTeamSpeedSlow;
        // team color?  change according to demo field
        teamBlue = true;
        if (teamBlue) TB_Misc.setAllianceBlue(); else TB_Misc.setAllianceRed();
    }

    @Override
    public void settingsOverrideParts() {
        // This is for overrides to parts.xxx; won't work until parts.setup() has created all the sub-parts
        parts.controls.controlMode = 10;   // demo controls is 10
    }
}
