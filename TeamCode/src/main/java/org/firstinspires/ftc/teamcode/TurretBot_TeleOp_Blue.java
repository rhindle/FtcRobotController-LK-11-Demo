package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.PartsTB;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Intake;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Misc;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Tasks;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Turret;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "2 TurretBot BLUE Alliance", group = "14273")
@Disabled
public class TurretBot_TeleOp_Blue extends TurretBot_TeleOp_AutoAlliance {

    @Override
    public void settingsOverride() {
        // team color?  better to hold from autonomous?
        teamBlue = true;
        if (teamBlue) TB_Misc.setAllianceBlue(); else TB_Misc.setAllianceRed();
    }

    @Override
    public void settingsOverrideParts() {
//        parts.drivetrain.accelControl = true;
//        parts.drivetrain.accelControlRamp = 125;
    }
}
