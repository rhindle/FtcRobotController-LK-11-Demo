package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.PartsTB;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Misc;
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_TasksAuto;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "TurretBot_AutoTest1", group = "")
//@Disabled
public class TurretBot_AutoTest1 extends LinearOpMode {

    public Parts parts;

    boolean teamBlue = false;

    @Override
    public void runOpMode() {

        parts = new PartsTB(this);

        parts.useODO = false;
        parts.usePinpoint = true;
        parts.useLimeLight = false;
        parts.useIMU = false;
        parts.useSlamra = false;
        parts.useNeoMatrix = false;
        parts.useAprilTag = false;
        parts.useDrivetrainEncoders = true;
        parts.reverseDrive = false;
        parts.useDistanceSensors = false;
        parts.fieldStartPosition = new Position(0,0,180);  // center, facing back ball
            // reminder: Y goes away from red.

        parts.speedMaximum = 1;

        settingsOverride();

        parts.setup();
        parts.preInit();

        TB_Misc.forza = true;

        TelemetryMgr.setDebugLevel(10);
        TelemetryMgr.enableAllCategories();

        /* Init Loop */
        while (!isStarted()) {
            TelemetryMgr.message(Category.MANDATORY, "Press Play to start");
            TelemetryMgr.message(Category.MANDATORY, "Heading", (parts.positionMgr.headingOnly==null) ? "(null)" : parts.positionMgr.headingOnly.toString(2));
            TelemetryMgr.message(Category.MANDATORY, "(Press UP) Team,", teamBlue ? "Blue" : "Red");
            TelemetryMgr.message(Category.MANDATORY, "(Press X) Drive", TB_Misc.forza ? "Forza" : "Arcade");
            parts.initLoop();
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasTapped))
                teamBlue = !teamBlue;
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasPressed))
                TB_Misc.forza = !TB_Misc.forza;

            sleep(20);
        }

        if (teamBlue) TB_Misc.setAllianceBlue();
        else TB_Misc.setAllianceRed();


        parts.preRun();

        TB_TasksAuto.smAutoTest1.restart();   // get the ball rolling

        /* Run Loop */
        if (opModeIsActive()) {
            while (opModeIsActive()) {
//                parts.runLoop();
                parts.autoRunLoop();
            }
        }

        parts.stop();
    }

    public void settingsOverride(){
    }
}
