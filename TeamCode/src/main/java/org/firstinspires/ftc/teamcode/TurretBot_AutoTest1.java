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
import org.firstinspires.ftc.teamcode.RobotParts.TurretBot.TB_Turret;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "TurretBot_AutoTest1", group = "")
//@Disabled
public class TurretBot_AutoTest1 extends LinearOpMode {

    public Parts parts;

    boolean teamBlue = true;
    boolean startNear = true;

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
//        parts.fieldStartPosition = new Position(0,0,180);  // center, facing back ball
//        parts.fieldStartPosition = TB_Misc.fieldStartPosition;
//        parts.fieldStartPosition = TB_Misc.redOrBlue(TB_Misc.fieldStartPositionBlue);  //todo: Fix this hack?
            // reminder: Y goes away from red.

        parts.speedMaximum = 1;

        TB_Misc.setAuto();
        if (teamBlue) TB_Misc.setAllianceBlue(); else TB_Misc.setAllianceRed();
        parts.fieldStartPosition = TB_Misc.fieldStartPosition;
//        parts.pinpoint.pinpointFieldStart = TB_Misc.fieldStartPosition; // not constructed yet

        settingsOverride();

        parts.setup();
        parts.pinpoint.pinpointFieldStart = TB_Misc.fieldStartPosition;
        parts.drivetrain.accelControl = false;
        parts.preInit();

        parts.controls.controlMode=100;

        TelemetryMgr.setDebugLevel(10);
        TelemetryMgr.enableAllCategories();

        /* Init Loop */
        while (!isStarted()) {
            TelemetryMgr.message(Category.MANDATORY, "Press Play to start");
            TelemetryMgr.message(Category.MANDATORY, "Heading", (parts.positionMgr.headingOnly==null) ? "(null)" : parts.positionMgr.headingOnly.toString(2));
            TelemetryMgr.message(Category.MANDATORY, "(Press UP) Team,", teamBlue ? "Blue" : "Red");
            TelemetryMgr.message(Category.MANDATORY, "(Press Down) Near/Far,", startNear ? "Near" : "Far");
            parts.initLoop();
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasTapped)) {
                teamBlue = !teamBlue;
                if (teamBlue) TB_Misc.setAllianceBlue(); else TB_Misc.setAllianceRed();
                parts.fieldStartPosition = TB_Misc.fieldStartPosition;
                parts.pinpoint.pinpointFieldStart = TB_Misc.fieldStartPosition;
                parts.pinpoint.preRun();
            }
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasTapped)) {
                startNear = !startNear;
                if (startNear) TB_Misc.setAutoNear(); else TB_Misc.setAutoFar();
                parts.fieldStartPosition = TB_Misc.fieldStartPosition;
                parts.pinpoint.pinpointFieldStart = TB_Misc.fieldStartPosition;
                parts.pinpoint.preRun();
            }

            sleep(20);
        }

//        if (teamBlue) TB_Misc.setAllianceBlue();
//        else TB_Misc.setAllianceRed();

        parts.preRun();

//        TB_TasksAuto.smAutoTest1.restart();   // get the ball rolling
        TB_Turret.armTurret(false);
        TB_Turret.armSpinner(false);

        /* Run Loop */
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                parts.autoRunLoop();
            }
        }

        parts.stop();
    }

    public void settingsOverride(){
    }
}
