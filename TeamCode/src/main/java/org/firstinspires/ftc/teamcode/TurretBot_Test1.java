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

@TeleOp(name = "TurretBot_Test1", group = "")
@Disabled
public class TurretBot_Test1 extends LinearOpMode {

    public Parts parts;

//    boolean teamBlue = false;
    boolean teamBlue = TB_Misc.isAllianceBlue();

    @Override
    public void runOpMode() {

        parts = new PartsTB(this);

        parts.useODO = false;
        parts.usePinpoint = true;
        parts.useLimeLight = true; //false
        parts.useIMU = false;
        parts.useSlamra = false;
        parts.useNeoMatrix = false;
        parts.useAprilTag = false;
        parts.useDrivetrainEncoders = true;
        parts.reverseDrive = false;
        parts.useDistanceSensors = false;
//        parts.fieldStartPosition = new Position(36,-63,90);  // red side, to the right, facing forward
        parts.fieldStartPosition = new Position(0,0,180);  // center, facing back ball
            // reminder: Y goes away from red.
//        parts.odoRobotOffset = new Position (2.25,0,0);  // if this is inherent to the robot, should it be in PartsDS?
//        parts.slamraRobotOffset = new Position(-8,-0.75,0); //new Position(-8,-1,0);
//        parts.pinpointRobotOffset = new Position (-56.0,52.0,0);  // In mm, Refer to User Guide, Y offset of X, X offset of Y, R will be ignored
        parts.speedMaximum = 1;
//        DSAprilTag.USE_WEBCAM = true;
        parts.useForzaControls = true;

        TB_Misc.setTeleOp();

        settingsOverride();

        parts.setup();
        parts.preInit();

        TelemetryMgr.setDebugLevel(10);
        TelemetryMgr.enableAllCategories();
//        parts.opMode.telemetry.setMsTransmissionInterval(11);

        /* Init Loop */
        while (!isStarted()) {
            TelemetryMgr.message(Category.MANDATORY, "Press Play to start");
            TelemetryMgr.message(Category.MANDATORY, "Heading", (parts.positionMgr.headingOnly==null) ? "(null)" : parts.positionMgr.headingOnly.toString(2));
            TelemetryMgr.message(Category.MANDATORY, "(Press UP) Team", teamBlue ? "Blue" : "Red");
            TelemetryMgr.message(Category.MANDATORY, "(Press X) Drive",  parts.useForzaControls ? "Forza" : "Arcade");
            TelemetryMgr.message(Category.MANDATORY, "(Press B) SlowShoot", TB_Turret.slowForTest ? "True" : "False");
            TelemetryMgr.message(Category.MANDATORY, "(Press DOWN) Center Position");
            parts.initLoop();
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasTapped)) {
                teamBlue = !teamBlue;
                if (teamBlue) TB_Misc.setAllianceBlue(); else TB_Misc.setAllianceRed();
            }
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasPressed))
                parts.useForzaControls = !parts.useForzaControls;
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.b, ButtonMgr.State.wasPressed))
                TB_Turret.slowForTest = !TB_Turret.slowForTest;
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasPressed))
                parts.pinpoint.setPosition(parts.fieldStartPosition);

            sleep(20);
        }

        if (teamBlue) TB_Misc.setAllianceBlue();
        else TB_Misc.setAllianceRed();

        parts.preRun();

        // automate things that were manual:
        //   set drive speed to full, arm turret parts, turn on intake
//        TB_Misc.controlsFullSpeed = true;  // driver won't have to press right bumper
        TB_Misc.fullSpeed(true);    // driver won't have to press right bumper
        TB_Turret.armTurret(true);
        TB_Turret.armSpinner(true);
        TB_Intake.intakeRunning = true;
        TB_Tasks.smIntakeOn.restart();

        /* Run Loop */
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                parts.runLoop();
            }
        }

        parts.stop();
    }

    public void settingsOverride(){
        // team color?  better to hold from autonomous?
    }
}

//public Position turretPosition() {
//    return new Position(positionTracker.getOverridePosition()).transformPosition(new Position(-2,0,0));
//}