package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.PartsSB;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "Spintake_Test1", group = "")
//@Disabled
public class Spintake_Test1 extends LinearOpMode {

    public Parts parts;

    boolean teamBlue = false;

    @Override
    public void runOpMode() {

        parts = new PartsSB(this);

        parts.useODO = false;
        parts.useIMU = false;
        parts.useSlamra = false;
        parts.useNeoMatrix = false;
        parts.useAprilTag = false;
        parts.useDrivetrainEncoders = true;
        parts.reverseDrive = false;
        parts.useDistanceSensors = false;
        parts.odoRobotOffset = new Position (2.25,0,0);  // if this is inherent to the robot, should it be in PartsDS?
        parts.slamraRobotOffset = new Position(-8,-0.75,0); //new Position(-8,-1,0);
        parts.speedMaximum = 1;
//        DSAprilTag.USE_WEBCAM = true;

        settingsOverride();

        parts.setup();
        parts.preInit();

        TelemetryMgr.setDebugLevel(10);
        TelemetryMgr.enableAllCategories();

        /* Init Loop */
        while (!isStarted()) {
            TelemetryMgr.message(Category.MANDATORY, "Press Play to start");
            TelemetryMgr.message(Category.MANDATORY, "Heading", (parts.positionMgr.headingOnly==null) ? "(null)" : parts.positionMgr.headingOnly.toString(2));
            TelemetryMgr.message(Category.MANDATORY, "Team,", teamBlue ? "Blue" : "Red");
            parts.initLoop();
            if (parts.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasTapped))
                teamBlue = !teamBlue;
            sleep(20);
        }

//        DSMisc.autoLaunchPos = teamBlue ? DSMisc.autoLaunchPosKnowlton : DSMisc.autoLaunchPosDemo;

        parts.preRun();

        /* Run Loop */
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                parts.runLoop();
            }
        }

        parts.stop();
    }

    public void settingsOverride(){
    }
}
