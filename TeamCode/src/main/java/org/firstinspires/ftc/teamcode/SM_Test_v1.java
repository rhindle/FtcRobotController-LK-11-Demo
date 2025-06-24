package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.RobotParts.SMTest.PartsSMT;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.PartsT24;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "SM_Test_v1", group = "")
//@Disabled
public class SM_Test_v1 extends LinearOpMode {

    public Parts parts;

    @Override
    public void runOpMode() {

        parts = new PartsSMT(this);

//        parts.useODO = true; //false;
//        parts.useIMU = true;
//        parts.useSlamra = true; //false;
//        parts.useNeoMatrix = false;
//        parts.useEncoderTracker = true;
//        parts.fieldStartPosition = new Position(0,0,0);
//        parts.useAprilTag = false;
//        parts.useDrivetrainEncoders = true;
//        parts.reverseDrive = false;
//        parts.useDistanceSensors = false;
//        parts.odoRobotOffset = new Position (2.25,0,0);  // if this is inherent to the robot, should it be in PartsDS?
//        parts.slamraRobotOffset = new Position(-8,-0.75,0); //new Position(-8,-1,0);
//        parts.speedMaximum = 1;

        settingsOverride();

        parts.setup();
        parts.preInit();

        TelemetryMgr.setDebugLevel(10);
        TelemetryMgr.enableAllCategories();

        /* Init Loop */
        while (!isStarted()) {
            TelemetryMgr.message(Category.MANDATORY, "Press Play to start");
            TelemetryMgr.message(Category.MANDATORY, "Heading", (parts.positionMgr.headingOnly==null) ? "(null)" : parts.positionMgr.headingOnly.toString(2));
            parts.initLoop();
            sleep(20);
        }

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
