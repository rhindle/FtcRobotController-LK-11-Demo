package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.PartsDS;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

@TeleOp(name = "DiscShooter_without_Position", group = "")
//@Disabled
public class DiscShooter_without_Position extends LinearOpMode {

   public Parts parts;

   @Override
   public void runOpMode() {

      parts = new PartsDS(this);

      parts.useODO = false;
      parts.useIMU = true;   //!!!
      parts.useSlamra = false;  //!!!
      parts.useNeoMatrix = true;
      parts.useAprilTag = false;
      parts.useDrivetrainEncoders = true;
      parts.reverseDrive = false;
      parts.useDistanceSensors = false;
      parts.odoRobotOffset = new Position (2.25,0,0);  // if this is inherent to the robot, should it be in PartsDS?
      parts.slamraRobotOffset = new Position(-8,-1,0);
      parts.speedMaximum = 1;

      parts.setup();
      parts.preInit();

      TelemetryMgr.setDebugLevel(10);
//      TelemetryMgr.enableCategories(new TelemetryMgr.Category[] {Category.AUTODRIVE});
      TelemetryMgr.enableAllCategories();

      /* Init Loop */
      while (!isStarted()) {
         TelemetryMgr.message(Category.MANDATORY, "Press Play to start");
//         if (parts.useIMU) TelemetryMgr.message(Category.MANDATORY, "Heading","%.1f", parts.imuMgr.returnImuHeadingRaw(true));
         TelemetryMgr.message(Category.MANDATORY, "Heading", (parts.positionMgr.headingOnly==null) ? "(null)" : parts.positionMgr.headingOnly.toString(2));
         TelemetryMgr.message(Category.MANDATORY, "Drive Type", parts.reverseDrive ? "AndyMark" : "GobildaBot");

         parts.initLoop();

         if (parts.buttonMgr.wasTapped(1, ButtonMgr.Buttons.x))
            parts.reverseDrive = !parts.reverseDrive;
         if (parts.buttonMgr.wasTapped(2, ButtonMgr.Buttons.x))
            parts.reverseDrive = !parts.reverseDrive;

//         TelemetryMgr.Update(true);
         sleep(20);
      }

      parts.preRun();

      /* Run Loop */
      if (opModeIsActive()) {
         while (opModeIsActive()) {
//            addTelemetryLoopStart();
            parts.runLoop();
//            addTelemetryLoopEnd();
//            TelemetryMgr.Update();
         }
      }

      parts.stop();

   }
}
