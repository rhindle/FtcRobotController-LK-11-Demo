package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class TB_WFLED {

   /* Public OpMode members. */
   public static Parts parts;

   public static StateMachine smWFLED;

   public static ServoSSR servoWFLED;

   public static int LedMode = 0;
   public static int LedMode0 = LedMode;
   //0 idle, 1 driving, 2 rotating, 3 driving & rotating, 4 aiming, 5 shooting
   public static boolean LedDirection = true;
   public static boolean LedDirection0 = LedDirection;
   // true = left/CCW, false = right/CW
   // 7 left, 8 right
   public static double LedAngle = 0;
   public static double LedAngle0 = LedAngle;
   // 10 through 25 = 16 positions
   public static int LedSpeed = 0;
   public static int LedSpeed0 = LedSpeed;
   // 30 through 36 = 7 speeds
   public final static double LedAngleValue = 360.0 / 16;
   public final static double LedServoValue = 1.0 / 36;
   public static boolean LedDelay = false;
   public final static int LedDelayTime = 60;  // Long enough for two read attempts?

   public static DriveData driveData = new DriveData();

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize(){
      buildStateMachines();
      servoWFLED = new ServoSSR(parts.robotV2.getServoByName("servo1"));
   }

   public static void preRun() {
      if (!TB_Misc.modeDemo) return;
      smWFLED.restart();
   }

   public static void runLoop() {
      if (!TB_Misc.modeDemo) return;
      if (TB_Intake.transferRunning) {
         LedMode = 5;
      }
      else if (TB_Turret.turretArmed) {
         LedMode = 4;
         LedAngle = (int)TB_Turret.turretAngle;
         LedSpeed = Math.min(7,(int)(TB_Turret.targetVector.distance/120*7));
      }
      else if (driveData.driveSpeed != 0 && driveData.rotate == 0) {
         LedMode = 1;
         LedAngle = driveData.driveAngle;
         LedSpeed = Math.min(7,(int)(driveData.driveSpeed/0.5*7));
      }
      else if (driveData.driveSpeed == 0 && driveData.rotate != 0) {
         LedMode = 2;
         LedDirection = driveData.rotate > 0;
      }
      else if (driveData.driveSpeed != 0 && driveData.rotate != 0) {
         LedMode = 3;
         LedDirection = driveData.rotate > 0;
         LedAngle = driveData.driveAngle;
         LedSpeed = Math.min(7,(int)(driveData.driveSpeed/0.5*7));
      }
      else {
         LedMode = 0;
      }
   }

   static void setMode() {
      if (LedMode == LedMode0) {LedDelay=false; return;}
      LedMode0 = LedMode;
      LedDelay = true;
      // mode is 0 through 5 and doesn't need to be calculated
      setLedServo(LedMode);
   }

   static void setDirection() {
      if (LedDirection == LedDirection0) {LedDelay=false; return;}
      LedDirection0 = LedDirection;
      LedDelay = true;
      // true = left/CCW, false = right/CW  // 7 left, 8 right
      setLedServo(LedDirection ? 7 : 8);
   }

   static void setAngle() {
      if (LedAngle == LedAngle0) {LedDelay=false; return;}
      LedAngle0 = LedAngle;
      LedDelay = true;
      // 10 through 25 = 16 positions
      int normAngle = (int)LedAngle;
      if (normAngle < 0) normAngle += 360;
      setLedServo(Math.min((int)(10 + 0.5 + normAngle / LedAngleValue), 25));
   }

   static void setSpeed() {
      if (LedSpeed == LedSpeed0) {LedDelay=false; return;}
      LedSpeed0 = LedSpeed;
      LedDelay = true;
      // 30 through 36 = 7 speeds
      setLedServo(Math.min((int)(30 + 0.5 + LedSpeed), 36));
      setLedServo(30 + LedSpeed);
   }

   static void setLedServo(int sv) {
      servoWFLED.setPosition(sv * LedServoValue);
   }

   static void buildStateMachines() {
      if (!TB_Misc.modeDemo) return;
      StateMachine task;

      smWFLED = new StateMachine("WF LED");
      task = smWFLED;
      task.setAutoRestart(true);
      task.setNoBulkStop(true);  // stay running
      task.addRunOnce(TB_WFLED::setMode);
      task.addWaitFor( ()-> !LedDelay, LedDelayTime );
      task.addRunOnce(TB_WFLED::setDirection);
      task.addWaitFor( ()-> !LedDelay, LedDelayTime );
      task.addRunOnce(TB_WFLED::setAngle);
      task.addWaitFor( ()-> !LedDelay, LedDelayTime );
      task.addRunOnce(TB_WFLED::setSpeed);
      task.addWaitFor( ()-> !LedDelay, LedDelayTime );

   }

}