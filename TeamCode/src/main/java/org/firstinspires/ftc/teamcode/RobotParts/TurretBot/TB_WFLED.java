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
   public final static int LedDelayTime = 40;  // Long enough for two read attempts? // was 60

   static long idlePreTimer = 0;
   static long idleResetTimer = 0;

   public static DriveData driveData = new DriveData();

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize(){
      buildStateMachines();
      servoWFLED = new ServoSSR(parts.robotV2.getServoByName("servo1"));
      servoWFLED.setPosition(0);
   }

   public static void preRun() {
      if (!TB_Misc.modeDemo) return;
      servoWFLED.setPosition(0);
      smWFLED.restart();
   }

   public static void runLoop() {
      if (!TB_Misc.modeDemo) return;
      if (TB_Intake.transferRunning) {
         LedMode = 5;
         resetIdlePreTimer();
      }
      else if (TB_Turret.turretArmed && !TB_Turret.turretPaused) {
         LedMode = 4;
         LedAngle = (int)TB_Turret.turretAngle;
         LedSpeed = Math.min(7,(int)(TB_Turret.targetVector.distance/120*7));
         resetIdlePreTimer();
         // Because setting up the robot for aim can be time consuming, the LED idle is likely
         // to activate. Sending a "9" signal resets the idle timer. Next iteration, back to "4".
         // If the program stops, this keepalive signal stops, so timeout can happen normally.
         if (System.currentTimeMillis() > idleResetTimer) {
            LedMode = 9;   // special value for resetting the idle timer
            idleResetTimer = System.currentTimeMillis() + 5000;
         }
      }
      else if (driveData.driveSpeed != 0 && driveData.rotate == 0) {
         LedMode = 1;
         LedAngle = driveData.driveAngle;
         LedSpeed = Math.min(7,(int)(driveData.driveSpeed/0.5*7));
         resetIdlePreTimer();
      }
      else if (driveData.driveSpeed == 0 && driveData.rotate != 0) {
         LedMode = 2;
         LedDirection = driveData.rotate > 0;
         resetIdlePreTimer();
      }
      else if (driveData.driveSpeed != 0 && driveData.rotate != 0) {
         LedMode = 3;
         LedDirection = driveData.rotate > 0;
         LedAngle = driveData.driveAngle;
         LedSpeed = Math.min(7,(int)(driveData.driveSpeed/0.5*7));
         resetIdlePreTimer();
      }
      else {
         if (System.currentTimeMillis() > idlePreTimer) LedMode = 0;
      }
   }

   static void resetIdlePreTimer() {
      idlePreTimer = System.currentTimeMillis() + 2000;
   }

   static void setMode() {
      if (LedMode == LedMode0) {LedDelay=false; return;}
      LedMode0 = LedMode;
      LedDelay = true;
      // mode is 0 through 5 and doesn't need to be calculated [also 9 for reset idle]
      setLedServo(LedMode);
   }

   static void setDirection() {
      if (LedDirection == LedDirection0) {LedDelay=false; return;}
      LedDirection0 = LedDirection;
      LedDelay = true;
      // true = left/CCW, false = right/CW  // 7 left, 8 right
      //setLedServo(LedDirection ? 7 : 8);
      // This was exactly backwards and I don't know why, so here is a hack fix
      setLedServo(LedDirection ? 8 : 7);
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