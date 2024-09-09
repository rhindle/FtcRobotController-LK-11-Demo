package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class DSLed implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public int cols = 8;  //18,16
   public int rows = 8;

   public int attractDelay = 250;
   public long attractTime = 0;
//   int[][] attractMatrix = new int[1][8];

   int[][] messageMatrix = new int[cols][rows];
   int[][] normalMatrix = new int[cols][rows];
   int[][] finalMatrix = new int[cols][rows];

   private static Servo servoBlinkin;
   //private static ServoImplEx servoBlinkin;
   int servoBlinkinSetting, servoBlinkinSettingLast;

   /* Constructor */
   public DSLed(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      if (parts.useNeoMatrix) {
         parts.neo.initialize();
         parts.neo.flipVert=true;
         parts.neo.flipHoriz=true;
         parts.neo.setUpdateLimit(0);
         parts.neo.setPreventTearing(true);
         parts.neo.setDimmingValue(255);
         parts.neo.drawRectangle(0, 7, 0, 7, Color.rgb(10, 10, 0));
         normalMatrix = parts.neo.buildPixelMapFromString("abcd", marquis, Color.rgb(10,10,0), Color.rgb(0,0,0));
      }
      servoBlinkin = parts.robot.servo2B;
      ServoImplEx adjust = parts.opMode.hardwareMap.get(ServoImplEx.class,"servo2B");
      adjust.setPwmRange(new PwmControl.PwmRange(500, 2500));
      setBlinkinPattern(60);
   }

   public void preInit() {
   }

   public void initLoop() {
      if (parts.useNeoMatrix) {
         parts.neo.applyPixelMapToBuffer(normalMatrix, 0, 7, 0, true);
         //parts.neo.applyPixelMapToBuffer(parts.neo.reversePixelMap(normalMatrix), 8, 15, 0, true);
         normalMatrix = parts.neo.shiftPixelMap(normalMatrix, -8, 0, true);
         clearMessage();
         parts.neo.runLoop();
      }
   }

   public void preRun() {
      if (parts.useNeoMatrix) {
         parts.neo.clearMatrix();
         normalMatrix = new int[cols][rows];
         updateGraphic('4', Color.rgb(2, 2, 2));
//         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 15, 0, true);
         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 7, 0, true);
         parts.neo.forceUpdateMatrix();
         parts.neo.setUpdateLimit(2);
//         attractMatrix[0] = chase;
      }
   }

   public void runLoop() {
      if (parts.useNeoMatrix) {
         clearMessage();
//         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 15, 0, true);
         parts.neo.applyPixelMapToBuffer(finalMatrix, 0, 7, 0, true);

         //attract();
         parts.neo.runLoop();
      }
      autoSetBlinkin();
   }

   public void stop() {
   }

   long clearMessageTimer = 0;
   int messageDisplayTime = 1000;

   public void displayMessage (char msgChar, boolean boo) {
      if (boo) displayMessage(msgChar, 2);
      else displayMessage(msgChar, 3);
   }
   public void displayMessage (char msgChar, int color) {
      if (!parts.useNeoMatrix) return;
      clearMessageTimer = System.currentTimeMillis() + messageDisplayTime;
      int msgColor;
      int[][] textMatrix;
      switch (color) {
         case 2:
            msgColor = Color.rgb(0,40,0);
            break;
         case 3:
            msgColor = Color.rgb(40,0,0);
            break;
         case 4:
            msgColor = Color.rgb(0,0,40);
            break;
         case 1:
         default:
            msgColor = Color.rgb(20,20,20);
      }
      textMatrix = parts.neo.buildPixelMapFromString(String.valueOf(msgChar), parts.neo.bigLetters, msgColor);
      messageMatrix = new int[cols][rows];
      messageMatrix = parts.neo.overlayPixelMap(textMatrix, messageMatrix, 2);
      //messageMatrix = parts.neo.overlayPixelMap(textMatrix, messageMatrix, 10);
      finalMatrix = parts.neo.cloneArray(messageMatrix);
   }

   public void updateGraphic (char msgChar, int color) {
      if (!parts.useNeoMatrix) return;
      int[][] textMatrix;
      textMatrix = parts.neo.buildPixelMapFromString(String.valueOf(msgChar), circles, color);
      normalMatrix = parts.neo.overlayPixelMap(textMatrix, normalMatrix,0);
      //normalMatrix = parts.neo.overlayPixelMap(textMatrix, normalMatrix,8);
   }

   public void clearMessage () {
      if (clearMessageTimer == 0) {
         finalMatrix = parts.neo.cloneArray(normalMatrix);
         return;
      }
      if (System.currentTimeMillis() >= clearMessageTimer) {
         clearMessageTimer = 0;
      }
   }

//   public void attract () {
//      if (System.currentTimeMillis() > attractTime) {
//         attractTime = System.currentTimeMillis() + attractDelay;
//         attractMatrix = parts.neo.shiftPixelMap(attractMatrix,0,1,true);
//      }
//      parts.neo.applyPixelMapToBuffer(attractMatrix, 16, 16,0,true);
//   }

   public void setBlinkinPattern(int pattern) {
      if (pattern<1 || pattern>100) return;  //or throw an error
      int pulseWidth = 995 + 10*pattern;                 // convert pattern number to pulse width between 1000-2000 μs)
      double setting = (pulseWidth - 500) / 2000.0;      // covert pulse width to 0-1 servo position (based on 500-2500 μs)
      servoBlinkin.setPosition(setting);
   }

   public void setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
      setBlinkinPattern(pattern.ordinal()+1);            // ordinal starts at 0, so need to add 1
   }

   public void autoSetBlinkin() {
      if (parts.dsShooter.isArmed) servoBlinkinSetting = 21;
      else if (parts.autoDrive.isNavigating) servoBlinkinSetting = 71;
      else if (parts.userDrive.isDriving) servoBlinkinSetting = 51;
      else servoBlinkinSetting = 25; //43;
      if (servoBlinkinSetting != servoBlinkinSettingLast) setBlinkinPattern(servoBlinkinSetting);
      servoBlinkinSettingLast = servoBlinkinSetting;
   }

   public final char[][] marquis = {
           {'a', 17, 128, 0, 0, 1, 128, 0, 34},
           {'b', 34, 0, 128, 1, 0, 0, 128, 17},
           {'c', 68, 0, 1, 128, 0, 0, 1, 136},
           {'d', 136, 1, 0, 0, 128, 1, 0, 68},
           {'e', 238, 1, 129, 129, 128, 1, 129, 221},
           {'f', 221, 129, 1, 128, 129, 129, 1, 238},
           {'g', 187, 129, 128, 1, 129, 129, 128, 119},
           {'h', 119, 128, 129, 129, 1, 128, 129, 187} };

   public final char[][] circles = {
           {'1', 0, 0, 0, 24, 24, 0, 0, 0},
           {'2', 0, 0, 60, 36, 36, 60, 0, 0},
           {'3', 0, 60, 66, 66, 66, 66, 60, 0},
           {'4', 60, 66, 129, 129, 129, 129, 66, 60} };

   public final int[] chase = { 10, 20, 40, 60, 20, 0, 0, 0};
}