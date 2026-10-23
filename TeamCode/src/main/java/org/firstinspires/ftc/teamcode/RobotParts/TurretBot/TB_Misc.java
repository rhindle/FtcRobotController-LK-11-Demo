package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class TB_Misc  {

   /* Public OpMode members. */
   public static Parts parts;

   public static final Position targetRed              = new Position(-70.5, 68, 0.0); //y 70.5
   public static final Position targetBlue             = new Position(-70.5, -70.5, 0.0);

   public static double speedSlow = 0.25;
   public static double speedNormal = 0.5;
   public static double speedFast = 1.0;
   public static float speedFactor = (float)speedNormal;  // this is float on purpose so drivedata overload is correct!

   public static final Position fieldStartPositionBlueNear = new Position(-40, -56, 180);
   public static final Position fieldStartPositionBlueFar  = new Position(63.5, -16, 180);  // todo: update this
   public static Position fieldStartPositionBlue = fieldStartPositionBlueNear.clone();
   public static Position fieldStartPosition = fieldStartPositionBlue.clone();

   public static boolean modeDemo = false;
   public static double demoTeamSpeedFast = 1.0;
   public static double demoTeamSpeedSlow = 0.5;
   public static double demoGuestSpeed = 0.3;

   static boolean modeTeleOp = true;
   static boolean allianceBlue = true;
   static boolean autoNear = true;
   public static Position targetCurrent = targetBlue;
   public static boolean servosInit = false;
   public static boolean noPosition = true;
   public static boolean controlsFullSpeed = false;
//   public static boolean forza = false;
   public static Position currentPosition = new Position();

   //============ demo LED additions ============

   public static int LEDmode = 0;
   public static int LEDmode0 = LEDmode;
   //0 idle, 1 driving, 2 rotating, 3 driving & rotating, 4 aiming, 5 shooting
   public static boolean LEDdirection = true;
   public static boolean LEDdirection0 = LEDdirection;
   // true = left/CCW, false = right/CW
   // 7 left, 8 right
   public static int LEDangle = 0;
   public static int LEDangle0 = LEDangle;
   // 10 through 25 = 16 positions
   public static int LEDspeed = 0;
   public static int LEDspeed0 = LEDspeed;
   // 30 through 36 = 7 speeds
   public static double LEDangleValue = 360.0 / 16;

   //============================================

   public static PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
   public static PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
   public static PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
   public static PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
   public static PositionTolerance toleranceTransition = new PositionTolerance(4.0,90.0,0);

   public static void setup(Parts p) {
      parts = p;
   }

   public static void setTeleOp() { modeTeleOp = true; }
   public static void setAuto() { modeTeleOp = false; }
   public static boolean isTeleOp() { return modeTeleOp; }
   public static boolean isAuto() { return !modeTeleOp; }

   public static void setAllianceBlue() {
      allianceBlue = true; targetCurrent=targetBlue;
      fieldStartPosition = redOrBlue(fieldStartPositionBlue);
   }
   public static void setAllianceRed() {
      allianceBlue = false; targetCurrent=targetRed;
      fieldStartPosition = redOrBlue(fieldStartPositionBlue);

   }
   public static void setAutoNear() {
      autoNear = true;
      fieldStartPositionBlue = fieldStartPositionBlueNear.clone();
      fieldStartPosition = redOrBlue(fieldStartPositionBlueNear);
   }
   public static void setAutoFar() {
      autoNear = false;
      fieldStartPositionBlue = fieldStartPositionBlueFar.clone();
      fieldStartPosition = redOrBlue(fieldStartPositionBlueFar);
   }

   public static boolean isAllianceBlue() { return allianceBlue; }
   public static boolean isAllianceRed() { return !allianceBlue; }
   public static boolean isAutoNear() { return autoNear; }

   public static Position redOrBlue (double X, double Y, double R) {
      if (TB_Misc.isAllianceRed()) {
         return new Position(X, -Y, -R);
      }
      else {
         return new Position(X, Y, R);
      }
   }
   public static Position redOrBlue (Position pos) {
      if (TB_Misc.isAllianceRed()) {
         return new Position(pos.X, -pos.Y, -pos.R);
      }
      else {
         return pos.clone();
      }
   }

   public static void fullSpeed(boolean goFast) {
      controlsFullSpeed = goFast;
      if (controlsFullSpeed) speedFactor = (float)speedFast;
      else speedFactor = (float)speedNormal;
      parts.userDrive.setSpeedMaximum(speedFactor);
   }

   public enum rgbIndicatorColor {
      Off (0.0),
      Red (0.279),
      Orange (0.333),
      Yellow (0.388),
      Sage (0.444),
      Green (0.500),
      Azure (0.555),
      Blue (0.611),
      Indigo (0.666),
      Violet (0.715), //(0.722),
      White (1.0);

      final double color;

      rgbIndicatorColor(double color) {
         this.color = color;
      }
   }
}