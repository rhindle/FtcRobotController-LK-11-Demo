package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class TB_Misc  {

   /* Public OpMode members. */
   public static Parts parts;

   public static final Position targetRed              = new Position(-70.5, 70.5, 0.0);
   public static final Position targetBlue             = new Position(-70.5, -70.5, 0.0);

   static boolean modeTeleOp = true;
   static boolean allianceBlue = true;
   public static Position targetCurrent = targetBlue;
   public static boolean servosInit = false;
   public static boolean noPosition = true;
//   public static boolean forza = false;
   public static Position currentPosition = new Position();

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

   public static void setAllianceBlue() { allianceBlue = true; targetCurrent=targetBlue; }
   public static void setAllianceRed() { allianceBlue = false; targetCurrent=targetRed; }
   public static boolean isAllianceBlue() { return allianceBlue; }
   public static boolean isAllianceRed() { return !allianceBlue; }


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