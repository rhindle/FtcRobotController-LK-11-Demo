package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class TB_Misc  {

   /* Public OpMode members. */
   public static Parts parts;

   public static PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
   public static PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
   public static PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
   public static PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
   public static PositionTolerance toleranceTransition = new PositionTolerance(4.0,90.0,0);

   public static void setup(Parts p) {
      parts = p;
   }
}