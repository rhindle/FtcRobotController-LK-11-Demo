package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class SB_Misc implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
   public PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
   public PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
   public PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
   public PositionTolerance toleranceTransition = new PositionTolerance(4.0,90.0,0);

   /* Constructor */
   public SB_Misc(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
   }

   public void stop() {
   }

}