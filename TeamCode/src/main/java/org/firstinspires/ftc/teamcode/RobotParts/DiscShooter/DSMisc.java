package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class DSMisc implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public static final Position aimPosition            = new Position (12, -4, 0);
   public static final Position tagReadPos             = new Position(-28,-1,0);
   public static final Position autoLaunchPosKnowlton  = new Position(-33, -4, 0);  // must be updated!!
   public static final Position autoLaunchPosDemo      = new Position(-75, -4, 0);  // must be updated!!
   public static Position autoLaunchPos                = new Position(-75, -4, 0);  // must be updated!!

   public PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
   public PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
   public PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
   public PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
   public PositionTolerance toleranceTransition = new PositionTolerance(4.0,180.0,0);

   NavigationTarget test = new NavigationTarget(new Position(-24,0,0), toleranceTransition,1.0,5000,true);

   /* Constructor */
   public DSMisc(Parts parts){
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