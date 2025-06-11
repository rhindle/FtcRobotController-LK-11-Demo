package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.ArcPath;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class DSAuto implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public static boolean isAuto = false;

   /* Constructor */
   public DSAuto(Parts parts){
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

   public void eStop() {
      isAuto = false;
   }

   public boolean driveToTargetBackground(NavigationTarget navTarget) {
      return driveToTargetsBackground(new NavigationTarget[] {navTarget});
   }

   public boolean driveToTargetsBackground(NavigationTarget... navTargets) {
      if (!isAutoRunning()) return false;    //exit right away if stopped
      parts.autoDrive.addNavTargets(navTargets);
      parts.autoRunLoop();                   // todo: do this or not?
      return true;
   }

   public boolean driveToTarget(NavigationTarget navTarget) {
      return driveToTargets(new NavigationTarget[] {navTarget});
   }

   public boolean driveToTargets(NavigationTarget... navTargets) {
      if (!isAutoRunning()) return false;    //exit right away if stopped
      parts.autoDrive.addNavTargets(navTargets);
      return waitForDriveComplete();
   }

   public boolean waitForDriveComplete() {
      while (isAutoRunning()) {
         parts.autoRunLoop();
         switch (parts.autoDrive.getStatus()) {
            case SUCCESS:         // we're done, successfully
               return true;
            case DRIVING:         // we're still in progress
               break;
            case HOLDING:
            case IDLE:
               return true;        // todo: figure this out for sure, combine with success??
            case LATE:
            case TIMEOUT:
            case CANCELED:
            case LOST:
            default:              // we're done, unsuccessfully
               parts.autoDrive.cancelNavigation();
               return false;
         }
      }
      return false;               // auto or opmode stopped
   }

   public void delay(long ms) {
      delay(ms,false);
   }
   public void delay(long ms, boolean blocking) {
      if (blocking) {
         parts.opMode.sleep(ms);
      } else {
         ms += System.currentTimeMillis();
         while (isAutoRunning() && ms > System.currentTimeMillis()) parts.autoRunLoop();
      }
   }

   public boolean isAutoRunning() {
      return (parts.opMode.opModeIsActive() && isAuto);
   }

   public void setIsAuto(boolean boo){
      isAuto = boo;
   }

   public void testAutoMethod0() {
      driveToTargetsBackground(new NavigationTarget[]{
              new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-15, -1, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-12, -13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,false),
              new NavigationTarget(new Position(-30, -13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-12, 13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-24, 13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-33, -4, 0), parts.dsMisc.toleranceHigh, 1.0,5000,false) });
      waitForDriveComplete();
   }

   public void testAutoMethod0v() {
      driveToTargetsBackground(
              new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-15, -1, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-12, -13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,false),
              new NavigationTarget(new Position(-30, -13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-12, 13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-24, 13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
              new NavigationTarget(new Position(-33, -4, 0), parts.dsMisc.toleranceHigh, 1.0,5000,false) );
      waitForDriveComplete();
   }


   public void testAutoMethod() {
      isAuto = true;
      boolean result;
      int timeLimit = 5000;
      double speed = 1.0;
      // warning: don't use noSlow=true for tight tolerances or when the robot is simply rotating (the results are terrible jumpy movement)
      result = driveToTarget( new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceMedium, speed, timeLimit));
      if (!result) return;  // could branch or something
      delay(500);
      driveToTarget( new NavigationTarget(new Position(-28,-1,135), parts.dsMisc.toleranceMedium, speed, timeLimit));
      delay(500);
      driveToTarget( new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-28,-1,-90), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-28,-1,-135), parts.dsMisc.toleranceMedium, speed, timeLimit));
      delay(500);
      driveToTarget( new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceMedium, speed, timeLimit));
      delay(1000);
      speed = 0.7;
      driveToTargetsBackground(new NavigationTarget[]{
              new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, speed, timeLimit,true),
              new NavigationTarget(new Position(-15, -1, 0), parts.dsMisc.toleranceTransition, speed, timeLimit,true),
              new NavigationTarget(new Position(-12, -13, 0), parts.dsMisc.toleranceTransition, speed, timeLimit,false),
              new NavigationTarget(new Position(-30, -13, 30), parts.dsMisc.toleranceTransition, speed, timeLimit,true),
              new NavigationTarget(new Position(-12, 13, -30), parts.dsMisc.toleranceTransition, speed, timeLimit,true),
              new NavigationTarget(new Position(-24, 13, 0), parts.dsMisc.toleranceTransition, speed, timeLimit,true),
              new NavigationTarget(new Position(-33, -4, 0), parts.dsMisc.toleranceHigh, speed, timeLimit,false) });
      waitForDriveComplete();
   }

   public void testAutoMethod2() {
      isAuto = true;
      boolean result;
      int timeLimit = 5000;
      double speed = 1.0;
//      driveToTarget( new NavigationTarget(new Position(-72,-24,0), parts.dsMisc.toleranceMedium, speed, timeLimit));
//      driveToTarget( new NavigationTarget(new Position(-24,-24,90), parts.dsMisc.toleranceMedium, speed, timeLimit));
//      driveToTarget( new NavigationTarget(new Position(-24,24,180), parts.dsMisc.toleranceMedium, speed, timeLimit));
//      driveToTarget( new NavigationTarget(new Position(-72,24,-90), parts.dsMisc.toleranceMedium, speed, timeLimit));

//      driveToTarget( new NavigationTarget(new Position(-72,-24,0), parts.dsMisc.toleranceMedium, speed, timeLimit));
//      driveToTarget( new NavigationTarget(new Position(-24,-24,90), parts.dsMisc.toleranceMedium, speed, timeLimit));
//      driveToTarget( new NavigationTarget(new Position(-24,24,180), parts.dsMisc.toleranceMedium, speed, timeLimit));
//      driveToTarget( new NavigationTarget(new Position(-72,24,-90), parts.dsMisc.toleranceMedium, speed, timeLimit));

      driveToTarget( new NavigationTarget(new Position(-72,0,-90), parts.dsMisc.toleranceMedium, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-69,-12,-60), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-60,-21,-30), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-48,-24,0), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-36,-21,30), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-27,-12,60), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-24,0,90), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-27,12,120), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-36,21,150), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-48,24,180), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-60,21,-150), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-69,12,-120), parts.dsMisc.toleranceTransition, speed, timeLimit));
      driveToTarget( new NavigationTarget(new Position(-72,0,-90), parts.dsMisc.toleranceMedium, speed, timeLimit));
   }

   public void testAutoMethod3() {
      isAuto = true;
      boolean result;
      int timeLimit = 5000;
      double speed = 1.0;
      Position posStart = new Position(-72,0,-90);
      Position posOpp   = new Position(-24,0,90);
      Position posCenter = new Position(-48,0,0);

      // Make a circle driving forward
      driveToTarget( new NavigationTarget(posStart, parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(2000);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.calculateArcPathWithDepth(posStart, posOpp, 1, 1, 11),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              speed, timeLimit, true ));
      waitForDriveComplete();
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.calculateArcPathWithDepth(posOpp, posStart, 1, 1, 11),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false ));
      waitForDriveComplete();

      // Make a circle driving backward
      driveToTarget( new NavigationTarget(posStart.withR(90), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingRelative(ArcPath.calculateArcPathWithDepth(posStart, posOpp, 1, 1, 11), 180),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              speed, timeLimit, true ));
      waitForDriveComplete();
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingRelative(ArcPath.calculateArcPathWithDepth(posOpp, posStart, 1, 1, 11), 180),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false ));
      waitForDriveComplete();

      // Make a circle aiming inward
      driveToTarget( new NavigationTarget(posStart.withR(0), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingRelative(ArcPath.calculateArcPathWithDepth(posStart, posOpp, 1, 1, 11), 90),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              speed, timeLimit, true ));
      waitForDriveComplete();
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingRelative(ArcPath.calculateArcPathWithDepth(posOpp, posStart, 1, 1, 11), 90),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false ));
      waitForDriveComplete();

      // Make a circle aiming right (-90)
      driveToTarget( new NavigationTarget(posStart.withR(-90), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingConstant(ArcPath.calculateArcPathWithDepth(posStart, posOpp, 1, 1, 11), -90),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              speed, timeLimit, true ));
      waitForDriveComplete();
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingConstant(ArcPath.calculateArcPathWithDepth(posOpp, posStart, 1, 1, 11), -90),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false ));
      waitForDriveComplete();

      // Make a circle aiming at the target (DSMisc.aimPosition)
      driveToTarget( new NavigationTarget(posStart.withR(0), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingTarget(ArcPath.calculateArcPathWithDepth(posStart, posOpp, 1, 1, 11), DSMisc.aimPosition),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              speed, timeLimit, true ));
      waitForDriveComplete();
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingTarget(ArcPath.calculateArcPathWithDepth(posOpp, posStart, 1, 1, 11), DSMisc.aimPosition),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false ));
      waitForDriveComplete();

      // Make a circle while smoothly changing heading
      driveToTarget( new NavigationTarget(posStart.withR(90), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingStartEnd(ArcPath.calculateArcPathWithDepth(posStart, posOpp, 1, 1, 11), 90, -45),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              speed, timeLimit, true ));
      waitForDriveComplete();
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.adjustArcPathHeadingStartEnd(ArcPath.calculateArcPathWithDepth(posOpp, posStart, 1, 1, 11), -45, 90),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false ));
      waitForDriveComplete();

      // Drive back to "center"
      driveToTarget( new NavigationTarget(posCenter, parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
   }

   public void testAutoMethod4() {
      isAuto = true;
      boolean result;
      int timeLimit = 5000;
      double speed = 1.0;
      Position posStart = new Position(-72,0,-90);
      Position posOpp   = new Position(-24,0,90);
      Position posCenter = new Position(-48,0,0);

      // Make splines
      Position posRR = new Position(posStart.X, -24,0);
      Position posFL = new Position(posOpp.X,24,-90);
      // spline 1
      driveToTarget( new NavigationTarget(posRR, parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(2000);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.calculateSplineApprox(posRR, posFL, null, 1,1, 11),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false));
      waitForDriveComplete();
      delay(1000);
      // spline 2
      driveToTarget( new NavigationTarget(posFL, parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(2000);
      driveToTargetsBackground(ArcPath.buildNavTargetArray(
              ArcPath.calculateSplineApprox(posFL, posRR, null, 1,-1, 11),
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed, timeLimit, false));
      waitForDriveComplete();
      delay(1000);

      // Drive back to "center"
      driveToTarget( new NavigationTarget(posCenter, parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);

      // Make a circle driving forward
      driveToTarget( new NavigationTarget(posStart, parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(2000);
      driveToTargetsBackground(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.FORWARD));
      waitForDriveComplete();

      // Make a circle driving backward
      driveToTarget( new NavigationTarget(posStart.withR(90), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.BACKWARD));
      waitForDriveComplete();

      // Make a circle aiming inward
      driveToTarget( new NavigationTarget(posStart.withR(0), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.INWARD));
      waitForDriveComplete();

      // Make a circle aiming right (-90)
      driveToTarget( new NavigationTarget(posStart.withR(-90), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.RIGHT));
      waitForDriveComplete();

      // Make a circle aiming at the target (DSMisc.aimPosition)
      driveToTarget( new NavigationTarget(posStart.withR(0), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.TARGET));
      waitForDriveComplete();

      // Make a circle while smoothly changing heading
      driveToTarget( new NavigationTarget(posStart.withR(90), parts.dsMisc.toleranceMedium, speed, timeLimit, false));
      delay(500);
      driveToTargetsBackground(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.SMOOTHCHANGE));
      waitForDriveComplete();

   }

   public NavigationTarget[] generateNavCircle (Position posStart, Position posMid, double speed, int timeLimit, circleVar var){
      Position[] arc1 = ArcPath.calculateArcPathWithDepth(posStart, posMid, 1, 1, 11);
      Position[] arc2 = ArcPath.calculateArcPathWithDepth(posMid, posStart, 1, 1, 11);
      Position[] circle = ArcPath.combinePaths(arc1, arc2, true);
      switch (var) {
         case FORWARD:
            break;
         case BACKWARD:
            circle = ArcPath.adjustArcPathHeadingRelative(circle, 180);
            break;
         case INWARD:
            circle = ArcPath.adjustArcPathHeadingRelative(circle, 90);
            break;
         case RIGHT:
            circle = ArcPath.adjustArcPathHeadingConstant(circle, -90);
            break;
         case TARGET:
            circle = ArcPath.adjustArcPathHeadingTarget(circle, DSMisc.aimPosition);
            break;
         case SMOOTHCHANGE:
            circle = ArcPath.combinePaths(
                    ArcPath.adjustArcPathHeadingStartEnd(arc1, 90, -45),
                    ArcPath.adjustArcPathHeadingStartEnd(arc2, -45, 90),
                    true);
            break;
         default: break;
      }
      return ArcPath.buildNavTargetArray(
              circle,
              parts.dsMisc.toleranceTransition,
              parts.dsMisc.toleranceMedium,
              speed,
              timeLimit,
              false);
   }

   public enum circleVar {
      FORWARD,
      BACKWARD,
      INWARD,
      RIGHT,
      TARGET,
      SMOOTHCHANGE;
   }
}