package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
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
//      isAuto = true;
//      if (!isAutoRunning()) return false;    //exit right away if stopped
//      parts.autoDrive.setNavTarget(navTarget);
//      parts.autoRunLoop();                   // todo: do this or not?
//      return true;
      return driveToTargetsBackground(new NavigationTarget[] {navTarget});
   }

   public boolean driveToTargetsBackground(NavigationTarget[] navTargets) {
//      isAuto = true;
      if (!isAutoRunning()) return false;    //exit right away if stopped
      parts.autoDrive.addNavTargets(navTargets);
      parts.autoRunLoop();                   // todo: do this or not?
      return true;
   }

   public boolean driveToTarget(NavigationTarget navTarget) {
//      isAuto = true;
//      if (!isAutoRunning()) return false;    //exit right away if stopped
//      parts.autoDrive.setNavTarget(navTarget);
//      return waitForDriveComplete();
      return driveToTargets(new NavigationTarget[] {navTarget});
   }

   public boolean driveToTargets(NavigationTarget[] navTargets) {
//      isAuto = true;
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
//      isAuto = true;
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
              new NavigationTarget(new Position(-33, -4, 0), parts.dsMisc.toleranceHigh, 1.0,5000,false),
      });
      waitForDriveComplete();
   }

   public void testAutoMethod() {

      isAuto = true;
      boolean stat;
      stat=driveToTarget( new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceMedium, 1.0,5000,false));
//      if(!stat) return;
      delay(500);
      stat=driveToTarget( new NavigationTarget(new Position(-28,-1,135), parts.dsMisc.toleranceMedium, 1.0,5000,false));
//      if(!stat) return;
      delay(500);
      stat=driveToTarget( new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, 1.0,5000,false));
//      if(!stat) return;
      stat=driveToTarget( new NavigationTarget(new Position(-28,-1,-90), parts.dsMisc.toleranceTransition, 1.0,5000,false));
//      if(!stat) return;
      stat=driveToTarget( new NavigationTarget(new Position(-28,-1,-135), parts.dsMisc.toleranceMedium, 1.0,5000,false));
//      if(!stat) return;
      delay(500);
      stat=driveToTarget( new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceMedium, 1.0,5000,false));
//      if(!stat) return;
      delay(1000);
      double spd = 0.7;
      driveToTargetsBackground(new NavigationTarget[]{
              new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, spd,5000,true),
              new NavigationTarget(new Position(-15, -1, 0), parts.dsMisc.toleranceTransition, spd,5000,true),
              new NavigationTarget(new Position(-12, -13, 0), parts.dsMisc.toleranceTransition, spd,5000,false),
              new NavigationTarget(new Position(-30, -13, 30), parts.dsMisc.toleranceTransition, spd,5000,true),
              new NavigationTarget(new Position(-12, 13, -30), parts.dsMisc.toleranceTransition, spd,5000,true),
              new NavigationTarget(new Position(-24, 13, 0), parts.dsMisc.toleranceTransition, spd,5000,true),
              new NavigationTarget(new Position(-33, -4, 0), parts.dsMisc.toleranceHigh, spd,5000,false) });
      waitForDriveComplete();
   }

}