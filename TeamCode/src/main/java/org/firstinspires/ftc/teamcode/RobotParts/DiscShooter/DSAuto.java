package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
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
      isAuto = true;
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
      isAuto = true;
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
      isAuto = true;
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

}