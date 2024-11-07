package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

import java.util.Arrays;
import java.util.Deque;
import java.util.ArrayDeque;

import androidx.annotation.NonNull;

public class AutoDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   Status status;
   public DrivePowers drivePowers;
   double speedMaximum = 1;
   double powerMotorMinimum = 0.025;
   double powerMotorMaximum = 1;
   public PIDCoefficients PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
   public PIDCoefficients PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
   PIDCoefficients PIDmovement_calculated = new PIDCoefficients(0,0,0);
   PIDCoefficients PIDrotate_calculated = new PIDCoefficients(0,0,0);
   public boolean onTargetByAccuracy = false;
   public boolean isNavigating = false;
   public boolean isQueueing = false;
   public boolean isHolding = false;
   public boolean isLate = false;
   public boolean abortOnTimeout = true;
   NavigationTarget navTarget;
   Error error, errorLast;
   long timeNavStart, timePIDCurrent, timePIDLast;
   double powerTranslate, powerRotate, navAngle, navAngleLast;
   Deque<NavigationTarget> navQueue = new ArrayDeque<>();

   /* Constructor */
   public AutoDrive(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      drivePowers = new DrivePowers();
      error = new Error();
      errorLast = new Error();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      // only calculate the drive powers if actively navigating or holding
      if (isNavigating || isHolding) {
         status=isNavigating ? Status.DRIVING : Status.HOLDING;  //todo: is this needed?
         autoDrivePower();
      }
      else {
         if (status!=Status.SUCCESS && status!=Status.TIMEOUT && status!=Status.CANCELED && status!=Status.LOST) {
            // todo: not sure LOST belongs, because that implies navigation can resume? See also refuseNavigation().
            status=Status.IDLE;
         }
         dummyTelemetry();
      }
      TelemetryMgr.message(Category.AUTODRIVE, "isNavigating", isNavigating);
      TelemetryMgr.message(Category.AUTODRIVE, "isHolding", isHolding);
      TelemetryMgr.message(Category.AUTODRIVE, "isLate", isLate);
      TelemetryMgr.message(Category.AUTODRIVE, "onTargetByAccuracy", onTargetByAccuracy);
      TelemetryMgr.message(Category.AUTODRIVE, "AutoDrive", status);
      TelemetryMgr.message(Category.AUTODRIVE, "Target", (navTarget==null) ? "(null)" : navTarget.targetPos.toString(2));
   }

   public void stop() {
      cancelNavigation();
   }

   public void eStop() {
      cancelNavigation();
   }

   public void dummyTelemetry() {
      //placeholder telemetry so it won't bounce around
      TelemetryMgr.message(Category.AUTODRIVE, "DeltaX");
      TelemetryMgr.message(Category.AUTODRIVE, "DeltaY");
      TelemetryMgr.message(Category.AUTODRIVE, "NavDistance");
      TelemetryMgr.message(Category.AUTODRIVE, "NavAngle");
      TelemetryMgr.message(Category.AUTODRIVE, "NavRotation");
      TelemetryMgr.message(Category.AUTODRIVE, "NavPowTrans");
      TelemetryMgr.message(Category.AUTODRIVE, "NavPowerRot");
   }

   public void autoDrivePower () {
      /* This is used for both driving to position and holding a position */

//      /* Sync the navigation target with the first target in the queue, if being used */
//      //todo: maybe change this so only updated when the queue is modified?
//      if (isQueueing) {
//         NavigationTarget target = navQueue.peek();
//         if (target!=null) navTarget=target.clone();
//      }

      /* If there is no position, don't drive */
      if (parts.positionMgr.noPosition()) {
         //todo: make sure this doesn't leave motors running [now check the work]
         status=Status.LOST;
         parts.drivetrain.stopDriveMotors();
         dummyTelemetry();
         return;
      }

      /* Check for timeout (only if navigating, and if !abortOnTimeout, keep driving anyway). */
      if (isNavigating && System.currentTimeMillis() - timeNavStart > navTarget.timeLimit) {
         isLate=true;
         status=Status.TIMEOUT;
         if (abortOnTimeout) {
            cancelNavigation();
            status=Status.TIMEOUT;  //todo:remove maybe?
            dummyTelemetry();
            return;
         }
      } else isLate=false;

      /* Check for being on target */
      //todo: Do we want to switch mode from navigating to holding and thus avoid timeouts or whatever?
      //todo: Have navigate that ends in holding vs end idle?
      //todo: If Navigating is interrupted by UserDrive, then holding changes the navTarget and onTargetByAccuracy may not represent what we want
      onTargetByAccuracy = navTarget.inToleranceByTime(parts.positionMgr.robotPosition);
      if (onTargetByAccuracy && isQueueing) {
         navQueue.poll();  //removeFirst();
         if (!navQueue.isEmpty()) {
            onTargetByAccuracy = false;
            NavigationTarget target = navQueue.peek();
            if (target!=null) updateNavTarget(target.clone());
         }
      }
      if (onTargetByAccuracy) {
         status=isNavigating ? Status.SUCCESS : Status.HOLDING;
         isNavigating=false;
         if (!isHolding) parts.drivetrain.stopDriveMotors();
      } else status=Status.DRIVING;

      if (!isNavigating && !isHolding) return;
      if (isNavigating) parts.userDrive.storedHeading=parts.positionMgr.robotPosition.R;   // todo: improve this kludge

      updateError();
      navAngle = Math.toDegrees(Math.atan2(error.y,error.x));  // angle to xy destination (vector when combined with distance)
      timePIDCurrent = System.currentTimeMillis();

      /* Calculate translation PID and power */
      if (Math.abs(navAngle-navAngleLast)>45) PIDmovement_calculated.i = 0;  // test - zero the I if we pass through an inflection
      PIDmovement_calculated.p = PIDmovement.p * error.dist;
      PIDmovement_calculated.i += PIDmovement.i * error.dist * ((timePIDCurrent - timePIDLast) / 1000.0);
      PIDmovement_calculated.i = Math.max(Math.min(PIDmovement_calculated.i,1),-1);
      PIDmovement_calculated.d = PIDmovement.d * (error.dist - errorLast.dist) / ((timePIDCurrent - timePIDLast) / 1000.0);
      if (parts.useDrivetrainEncoders) {
         powerTranslate = PIDmovement_calculated.p;
      } else {
         powerTranslate = PIDmovement_calculated.p +  PIDmovement_calculated.i +  PIDmovement_calculated.d;
      }
      //if (navTarget.noSlow) powerTranslate = 1;  // don't bother with proportional when hitting transitional destinations
      // Don't use proportional when hitting transitional destinations unless still waiting for rotation
      if (navTarget.noSlow && !navTarget.inToleranceExceptRotation(parts.positionMgr.robotPosition)) powerTranslate = 1; ;
      powerTranslate = Math.max(Math.min(powerTranslate, powerMotorMaximum), powerMotorMinimum);

      /* Calculate rotation PID and power */
      PIDrotate_calculated.p = PIDrotate.p * error.rot;
      PIDrotate_calculated.i += PIDrotate.i * error.rot * ((timePIDCurrent - timePIDLast) / 1000.0);
      PIDrotate_calculated.i = Math.max(Math.min(PIDrotate_calculated.i,1),-1);
      PIDrotate_calculated.d = PIDrotate.d * (error.dist - errorLast.rot) / ((timePIDCurrent - timePIDLast) / 1000.0);
      if (parts.useDrivetrainEncoders) {
         powerRotate =  PIDrotate_calculated.p;
      } else {
         powerRotate =  PIDrotate_calculated.p +  PIDrotate_calculated.i +  PIDrotate_calculated.d;
      }
      powerRotate = Math.max(Math.min(Math.abs(powerRotate), powerMotorMaximum), powerMotorMinimum)*Math.signum(powerRotate)*-1;

      /* Save data for next loop */
      timePIDLast = timePIDCurrent;
      errorLast = error.clone();
      navAngleLast = navAngle;

      TelemetryMgr.message(Category.AUTODRIVE, "NavDistance", JavaUtil.formatNumber(error.dist, 2));
      TelemetryMgr.message(Category.AUTODRIVE, "NavAngle", JavaUtil.formatNumber(navAngle, 2));
      TelemetryMgr.message(Category.AUTODRIVE, "NavRotation", JavaUtil.formatNumber(error.rot, 2));
      TelemetryMgr.message(Category.AUTODRIVE, "NavPowTrans", JavaUtil.formatNumber(powerTranslate, 2));
      TelemetryMgr.message(Category.AUTODRIVE, "NavPowerRot", JavaUtil.formatNumber(powerRotate, 2));

      /* Calculate and set the drivetrain powers */
      navAngle -= parts.positionMgr.robotPosition.R;  // need to account for how the robot is oriented
      double autoSpeed = powerTranslate * 1;  // 1 here is maxspeed; could be turned into a variable
      // the following adds the mecanum X, Y, and rotation motion components for each wheel
      drivePowers.v0 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + powerRotate;
      drivePowers.v2 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + powerRotate;
      drivePowers.v1 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - powerRotate;
      drivePowers.v3 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - powerRotate;
      // scale so average motor speed is not more than maxSpeed, but only if maxspeed <> 1
      if (speedMaximum != 1) {
         drivePowers.scaleAverage(speedMaximum);
      }
      // scale to no higher than 1
      drivePowers.scaleMax(1);
      parts.drivetrain.setDrivePowers(drivePowers);
   }

   public void setSpeedMaximum(double speedMaximum) {
      this.speedMaximum = speedMaximum;
   }

   public void resetPID() {
      PIDmovement_calculated.i = 0;
      PIDrotate_calculated.i = 0;
   }

   public void updateError() {
      error.x = navTarget.targetPos.X - parts.positionMgr.robotPosition.X;
      error.y = navTarget.targetPos.Y - parts.positionMgr.robotPosition.Y;
      error.rot = getHeadingError(navTarget.targetPos.R);
      error.dist = Math.sqrt(Math.pow(error.x,2) + Math.pow(error.y,2));
      TelemetryMgr.message(Category.AUTODRIVE, "DeltaX", JavaUtil.formatNumber(error.x, 2));
      TelemetryMgr.message(Category.AUTODRIVE, "DeltaY", JavaUtil.formatNumber(error.y, 2));
   }

   public double getHeadingError(double targetAngle) {
//      if (parts.positionMgr.noPosition()) return 0;
      double robotError;
      robotError = targetAngle - (parts.positionMgr.hasPosition() ? parts.positionMgr.robotPosition.R : parts.positionMgr.headingOnly.R);
      return Functions.normalizeAngle(robotError);
   }

   public void setTargetToCurrentPosition(double withR) {
      if (parts.positionMgr.noPosition()) {
         cancelNavigation();
         status = Status.LOST;
         return;
      }
      navTarget = new NavigationTarget(new Position (
              Math.round(parts.positionMgr.robotPosition.X),
              Math.round(parts.positionMgr.robotPosition.Y),
              Math.round(withR)
      ));
      isNavigating = false;
      isHolding = true;
      resetPID();
   }
   public void setTargetToCurrentPosition() {
      if (parts.positionMgr.noPosition()) {
         cancelNavigation();
         status = Status.LOST;
         return;
      }
      setTargetToCurrentPosition(parts.positionMgr.robotPosition.R);
   }

   public void setNavTarget(NavigationTarget target) {
      setNavTarget(target, true);
   }

   public void setNavTarget(NavigationTarget target, boolean hold) {
      clearNavTargetQueue();         // new target from outside means stop the queue
      updateNavTarget(target, hold);
   }

   private void updateNavTarget(NavigationTarget target) {
      updateNavTarget(target, true);
   }

   private void updateNavTarget(NavigationTarget target, boolean hold) {
      //if (parts.positionMgr.noPosition()) return;  //todo:Is this necessary? Or just rely on the loop's checks
      this.navTarget = target;
      timeNavStart = System.currentTimeMillis();
      isNavigating = true;
      isHolding = hold;
      resetPID();
      onTargetByAccuracy = false;  //todo: does setting this to false cause any problems?  Needed for shooter state machines.
   }

   // included for legacy reasons, but should be phased out
   public void setTarget(Position target) {
      setNavTarget(new NavigationTarget(target));
   }

   public void setTargetByRobotDelta(double X, double Y, double R) {
      //if (parts.positionMgr.noPosition()) return;
      navTarget.targetPos.X = navTarget.targetPos.X + (X * Math.cos(Math.toRadians(navTarget.targetPos.R)) - Y * Math.sin(Math.toRadians(navTarget.targetPos.R)));
      navTarget.targetPos.Y = navTarget.targetPos.Y + (X * Math.sin(Math.toRadians(navTarget.targetPos.R)) + Y * Math.cos(Math.toRadians(navTarget.targetPos.R)));
      navTarget.targetPos.R += R;
      resetPID();
   }

   public void setTargetByFieldDelta(double X, double Y, double R) {
      //if (parts.positionMgr.noPosition()) return;
      navTarget.targetPos.X += X;
      navTarget.targetPos.Y += Y;
      navTarget.targetPos.R += R;
      resetPID();
   }

   public void cancelNavigation() {
      status = Status.CANCELED;
      isNavigating = false;
      isHolding = false;
//      parts.dsAuto.setIsAuto(false);    //lk need to put this back?  20241107
      clearNavTargetQueue();
      if (!parts.userDrive.isDriving) parts.drivetrain.stopDriveMotors(true);
   }

   public void setAutoDrive(boolean boo) {
      if (!boo) isNavigating = false;
      else if (parts.positionMgr.hasPosition()) isNavigating = true;
   }

   public void addNavTargets(NavigationTarget[] targets) {
      boolean update = navQueue.isEmpty();
      navQueue.addAll(Arrays.asList(targets));
      if (update) {
         NavigationTarget target = navQueue.peek();
         if (target!=null) updateNavTarget(target.clone());
      }
      isQueueing = true;
   }

   public void addNavTargetToQueue(NavigationTarget target) {
      if (target!=null) {
         navQueue.addLast(target);
         if (navQueue.size()==1) updateNavTarget(target.clone());
         isQueueing = true;
      }
   }

   public void addNavTargetToQueueStart(NavigationTarget target) {
      if (target!=null) {
         navQueue.addFirst(target);
         updateNavTarget(target.clone());
         isQueueing = true;
      }
   }

   public void clearNavTargetQueue() {
      navQueue.clear();
      isQueueing = false;
      // todo: do we want to stop navigation?
   }

   public void clearFirstNavTargetFromQueue() {
      navQueue.poll();   // returns null if empty
      if (navQueue.isEmpty()) {
         isQueueing = false;
      }
      else {
         NavigationTarget target = navQueue.peek();
         if (target!=null) updateNavTarget(target.clone());
      }
   }

   public Status getStatus() {
      return status;
   }

   public enum Status {
      DRIVING,
      HOLDING,
      IDLE,
      SUCCESS,
      LATE,
      TIMEOUT,
      CANCELED,
      LOST
   }

   public static class Error {

      public double x, y, rot, dist;

      public Error(double x, double y, double rot, double dist) {
         this.x = x;
         this.y = y;
         this.rot = rot;
         this.dist = dist;
      }

      public Error() {
         this.x = 0;
         this.y = 0;
         this.rot = 0;
         this.dist = 0;
      }

      @NonNull
      public Error clone() {
         return new Error(x, y, rot, dist);
      }
   }
}