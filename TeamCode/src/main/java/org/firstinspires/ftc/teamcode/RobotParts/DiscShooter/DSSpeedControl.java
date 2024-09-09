package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class DSSpeedControl implements PartsInterface {

   /* The purpose of this class is to limit the speed of the robot as it gets near the perimeter of the field (to minimize slamming)
      This is accomplished by setting two square "fences" in the interior of the field. When the robot is outside the "fence",
      it is limited to a slower speed unless its direction is back toward the "fence".
    */

   /* Public OpMode members. */
   public Parts parts;

   public DrivePowers drivePowers;
   public double speedMaximum;
   public double speedMaximumNoPosition = 0.5;
   public Fence fenceInner = new Fence(0.5, new Position(-30, 17), new Position(-64, -17));
   public Fence fenceOuter = new Fence(0.25, new Position(-18, 29), new Position(-76, -29));

   /* Constructor */
   public DSSpeedControl(Parts parts){
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
      if (!parts.userDrive.isDriving) {
         // insert dummy telemetry so it doesn't bounce around
         TelemetryMgr.message(Category.SPEED,"SpdCtrl - Not Driving");
         TelemetryMgr.message(Category.SPEED,"Fence : Not applicable");
         return;
      }
      // if rotating only, don't use the fences  //todo:consider this
      if (parts.userDrive.driveSpeed==0) {
         // insert dummy telemetry so it doesn't bounce around
         TelemetryMgr.message(Category.SPEED,"SpdCtrl - Rotating only");
         TelemetryMgr.message(Category.SPEED,"Fence : Not applicable");
         return;
      }
      // if UserDrive is driving robot, adjust the powers as necessary and send them to drivetrain again
      drivePowers=parts.userDrive.drivePowers.clone();
      //speedMaximum=parts.userDrive.speedMaximum;
      speedMaximum = 1;
      applySpeedFences();
      if (speedMaximum != 1) {
         drivePowers.scaleAverage(speedMaximum);
      }
      parts.drivetrain.setDrivePowers(drivePowers);
      TelemetryMgr.message(Category.SPEED,"Fence", drivePowers.toString(2));
   }

   public void stop() {
   }

   public void applySpeedFences() {
      // note: speed is already scaled to speedMaximum in UserDrive
      if (parts.positionMgr.noPosition()) {
         speedMaximum = speedMaximumNoPosition;
         TelemetryMgr.message(Category.SPEED,"SpdCtrl - No Position", JavaUtil.formatNumber(speedMaximum, 2));
      }
      else {
//         speedMaximum = parts.userDrive.speedMaximumWithPosition * checkFences(driveAngle, useFieldCentricDrive);  //todo: verify this
         // this will be 1 or a lesser value from the fences above
         speedMaximum = checkFences(parts.userDrive.driveAngle, parts.userDrive.useFieldCentricDrive);
      }
   }

   public double checkFences (double direction) {
      return checkFences(parts.positionMgr.robotPosition, direction);
   }

   public double checkFences (double direction, boolean fieldCentric) {
      return checkFences(parts.positionMgr.robotPosition, direction, fieldCentric);
   }

   public double checkFences (Position currentPosition, double direction) {
      // assumes that the controls (and thus direction) are robot centric (vs. field centric)
      // which means the direction has to be manipulated to a field centric direction
      direction += currentPosition.R;
      double outerSpeed = fencedSpeed(currentPosition, direction, fenceOuter);
      double innerSpeed = fencedSpeed(currentPosition, direction, fenceInner);
      if (outerSpeed != 1) {
         TelemetryMgr.message(Category.SPEED,"SpdCtrl - Outer Fence", JavaUtil.formatNumber(outerSpeed, 2));
         return outerSpeed;
      }
      if (innerSpeed != 1) {
         TelemetryMgr.message(Category.SPEED,"SpdCtrl - Inner Fence", JavaUtil.formatNumber(innerSpeed, 2));
         return innerSpeed;
      }
      TelemetryMgr.message(Category.SPEED,"SpdCtrl - No Fence XX", JavaUtil.formatNumber(1,2));
      return 1;
   }

   public double checkFences (Position currentPosition, double direction, boolean fieldCentric) {
      if (fieldCentric) {
         // if the controls are field centric, direction is based on the field (i.e., robot angle is 0)
         return checkFences(currentPosition.withR(0), direction);
      }
      else {
         return checkFences(currentPosition, direction);
      }
   }

   public double fencedSpeed (Position currentPosition, double direction, Fence fence) {
      boolean outside=false;
      direction = Functions.normalizeAngle(direction);
      //top, bottom, left, right
      if (currentPosition.X > fence.xMax && (direction > -95 && direction < 95)) outside=true;
      if (currentPosition.X < fence.xMin && (direction < -85 || direction > 85)) outside=true;
      if (currentPosition.Y > fence.yMax && (direction > -5 || direction < -175)) outside=true;
      if (currentPosition.Y < fence.yMin && (direction <  5 || direction >  175)) outside=true;

      return outside ? fence.outsideSpeed : fence.insideSpeed;
   }

   public static class Fence {

      public double insideSpeed, outsideSpeed;
      public double xMax, xMin, yMax, yMin;

      public Fence(double speed, double xMax, double xMin, double yMax, double yMin) {
         this.outsideSpeed = speed;
         this.insideSpeed = 1;
         this.xMax = xMax;
         this.xMin = xMin;
         this.yMax = yMax;
         this.yMin = yMin;
      }

      public Fence(double speed, Position topLeft, Position bottomRight) {
         this.outsideSpeed = speed;
         this.insideSpeed = 1;
         this.xMax = topLeft.X;
         this.xMin = bottomRight.X;
         this.yMax = topLeft.Y;
         this.yMin = bottomRight.Y;
      }

      // If this gets used, need to change the logic in fencedSpeed()
      public Fence(double outsideSpeed, double insideSpeed, Position topLeft, Position bottomRight) {
         this.outsideSpeed = outsideSpeed;
         this.insideSpeed = insideSpeed;
         this.xMax = topLeft.X;
         this.xMin = bottomRight.X;
         this.yMax = topLeft.Y;
         this.yMin = bottomRight.Y;
      }
   }
}