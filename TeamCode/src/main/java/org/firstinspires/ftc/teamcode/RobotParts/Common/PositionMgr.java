package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class PositionMgr implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;
   public Position robotPosition;
   public Position odoPosition;
   public Position slamraPosition;
   public Position tagPosition;
   public Position pinpointPosition;
   public Position imuHeading;
   public Position headingOnly;
   public Position imusHeadingOnly;
   public Position slamraHeading;
   public Position pinpointHeading;

   public PosSource[] priorityList = {PosSource.PINPOINT, PosSource.ODO, PosSource.SLAMRA, PosSource.TAG};
   public PosSource[] headingOnlyPriority = {PosSource.PINPOINT_R, PosSource.IMU, PosSource.SLAMRA_R, PosSource.ODO, PosSource.TAG};
   public PosSource[] imusOnlyPriority = {PosSource.PINPOINT_R, PosSource.IMU, PosSource.SLAMRA_R};
   public Boolean prioritizeSlamraRforODO = false;      // use Slamra R instead of IMU for ODO
   public Boolean prioritizeIMUforSLAMRA = false;       // use IMU instead of Slarma R for SLAMRA
   public PosSource posSource;

   /* Constructor */
   public PositionMgr(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
//      if (fieldStartPosition!=null) {
//         if (parts.useODO) parts.odometry.odoFieldStart=fieldStartPosition;
//         if (parts.useSlamra) parts.slamra.slamraFieldStart=fieldStartPosition;
//      }
   }

   public void preInit() {
   }

   public void initLoop() {
      runLoop();
   }

   public void preRun() {
   }

   public void runLoop() {
      if (parts.useODO) {
         odoPosition = parts.odometry.isOdoPositionGood() ? parts.odometry.odoRobotPosition : null;
      }
      if (parts.usePinpoint) {
         pinpointPosition = parts.pinpoint.pinpointRobotPosition;   // could be null
      }
      if (parts.useSlamra) {
         slamraPosition = parts.slamra.isSlamraPositionGood() ? parts.slamra.slamraRobotPosition : null;
         slamraHeading = parts.slamra.slamraRobotPosition;
      }
      if (parts.useAprilTag) {
         tagPosition = parts.dsApriltag.tagRobotPosition;
      }
      if (parts.useIMU) {
         imuHeading = parts.imuMgr.returnImuRobotHeadingAsPosition();
      }
      imusHeadingOnly = headingOnlyUpdate(imusOnlyPriority);
      headingOnly = headingOnlyUpdate(headingOnlyPriority);
      robotPosition = normalUpdate();

      TelemetryMgr.message(Category.POSITION, "odo__", (odoPosition==null) ? "(null)" : odoPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "pinpt", (pinpointPosition==null) ? "(null)" : pinpointPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "slmra", (slamraPosition==null) ? "(null)" : slamraPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "tag__", (tagPosition==null) ? "(null)" : tagPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "imu__", (imuHeading==null) ? "(null)" : imuHeading.toString(2));
      TelemetryMgr.message(Category.POSITION, "final", (robotPosition==null) ? "(null)" : robotPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "imus_", (imusHeadingOnly==null) ? "(null)" : imusHeadingOnly.toString(2));
      TelemetryMgr.message(Category.POSITION, "head_", (headingOnly==null) ? "(null)" : headingOnly.toString(2));
   }

   public void stop() {
   }

   public Boolean hasPosition () {
      return (robotPosition!=null);
   }
   public Boolean noPosition () {
      return (robotPosition==null);
   }
   public Boolean hasHeading() {
      return (headingOnly!=null);
   }
   public Boolean hasImusHeading() {
      return (imusHeadingOnly!=null);
   }

   Position normalUpdate() {
      posSource = returnPrioritySource(priorityList);
      switch (posSource) {
         case ODO:
            if (prioritizeSlamraRforODO && slamraPosition!=null) {
               return odoPosition.withR(slamraPosition.R);
            }
            return odoPosition;
         case PINPOINT:
            return pinpointPosition;
         case SLAMRA:
            //todo: finish/check this
            if (prioritizeIMUforSLAMRA && imuHeading!=null) {
               return slamraPosition.withR(imuHeading.R);
            }
            return slamraPosition;
         case TAG:
            return tagPosition;
         case NONE:
         default:
            return null;
      }
   }

   Position headingOnlyUpdate(PosSource[] list) {
      posSource = returnPrioritySource(list);
      switch (posSource) {
         case IMU:
            return new Position().withR(imuHeading.R);
         case PINPOINT:
         case PINPOINT_R:
            return new Position().withR(pinpointPosition.R);
         case ODO:
            return new Position().withR(odoPosition.R);
         case SLAMRA:
         case SLAMRA_R:
            return new Position().withR(slamraHeading.R);
         case TAG:
            return new Position().withR(tagPosition.R);
         case NONE:
         default:
            return null;
      }
   }

   PosSource returnPrioritySource(PosSource[] list) {
      for (PosSource source : list) {
         switch (source) {
            case ODO:
               if (odoPosition!=null) return PosSource.ODO;
               break;
            case PINPOINT:
               if (pinpointPosition!=null) return PosSource.PINPOINT;
               break;
            case SLAMRA:
               if (slamraPosition!=null) return PosSource.SLAMRA;
               break;
            case TAG:
               if (tagPosition!=null) return PosSource.TAG;
               break;
            case IMU:
               if (imuHeading!=null) return PosSource.IMU;   // only valid for heading only
               break;
            case SLAMRA_R:
               if (slamraHeading!=null) return PosSource.SLAMRA_R;   // only valid for heading only
               break;
            default:
         }
      }
      return PosSource.NONE;
   }

   public enum PosSource {
      NONE,
      PINPOINT,
      PINPOINT_R,   // todo: needed?
      ODO,
      SLAMRA,
      TAG,
      ENCODER,
      IMU,
      SLAMRA_R
   }
}