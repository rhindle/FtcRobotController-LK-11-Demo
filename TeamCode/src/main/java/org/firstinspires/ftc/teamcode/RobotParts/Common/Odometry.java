package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

import androidx.annotation.NonNull;

public class Odometry implements PartsInterface {

   public Parts parts;

   public EncoderSetting odoEncXL, odoEncXR, odoEncY;
   public OdoData odoData, odoData0;
   public boolean useFusedHeading = true;
   public boolean use3encoders = true;

//   public Position odoRobotOffset = new Position (2.25,0,0);          // map odo to robot (so it holds turn position better)
   public Position odoRobotOffset = new Position();                     // map odo to robot (so it holds turn position better)
   public Position odoFieldStart = new Position();                      // field start position [blue right slot]

   Position odoRawPose = new Position (0,0,0);                 // original calculation of position before transforms applied
   Position odoRobotPose = new Position ();                             // odo mapped to robot position (minor change)
   Position odoFinalPose = new Position ();                             // odo mapped to field
   Position odoFieldOffset = new Position ();                           // transform from initial position (more relevant for slamra!)
   public Position odoRobotPosition; // = odoFieldStart.clone();
   final Position zeroPos = new Position (0, 0, 0);

   public double eTicksPerRotate;                            //169619;  // Both wheels have to be the same size

   public Odometry(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize() {
//      odoRobotPosition = odoFieldStart.clone();  //TODO: deal with odoFieldStart set to null
      odoData = new OdoData();
      if (!use3encoders) useFusedHeading = false;
      if (odoFieldStart!=null) odoRobotPosition = odoFieldStart.clone();
      else odoRobotPosition = new Position ();
      if (!parts.useODO) {
         odoRobotPosition = new Position(0,0, odoRobotPosition.R);
         return;
      }
      configureEncoders();
      if (use3encoders && (odoEncXL.ticksPerInch != odoEncXR.ticksPerInch)) { // this shouldn't have been called
         throw new RuntimeException("ticksPerInch must be the same for left and right deadwheels");
      }

      odoData.encoderXL = odoEncXL.encoderPort.getCurrentPosition() * (long)odoEncXL.direction;
      if (use3encoders) odoData.encoderXR = odoEncXR.encoderPort.getCurrentPosition() * (long)odoEncXR.direction;
      odoData.encoderY = odoEncY.encoderPort.getCurrentPosition() * (long)odoEncY.direction;
      //if (parts.useIMU) odoData.imuHeading = parts.imuMgr.returnImuHeadingRaw(true);
      if (parts.positionMgr.hasImusHeading()) odoData.imuHeading = parts.positionMgr.imusHeadingOnly.R;
      if (use3encoders) odoData.odoHeading = getOdoHeading();
      odoData.globalHeading = odoData.imuHeading;
      odoData0 = odoData.clone();

      // odo start position is 0,0,0; imu should also read 0.  odoRawPose is already 0,0,0
      odoRobotPose = getOdoRobotPose();
      odoFinalPose = getOdoFinalPose();
      setupFieldOffset();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      /* Update encoder readings */
      odoData.encoderXL = odoEncXL.encoderPort.getCurrentPosition() * (long)odoEncXL.direction;
      if (use3encoders) odoData.encoderXR = odoEncXR.encoderPort.getCurrentPosition() * (long)odoEncXR.direction;
      odoData.encoderY = odoEncY.encoderPort.getCurrentPosition() * (long)odoEncY.direction;

      /* Update heading */
      //if (parts.useIMU) odoData.imuHeading = parts.imuMgr.returnImuHeadingRaw();
      if (parts.positionMgr.hasImusHeading()) {
         odoData.imuHeading = parts.positionMgr.imusHeadingOnly.R;                  // todo: This isn't raw... Now what?
         odoFieldOffset.R = 0;  // if using final IMU heading, no need for offset?  // todo: OK?  this will mess up odoHeading absolute
      }
      if (use3encoders) odoData.odoHeading = getOdoHeading();
      odoData.globalHeading = fusedHeading();

      /* Calculate position */
      odoRawPose = updateOdoRawPose();
      odoRobotPose = getOdoRobotPose();
      odoFinalPose = getOdoFinalPose();

      /* Update robot position */
      odoRobotPosition = odoFinalPose.clone();

      /* Store current values for next loop */
      odoData0 = odoData.clone();
   }

   public void stop() {
   }

   private double fusedHeading() {
      /* If there is no IMU data available, need to use odoHeading only regardless of fused heading */
      if (!parts.positionMgr.hasImusHeading()) return odoData.odoHeading;
      /* Don't fuse if the flag isn't set */
      if (!useFusedHeading || !use3encoders) return odoData.imuHeading;
      /* Use imuHeading only if it's settled */
      if (Math.abs(Functions.normalizeAngle(odoData.imuHeading - odoData0.imuHeading)) < 0.5) return odoData.imuHeading;
      /* Otherwise fuse it with odoHeading data (relative change, not absolute) */
      return Functions.normalizeAngle(odoData0.globalHeading + (odoData.odoHeading - odoData0.odoHeading));
   }

   public void toggleUseFusedHeading() {
      useFusedHeading = !useFusedHeading;
   }

   /* Get heading from the odometry ... accuracy varies :-(  */
   private double getOdoHeading() {
      if (!use3encoders) { // this shouldn't have been called
         throw new RuntimeException("getOdoHeading cannot be used without 3 deadwheels");
      }
      double diffX;
      diffX = odoData.encoderXR - odoData.encoderXL;
      diffX = diffX % eTicksPerRotate;
      diffX = diffX / eTicksPerRotate * 360;
      return Functions.normalizeAngle(diffX);
   }

   public double returnOdoHeading() {
      return odoData.odoHeading;
   }

   public double returnGlobalHeading() {
      return odoData.globalHeading;
   }

   /* Get XY position data from odometry wheels */
   private Position updateOdoRawPose() {
      // this function could use some cleanup!
      double changeX, changeY;
      double myHeading;
      double xPos = odoRawPose.X;
      double yPos = odoRawPose.Y;

      /* Calculate the change in X & Y relative to the robot/encoders */
      if (use3encoders) changeX = (odoData.encoderXL + odoData.encoderXR - odoData0.encoderXL - odoData0.encoderXR) / 2.0 / odoEncXL.ticksPerInch;
      else changeX = (odoData.encoderXL - odoData0.encoderXL) / odoEncXL.ticksPerInch;  //for single encoder; no heading would be possible
      changeY = (odoData.encoderY - odoData0.encoderY) / odoEncY.ticksPerInch;

      /* Calculate average heading from previous loop to this (movement did not only at the end of the loop!) */
      myHeading = getAvgHeading(odoData0.globalHeading, odoData.globalHeading);
      TelemetryMgr.message(Category.ODOMETRY,"My Average Heading (raw)", JavaUtil.formatNumber(myHeading, 2));

      /* Calculate the new x and y positions */
      xPos = xPos + changeX * Math.cos(Math.toRadians(myHeading));
      yPos = yPos + changeX * Math.sin(Math.toRadians(myHeading));
      xPos = xPos + changeY * Math.sin(Math.toRadians(myHeading));
      yPos = yPos - changeY * Math.cos(Math.toRadians(myHeading));

      return new Position(xPos, yPos, odoData.globalHeading);
   }

   /* Calculate average of two headings */
   public static double getAvgHeading (double firstHeading, double secondHeading) {
      double robotHeading;
      /* Find the difference between them; based on sampling rate, assume large values wrapped */
      robotHeading = Functions.normalizeAngle(secondHeading - firstHeading);
      robotHeading /= 2;
      robotHeading += firstHeading;
      return Functions.normalizeAngle(robotHeading);
   }

   Position getOdoRobotPose() {
      //pos1 = odoRawPose, pos2 = odoRobotOffset
      return transformPosition(odoRawPose, odoRobotOffset);
   }

   Position getOdoFinalPose() {
      //pos1 = odoFieldOffset, pos2 = odoRobotPose
      Position odoFinal;
      odoFinal = transformPosition(odoFieldOffset, odoRobotPose);  //todo: get this reversed to be more intuitive; field offset will have to be calculated differently
      odoFinal.normalize();
      return odoFinal;
   }

   public boolean isOdoOffset() {
      return !odoFieldOffset.isEqualTo(zeroPos);
   }

   public boolean isOdoPositionGood() {
      return isOdoOffset();
      // could add future checks for weirdness indicating a wheel lifted or something
   }

   public void setupFieldOffset(Position fieldPosition) {
      odoFieldOffset = zeroPos;    // clear any existing offset
      odoFieldOffset = getOdoFieldOffset(odoRobotPose, fieldPosition);
   }
   public void setupFieldOffset() {
      odoFieldOffset = zeroPos;    // clear any existing offset
      if (odoFieldStart!=null) odoFieldOffset = getOdoFieldOffset(odoRobotPose, odoFieldStart);
      // if the field offset is 0,0,0, it can be known that it was not properly offset
   }

   Position getOdoFieldOffset(Position robotPose, Position fieldPose) {  //todo: maybe this should be inverted somehow?
      double offsetR = fieldPose.R - robotPose.R;
      return new Position (
              fieldPose.X - (robotPose.X*Math.cos(Math.toRadians(offsetR)) - robotPose.Y*Math.sin(Math.toRadians(offsetR))),
              fieldPose.Y - (robotPose.X*Math.sin(Math.toRadians(offsetR)) + robotPose.Y*Math.cos(Math.toRadians(offsetR))),
              offsetR
      );
   }

   Position transformPosition(Position pos1, Position pos2) {  //todo: figure out how to move this (both odometry and slamra) to Position methods
      return new Position(
              pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R))),
              pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R))),
              pos1.R + pos2.R
      );
   }

   public void addTeleOpTelemetry() {
      TelemetryMgr.message(Category.ODOMETRY, "odo-fldof", odoFieldOffset.toString(2));
      TelemetryMgr.message(Category.ODOMETRY, "odo-raw__", odoRawPose.toString(2));
      TelemetryMgr.message(Category.ODOMETRY, "odo-robot", odoRobotPose.toString(2));
      TelemetryMgr.message(Category.ODOMETRY, "odo-final", odoFinalPose.toString(2));
   }

   /* this method is separated with the intent to be overridden */
   public void configureEncoders() {
      double TPI = 82300 / 48.0;
      double TPR = 169619;
      //2022-12-22 Measured 10 rot: 846684,850800; 845875,851775; 845127,845543; 848073,850867
      //                            169748.4		169765		169067		169894		==>  169618.6

      /* direction should be -1 or 1 and is to account for the port being used elsewhere
         and needing to be set in direction opposite to what odometry needs */
      odoEncXL = new EncoderSetting(parts.robot.motor2B, 1, TPI);
      odoEncXR = new EncoderSetting(parts.robot.motor1B, 1, TPI);
      odoEncY  = new EncoderSetting(parts.robot.motor0B, 1, TPI);
      odoEncXL.encoderPort.setDirection(DcMotorEx.Direction.REVERSE);
      odoEncXR.encoderPort.setDirection(DcMotorEx.Direction.REVERSE);
      odoEncY.encoderPort.setDirection(DcMotorEx.Direction.FORWARD);
      odoEncXL.encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      odoEncXR.encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      odoEncY.encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      eTicksPerRotate = TPR;
   }

   public static class OdoData {
      long encoderXL, encoderXR, encoderY;
      double odoHeading, imuHeading, globalHeading;
      public OdoData(long encoderXL, long encoderXR, long encoderY, double odoHeading, double imuHeading, double globalHeading) {
         this.encoderXL = encoderXL;
         this.encoderXR = encoderXR;
         this.encoderY = encoderY;
         this.odoHeading = odoHeading;
         this.imuHeading = imuHeading;
         this.globalHeading = globalHeading;
      }
      public OdoData() {}
      @NonNull
      public OdoData clone() {
         return new OdoData(encoderXL, encoderXR, encoderY, odoHeading, imuHeading, globalHeading);
      }
   }

   public static class EncoderSetting {
      public DcMotorEx encoderPort;
      byte direction;
      double ticksPerInch;
      public EncoderSetting(DcMotorEx encoderPort, int direction, double ticksPerInch) {
         this.encoderPort = encoderPort;
         this.direction = (byte)direction;
         this.ticksPerInch = ticksPerInch;
      }
   }
}