package org.firstinspires.ftc.teamcode.RobotParts.Common.Position;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

// Based loosely on Om's very basic EncoderTracker

public class EncoderTracker implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;
   public Position encoderRobotPosition;          // updated each loop by other sources, if available
   public Position encoderRobotPositionAbsolute;  // Not updated by other sources (likely to be highly inaccurate)

   int[] lastMotorPos, currentMotorPos;
   double lastHeading, currentHeading;
   double ticksPerInchSideways;
   double ticksPerInchForward;

   /* Constructor */
   public EncoderTracker(Parts parts){
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
      configure();
      lastMotorPos = parts.drivetrain.getDriveEncoderValues();
      encoderRobotPosition.update(parts.fieldStartPosition);
      encoderRobotPositionAbsolute.update(parts.fieldStartPosition);
      lastHeading = encoderRobotPosition.R;
   }

   public void runLoop() {
      // For encoderRobotPosition, the starting position will be whatever is stored as the robotPosition.
      encoderRobotPosition.update(parts.positionMgr.robotPosition);
      currentHeading = encoderRobotPosition.R;
      encoderRobotPositionAbsolute.R = currentHeading;

      // Update the encoder values and calculate the changes from the last loop
      currentMotorPos = parts.drivetrain.getDriveEncoderValues();
      int[] diff = new int[]{
         currentMotorPos[0] - lastMotorPos[0],
         currentMotorPos[1] - lastMotorPos[1],
         currentMotorPos[2] - lastMotorPos[2],
         currentMotorPos[3] - lastMotorPos[3],
      };
      lastMotorPos = currentMotorPos;

      // Get the X and Y movement of the robot  [corrected from Om's orientation]
      double changeX = .25 * (diff[0] + diff[1] + diff[2] + diff[3]) / ticksPerInchForward;
      double changeY = -.25 * (-diff[0] + diff[1] + diff[2] - diff[3]) / ticksPerInchSideways;  //todo: seems backwards; perhaps updatePosition is wrong

      // Calculate average heading from previous loop to this (movement did not only at the end of the loop!)
      double avgHeading = getAvgHeading(lastHeading, currentHeading);

      // Update the stored positions with the changes
      updatePosition(encoderRobotPosition, changeX, changeY, avgHeading);
      updatePosition(encoderRobotPositionAbsolute, changeX, changeY, avgHeading);

      lastHeading = currentHeading;

      TelemetryMgr.message(TelemetryMgr.Category.ENCODER_EXT, "Raw X", JavaUtil.formatNumber(changeX,2));
      TelemetryMgr.message(TelemetryMgr.Category.ENCODER_EXT, "Raw Y", JavaUtil.formatNumber(changeY,2));
      TelemetryMgr.message(TelemetryMgr.Category.ENCODER, "Updated ", encoderRobotPosition.toString(2));
      TelemetryMgr.message(TelemetryMgr.Category.ENCODER, "Absolute", encoderRobotPositionAbsolute.toString(2));
   }

   public void stop() {
   }

   public static void updatePosition (Position pos, double X, double Y, double R) {
      /* Calculate the new x and y positions (borrowed from Odometry) */
      pos.X += X * Math.cos(Math.toRadians(R));
      pos.Y += X * Math.sin(Math.toRadians(R));
      pos.X += Y * Math.sin(Math.toRadians(R));
      pos.Y += -Y * Math.cos(Math.toRadians(R));
   }

   /* Calculate average of two headings (borrowed from Odometry) */
   public static double getAvgHeading (double firstHeading, double secondHeading) {
      double robotHeading;
      /* Find the difference between them; based on sampling rate, assume large values wrapped */
      robotHeading = Functions.normalizeAngle(secondHeading - firstHeading);
      robotHeading /= 2;
      robotHeading += firstHeading;
      return Functions.normalizeAngle(robotHeading);
   }

   /* this method is separated with the intent to be overridden */
   public void configure() {
      // This should be calibrated for the robot. Wheel size, traction, and robot weight likely affect this.
      // The numbers currently here were calculated by Om, and then scaled for the new 104mm wheels
    /* team numbers */
      ticksPerInchSideways = 52.38 * 96/104;
      ticksPerInchForward = 45.68 * 96/104;
    /* mentorbot below */
      ticksPerInchForward = 45.68 * 96/100 * 312/435*2;  //62.9
      ticksPerInchSideways = 52.38 * 96/100 * 312/435*2; //72.1
      //measured...
      ticksPerInchForward = (1462+1435+1457+1464)/4/23.5;  //61.9
      ticksPerInchSideways = ((1865+1699+1590+1545)+(1730+1702+1742+1753)+(1560+1524+1532+1545)+(1772+1650+1602+1613))/16/23.5;   //70.3
   }

}