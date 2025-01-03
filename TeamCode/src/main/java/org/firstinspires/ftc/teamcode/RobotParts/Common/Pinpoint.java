package org.firstinspires.ftc.teamcode.RobotParts.Common;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.i2c.GoBildaPinpointDriver;

import java.util.Locale;

public class Pinpoint implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   GoBildaPinpointDriver pinpoint;

   public String deviceName;
   public float resolution;
   public GoBildaPinpointDriver.EncoderDirection xDirection, yDirection;

   public Position pinpointRobotPosition;
   public Position pinpointRobotOffset = new Position(-84,-168,0);   // map pinpoint to robot (so it holds turn position better)
   public Position pinpointFieldStart = new Position();                      // field start position [blue right slot]

   /* Constructor */
   public Pinpoint(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      pinpointSettings();
      pinpoint = parts.opMode.hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
      pinpoint.setOffsets(pinpointRobotOffset.X, pinpointRobotOffset.Y); //these are tuned for 3110-0002-0001 Product Insight #1
      //pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
      pinpoint.setEncoderResolution(resolution);
      pinpoint.setEncoderDirections(xDirection, yDirection);
      pinpoint.resetPosAndIMU();
   }

   public void preInit() {
      setPosition(pinpointFieldStart);
   }

   public void initLoop() {
      update();
   }

   public void preRun() {
      setPosition(pinpointFieldStart);
   }

   public void runLoop() {
      update();
   }

   public void stop() {
   }

   public void setPosition(Position pos) {
      pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pos.X, pos.Y, AngleUnit.DEGREES, pos.R));
   }

   public void update() {
      pinpoint.update();

      Pose2D pos = pinpoint.getPosition();
      String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
      TelemetryMgr.message(TelemetryMgr.Category.PINPOINT, "Position", data);

      Pose2D vel = pinpoint.getVelocity();
      String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
      TelemetryMgr.message(TelemetryMgr.Category.PINPOINT, "Velocity", velocity);

      GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
      TelemetryMgr.message(TelemetryMgr.Category.PINPOINT, "Status", status);
            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */

      if (status==GoBildaPinpointDriver.DeviceStatus.READY) {
         pinpointRobotPosition = new Position(
                 pos.getX(DistanceUnit.INCH),
                 pos.getY(DistanceUnit.INCH),
                 pos.getHeading(AngleUnit.DEGREES)
                 );
      }
      else {
         pinpointRobotPosition = null;
      }
      TelemetryMgr.message(TelemetryMgr.Category.PINPOINT, "Final", (pinpointRobotPosition==null) ? "(null)" : pinpointRobotPosition.toString(2));

   }

   /* this method is separated with the intent to be overridden */
   public void pinpointSettings() {
      deviceName = "pinpoint";
      xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
      yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
      resolution = 13.26291192f;
      // private static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
      // private static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod
      //offsets are taken care of with parts.pinpointRobotOffset
      ////xOffset = -84.0;
      ////yOffset = -168.0;
   }

}