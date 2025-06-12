package org.firstinspires.ftc.teamcode.RobotParts.Test2024;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class T24Grabber implements PartsInterface {

   /* Settings */
   static final double pinchClosed              = 0.510;
   static final double pinchSlightOpen          = 0.455;
   static final double pinchFullOpen            = 0.400;
   static final int pinchCloseTime              = 250;
   static final int pinchOpenTime               = 250;

   static final double wristHorizontal          = 0.500;
   static final double wristReady               = 0.680;
   static final double wristGrab                = 0.830;
   static final double wristVertical            = 0.176;
   static final double wristMaxBack             = 0.000;
   static final int wristReadyToGrabTime        = 500;
   static final int wristHorizToReadyTime       = 750;

   /* Internal use */
   private static ServoSSR servoPinch;
   private static ServoSSR servoWrist;
   private static boolean servoPinchDisabled = false;
   private static boolean servoWristDisabled = false;
   private static long wristTimer = System.currentTimeMillis();
   private static long pinchTimer = System.currentTimeMillis();
   public static long dropTimer = System.currentTimeMillis();
   public static boolean isArmed = false;
   public static int intakeState = 0;

   /* Public OpMode members. */
   public static Parts parts;

   /* Constructor */
   public T24Grabber(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      servoPinch = new ServoSSR(parts.robot.servo0);
      servoWrist = new ServoSSR(parts.robot.servo2);
      servoPinch.setPosition(pinchSlightOpen);
      servoWrist.setPosition(wristHorizontal);
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
//      parts.robot.disableServo(servoPinch);
//      parts.robot.disableServo(servoWrist);
      servoPinch.disable();
      servoWrist.disable();
      servoPinchDisabled = true;
      servoWristDisabled = true;
      isArmed = false;
   }

   // this all needs to be improved to prevent crashes, etc.
   public static void grabberSafe() {
      setWristServo(wristHorizontal);
      setPinchServo(pinchFullOpen);
   }
   public static void grabberArmed() {
      setWristServo(wristReady);
      setPinchServo(pinchFullOpen);
//      setPinchServo(pinchSlightOpen);
   }
   public static void grabberStartGrab() {
      setWristServo(wristGrab);
      setPinchServo(pinchFullOpen);
//      setPinchServo(pinchSlightOpen);
   }
   public static void grabberGrab() {
      setWristServo(wristGrab);
      setPinchServo(pinchClosed);
   }
   public static void grabberVertical() {
      setWristServo(wristVertical);
      setPinchServo(pinchClosed);
   }
   public static void grabberMaxBack() {
      setWristServo(wristMaxBack);
      setPinchServo(pinchClosed);
   }
   public static void grabberRelease() {
      setPinchServo(pinchSlightOpen);
   }

//   public static void openGate() {
//      setGateServo(gateOpen);
//   }
//   public static void closeGate() {
//      setGateServo(gateClosed);
//      isArmed = false;
//   }
//   public static void extendPusher() {
//      setPusherServo(pusherExtended);
//   }
//   public static void retractPusher() {
//      setPusherServo(pusherRetracted);
//   }
//
   public static void setWristServo(double newPosition) {
      if (servoWristDisabled) {
         servoWristDisabled = false;
//         parts.robot.enableServo(servoWrist);
         servoWrist.enable();
      }
      if (isServoAtPosition(servoWrist, newPosition)) return;  // has already been set (but not necessarily done moving)
      servoWrist.setPosition(newPosition);
//      wristTimer = System.currentTimeMillis() + gateSweepTime;
   }

   public static void setPinchServo(double newPosition) {
      if (servoPinchDisabled) {
         servoPinchDisabled = false;
//         parts.robot.enableServo(servoPinch);
         servoPinch.enable();
      }
      if (isServoAtPosition(servoPinch, newPosition)) return;  // has already been set (but not necessarily done moving)
      servoPinch.setPosition(newPosition);
//      pinchTimer = System.currentTimeMillis() + pusherSweepTime;
   }
//
//   public static boolean isGateOpen() {
//      return isServoAtPosition(servoWrist,gateOpen, wristTimer);
//   }
//   public boolean isGateClosed() {
//      return isServoAtPosition(servoWrist,gateClosed, wristTimer);
//   }
//   public boolean isGateDoneMoving () {
//      return System.currentTimeMillis() >= wristTimer;
//   }
//
//   public static boolean isPusherExtended() {
//      return isServoAtPosition(servoPinch,pusherExtended, pinchTimer);
//   }
//   public static boolean isPusherRetracted() {
//      return isServoAtPosition(servoPinch,pusherRetracted, pinchTimer);
//   }
//   public boolean isPusherDoneMoving () {
//      return System.currentTimeMillis() >= pinchTimer;
//   }

   public static boolean isServoAtPosition(Servo servo, double comparePosition, long servoTimer) {
      return isServoAtPosition(servo.getPosition(), comparePosition) && System.currentTimeMillis() >= servoTimer;
   }
   public static boolean isServoAtPosition(Servo servo, double comparePosition) {
      return isServoAtPosition(servo.getPosition(), comparePosition);
   }
   public static boolean isServoAtPosition(double servoPosition, double comparePosition) {
      return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
   }

}