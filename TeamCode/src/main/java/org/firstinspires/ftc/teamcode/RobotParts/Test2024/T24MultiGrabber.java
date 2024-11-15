package org.firstinspires.ftc.teamcode.RobotParts.Test2024;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.LegacyBots.LiftbotLifter;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class T24MultiGrabber implements PartsInterface {

   /* Settings */
   static final double pinchClosed              = 0.717;
   static final double pinchLoose               = 0.682;
   static final double pinchSlightOpen          = 0.571;
   static final double pinchFullOpen            = 0.340;
   static final double pinchClearsSamples       = 0.431;
//   static final int pinchCloseTime              = 250;
//   static final int pinchOpenTime               = 250;

   static final double wristCenter              = 0.500;
   static final double wrist90Left              = 0.140;
   static final double wrist90Right             = 0.877;
//   static final int wristReadyToGrabTime        = 500;
//   static final int wristHorizToReadyTime       = 750;

   static final double rotatorCenter            = 0.500;
   static final double rotator45Left            = 0.695;
   static final double rotator45Right           = 0.276;
   static final double rotatorMildLeft          = 0.595;
   static final double rotatorMild5Right        = 0.362;

   static final double shoulderVertical             = 0.642;
   static final double shoulderFullBack             = 0.955;
   static final double shoulderHorizontal           = 0.260;
   static final double shoulderCapture              = 0.201;
   static final double shoulderSafe                 = 0.350;   // Todo THIS IS NOT REAL - GET VALUE

   private int slideTargetPosition;

   /* Internal use */
   private static Servo servoPinch;
   private static Servo servoWrist;
   private static Servo servoRotator;
   private static Servo servoShoulder;
   private static DcMotorEx motorIntake;
   private static boolean servoPinchDisabled = false;
   private static boolean servoWristDisabled = false;
   private static boolean servoshoulderDisabled = false;
   private static boolean servoRotatorDisabled = false;
   private static long timerWrist = System.currentTimeMillis();
   private static long timerPinch = System.currentTimeMillis();
   private static long timershoulder = System.currentTimeMillis();
   private static long timerRotator = System.currentTimeMillis();
   public static long dropTimer = System.currentTimeMillis();
   public static boolean isArmed = false;
   public static int intakeState = 0;

   boolean isSlideUnderManualControl = false;
   boolean isSlideHomed = false;
   boolean isSlideHoldDeferred = false;
   final int positionSlideMin                = 0;
   final int positionSlideMax                = 3200;   //todo: get real number
   final int positionSlideStartIntake        = 1500;   //todo: get real number
   final int positionSlidePitMin             = 500;    //todo: get real number
   int toleranceSlide                        = 20;

   /* Public OpMode members. */
   public static Parts parts;

   /* Constructor */
   public T24MultiGrabber(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      servoRotator = parts.robot.servo0;
      servoShoulder = parts.robot.servo2;
      servoWrist = parts.robot.servo4;
      servoPinch = parts.robot.servo0B;
      motorIntake = parts.robot.motor0B;
      initServos();
      initMotors();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      delayedActions();
   }

   public void stop() {
   }

   public void eStop() {
      parts.robot.disableServo(servoPinch);
      parts.robot.disableServo(servoWrist);
      servoPinchDisabled = true;
      servoWristDisabled = true;
      isArmed = false;
   }

   // this all needs to be improved to prevent crashes, etc.
   public static void grabberSafe() {
//      setWristServo(wristHorizontal);
      setPinchServo(pinchFullOpen);
   }
   public static void grabberArmed() {
//      setWristServo(wristReady);
      setPinchServo(pinchFullOpen);
//      setPinchServo(pinchSlightOpen);
   }
   public static void grabberStartGrab() {
//      setWristServo(wristGrab);
      setPinchServo(pinchFullOpen);
//      setPinchServo(pinchSlightOpen);
   }
   public static void grabberGrab() {
//      setWristServo(wristGrab);
      setPinchServo(pinchClosed);
   }
   public static void grabberVertical() {
//      setWristServo(wristVertical);
      setPinchServo(pinchClosed);
   }
   public static void grabberMaxBack() {
//      setWristServo(wristMaxBack);
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
         parts.robot.enableServo(servoWrist);
      }
      if (isServoAtPosition(servoWrist, newPosition)) return;  // has already been set (but not necessarily done moving)
      servoWrist.setPosition(newPosition);
//      timerWrist = System.currentTimeMillis() + gateSweepTime;
   }

   public static void setPinchServo(double newPosition) {
      if (servoPinchDisabled) {
         servoPinchDisabled = false;
         parts.robot.enableServo(servoPinch);
      }
      if (isServoAtPosition(servoPinch, newPosition)) return;  // has already been set (but not necessarily done moving)
      servoPinch.setPosition(newPosition);
//      timerPinch = System.currentTimeMillis() + pusherSweepTime;
   }
//
//   public static boolean isGateOpen() {
//      return isServoAtPosition(servoWrist,gateOpen, timerWrist);
//   }
//   public boolean isGateClosed() {
//      return isServoAtPosition(servoWrist,gateClosed, timerWrist);
//   }
//   public boolean isGateDoneMoving () {
//      return System.currentTimeMillis() >= timerWrist;
//   }
//
//   public static boolean isPusherExtended() {
//      return isServoAtPosition(servoPinch,pusherExtended, timerPinch);
//   }
//   public static boolean isPusherRetracted() {
//      return isServoAtPosition(servoPinch,pusherRetracted, timerPinch);
//   }
//   public boolean isPusherDoneMoving () {
//      return System.currentTimeMillis() >= timerPinch;
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

   public void initServos () {
      servoRotator.setDirection(Servo.Direction.FORWARD);
      servoShoulder.setDirection(Servo.Direction.FORWARD);
      servoWrist.setDirection(Servo.Direction.FORWARD);
      servoPinch.setDirection(Servo.Direction.FORWARD);

      servoRotator.setPosition(rotatorCenter);
      servoShoulder.setPosition(shoulderSafe);
      servoWrist.setPosition(wristCenter);
      servoPinch.setPosition(pinchFullOpen);
   }

   public void initMotors () {
      stopMotors();
      motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
      motorIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      slideTargetPosition = 0;
      motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //  Actuators: Motors and Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public void stopMotors() {
      motorIntake.setPower(0);
   }

   public void setDrivePowers (double m0) {
      motorIntake.setPower(m0);
   }

   public void stopMotorsAndHold() {
      setSlidePosition(motorIntake.getCurrentPosition(),0.5);
   }

//   public void setGrabServo (double goTo) {
//      grabOpenRequested = -1;
//      if (!isServoAtPosition(servoGrab, goTo)) {
//         if (goTo == grabberServoClosePos) {   // Closing should always be safe
//            servoGrab.setPosition(goTo);
//         } else if (isGrabberSafeToOpen()) {   // Opening is safe if enough time has passed
//            servoGrab.setPosition(goTo);
//         } else {
//            grabOpenRequested = goTo;         // for delayed opening
//         }
//      }
//   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //       Manual User Drive
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   public void setUserDriveSettings(double driveSpeed) {
      if (driveSpeed == 0 && !isSlideUnderManualControl) return;

      if (driveSpeed != 0) {
         int currentPos = motorIntake.getCurrentPosition();
         //enforce upper limits
         if (driveSpeed > 0 && currentPos > positionSlideMax) driveSpeed = 0;
         //enforce lower limits
         if (driveSpeed < 0 && currentPos < positionSlideMin) driveSpeed = 0;
//         if (driveSpeed < 0 && isLimitSwitchPressed()) driveSpeed = 0;
      }

      if (driveSpeed == 0) {  // when it drops out of manual control, hold
         isSlideUnderManualControl = false;
         stopMotors();
         isSlideHoldDeferred = true;
         return;
      }

      if (!isSlideUnderManualControl) {
         isSlideUnderManualControl = true;
         //stopStateMachine();
         stopMotors();
         motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         return;  // we'll set the speed next time... jerky if you do it immediately after changing mode
      }

      setDrivePowers(driveSpeed);
   }

   public void delayedActions() {
//      if (grabOpenRequested!=-1) setGrabServo(grabOpenRequested);  // simple handling of delayed servo opening
      if (isSlideHoldDeferred) {
         stopMotorsAndHold();
         isSlideHoldDeferred = false;
      };
   }

   public void setSlidePosition(int goTo) {
      setSlidePosition(goTo, 1);
   }

   public void setSlidePosition(int goTo, double pwr) {
//      if (stateMachineType != 0) {    // limits ignored if homing
         if (goTo < positionSlideMin || goTo > positionSlideMax) {  // something very wrong so bail
            stopMotors();
            return;
         }
//      }
      slideTargetPosition = goTo;
      stopMotors();
      motorIntake.setTargetPosition(slideTargetPosition);
      motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      setDrivePowers(pwr);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Actions
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   public void action(IntakeActions action) {
      switch (action) {
         case AUTO_EXTEND_TO_GRAB:
            break;
         case AUTO_RETRACT:
            break;
         case AUTO_GRAB:
            break;
         case AUTO_GRAB_AND_RETRACT:
            break;
         case AUTO_HOME:
            break;
         case AUTO_MAKE_SPACE:
            break;
         case SAFE:
            break;
         case GRAB_HOVER:
            break;
         case GRAB_OPEN:
            break;
         case GRAB_WIDEOPEN:
//            setGrabServo(grabberServoClosePos);
            break;
         case GRAB_CLOSE:
            break;
         case CANCEL:
            break;
         default:
            break;
      }
   }

   public enum IntakeActions {
      AUTO_EXTEND_TO_GRAB,
      AUTO_RETRACT,
      AUTO_GRAB,
      AUTO_GRAB_AND_RETRACT,
      AUTO_HOME,
      AUTO_MAKE_SPACE,
      SAFE,
      GRAB_HOVER,
      GRAB_OPEN,
      GRAB_WIDEOPEN,
      GRAB_CLOSE,
      CANCEL
   }
}