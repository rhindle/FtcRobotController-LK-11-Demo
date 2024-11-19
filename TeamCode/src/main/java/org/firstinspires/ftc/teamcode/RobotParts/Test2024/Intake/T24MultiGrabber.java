package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class T24MultiGrabber implements PartsInterface {

   /* Settings */
   static final double pinchClosed              = 0.717;
   static final double pinchLoose               = 0.682;
   static final double pinchTightOpen           = 0.622;
   static final double pinchSlightOpen          = 0.571;
   static final double pinchClearsSamples       = 0.431;
   static final double pinchFullOpen            = 0.340;
   static final double pinchLongDirection       = 0.560;
   static final int pinchSweepTime              = 1500;   // spec is 1250

   static final double wristCenter              = 0.500;
   static final double wrist90Left              = 0.140;
   static final double wrist90Right             = 0.877;
   static final double wristTransfer            = 0.467;
   static final int wristSweepTime              = 1500;

   static final double rotatorCenter            = 0.500;
   static final double rotator45Left            = 0.695;
   static final double rotator45Right           = 0.276;
   static final double rotatorMildLeft          = 0.595;
   static final double rotatorMildRight         = 0.362;
   static final double rotatorTransfer          = 0.536;
   static final int rotatorSweepTime            = 1500;

   static final double shoulderVertical         = 0.580;
   static final double shoulderFullBack         = 0.920;
   static final double shoulderHorizontal       = 0.260;
   static final double shoulderHover            = 0.230;
   static final double shoulderCapture          = 0.201;
   static final double shoulderDrag             = 0.180;
   static final double shoulderPush             = 0.200;
   static final double shoulderBalanced         = 0.643;
   static final double shoulderSafeOut          = 0.356;  //0.339;   // Todo THIS IS NOT REAL - GET VALUE
   static final double shoulderSafeIn           = 0.314;
   static final int shoulderSweepTime           = 1500;    // this is faster, but under load; need actual estimate

   static final double liftShoulderTransfer     = 0.831;
   static final double liftShoulderBack         = 0.000;
   static final double liftShoulderSafe         = 0.522;
   static final int liftShoulderSweepTime       = 1500;

   static final double liftPinchTransfer        = 0.905;
   static final double liftPinchSafe            = 0.854;
   static final double liftPinchWide            = 0.810;
   static final int liftPinchSweepTime          = 1500;

   static final int positionSlideMin            = 10;
   static final int positionSlideMax            = 1500;
   static final int positionSlideStartIntake    = 650;   //todo: finalize number
   static final int positionSlidePitMin         = 160;    //todo: finalize number
   static final int toleranceSlide              = 20;

   static final int positionLiftMin             = 10;
   static final int positionLiftMax             = 4000; //4350;   //todo:reverse direction
   static final int positionLiftTransfer        = 125;
   static final int toleranceLift               = 20;

   public static boolean slideOverride          = false;

   /* Internal use */
   private static Servo servoPinch;
   private static Servo servoWrist;
   private static Servo servoRotator;
   private static Servo servoShoulder;
   static Servo servoLiftShoulder;
   static double shoulderNominalPosition;
   private static Servo servoLiftPinch;
   private static DcMotorEx motorSlide;
   private static DcMotorEx motorLift;
   public static DigitalChannel slideLimitSwitchNO = null;
   public static DigitalChannel slideLimitSwitchNC = null;
   public static DigitalChannel liftLimitSwitchNO = null;
   public static DigitalChannel liftLimitSwitchNC = null;
   private static byte slideLimit = -1;
   private static byte liftLimit = -1;
   private static boolean slideLimitJustPressed = false;
   private static boolean liftLimitJustPressed = false;
   private static boolean servoPinchDisabled = false;
   private static boolean servoWristDisabled = false;
   private static boolean servoShoulderDisabled = false;
   private static boolean servoRotatorDisabled = false;
   private static boolean servoLiftShoulderDisabled = false;
   private static boolean servoLiftPinchDisabled = false;
   private static long timerWrist = System.currentTimeMillis();
   private static long timerPinch = System.currentTimeMillis();
   private static long timerShoulder = System.currentTimeMillis();
   private static long timerRotator = System.currentTimeMillis();
   private static long timerLiftShoulder = System.currentTimeMillis();
   private static long timerLiftPinch = System.currentTimeMillis();
//   public static int intakeState = 0;
   private static int slideTargetPosition;
   private static int liftTargetPosition;
   static boolean isSlideUnderManualControl = false;
   static boolean isLiftUnderManualControl = false;
   static boolean isSlideHomed = false;
   static boolean isLiftHomed = false;
   static boolean isSlideHoldDeferred = false;
   static boolean isLiftHoldDeferred = false;

   /* Internal use (Needs access by state machines in package) */




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
      servoLiftShoulder = parts.robot.servo2B;
      servoLiftPinch = parts.robot.servo4B;
      motorSlide = parts.robot.motor0B;
      motorLift = parts.robot.motor1B;
      slideLimitSwitchNO = parts.robot.digital1B;
      slideLimitSwitchNC = parts.robot.digital0B;
      liftLimitSwitchNO = parts.robot.digital3B;
      liftLimitSwitchNC = parts.robot.digital2B;
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
      updateLimits();
      delayedActions();
      fineTuneShoulder();

      smGoFish.stateMachine();
      smGrabAndRetract.stateMachine();
      smMakeSpace.stateMachine();
      smTransfer.stateMachine();
      smGrabAndInspect.stateMachine();
      smRetract.stateMachine();

      TelemetryMgr.message(TelemetryMgr.Category.T24MULTIGRAB,
              "States: " +
                      "GF: " + String.format("%02d", smGoFish.getState()) +
                      ", RI: " + String.format("%02d", smGrabAndRetract.getState()) +
                      ", MS: " + String.format("%02d", smMakeSpace.getState()) +
                      ", TR: " + String.format("%02d", smTransfer.getState()) +
                      ", GI: " + String.format("%02d", smGrabAndInspect.getState()) +
                      ", RE: " + String.format("%02d", smRetract.getState()) +
                      "");
   }

   public void stop() {
   }

   public void eStop() {
      stopMotors();
      cancelStateMachines();
      parts.robot.disableServo(servoPinch);
      parts.robot.disableServo(servoWrist);
      parts.robot.disableServo(servoShoulder);
      parts.robot.disableServo(servoRotator);
      parts.robot.disableServo(servoLiftShoulder);
      parts.robot.disableServo(servoLiftPinch);
      servoPinchDisabled = true;
      servoWristDisabled = true;
      servoShoulderDisabled = true;
      servoRotatorDisabled = true;
      servoLiftShoulderDisabled = true;
      servoLiftPinchDisabled = true;
//      isArmed = false;
   }

   public void cancelStateMachines() {
      smGoFish.mildStop();
      smGrabAndRetract.mildStop();
      smMakeSpace.mildStop();
      smTransfer.mildStop();
      smGrabAndInspect.mildStop();
      smRetract.mildStop();
//      isArmed = false;
   }

   public void stopStateMachines() {
      smGoFish.stop();
      smGrabAndRetract.stop();
      smMakeSpace.stop();
      smTransfer.stop();
      smGrabAndInspect.stop();
      smRetract.stop();
//      isArmed = false;
   }

   public void updateLimits() {
      byte slideTemp;
      byte liftTemp;
      // Figure out current state
      if (slideLimitSwitchNO.getState() && !slideLimitSwitchNC.getState()) slideTemp=0;
      else if (slideLimitSwitchNC.getState() && !slideLimitSwitchNO.getState()) slideTemp=1;
      else slideTemp = -1;
      if (liftLimitSwitchNO.getState() && !liftLimitSwitchNC.getState()) liftTemp=0;
      else if (liftLimitSwitchNC.getState() && !liftLimitSwitchNO.getState()) liftTemp=1;
      else liftTemp = -1;
      // Figure out if state changed to pressed
      slideLimitJustPressed = false;
      liftLimitJustPressed = false;
      if (slideLimit!=1 && slideTemp==1) slideLimitJustPressed=true;
      if (liftLimit!=1 && liftTemp==1) liftLimitJustPressed=true;
      // update state variables
      slideLimit = slideTemp;
      liftLimit = liftTemp;
      // reset encoders?
      if (slideLimitJustPressed) motorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      if (liftLimitJustPressed) motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
   }

   public void fineTuneShoulder() {
      //adjusts the shoulder to account for droop over extended distance
      //measured 0.230 @ 150, 0.253 @ 1500 (difference of 0.023)
      // ignore if slide retracted
      if (!isSamplingInProcess()) return;
      int slideCurrentPosition = motorSlide.getCurrentPosition();
//      if (slideCurrentPosition < positionSlidePitMin) return;
//      // ignore if servo isn't lowered to an intake position
//      if (shoulderNominalPosition >= shoulderSafeIn) return;
      // now calculate
      double adjustDiff = 0.023;
      double currentPosition = servoShoulder.getPosition();
      double newPosition = shoulderNominalPosition + Functions.interpolate(slideCurrentPosition, positionSlidePitMin, positionSlideMax, 0, adjustDiff);
      double change = newPosition - currentPosition;
      // only adjust if there's been enough change.  0.003 corresponds to 7 increments.
      if (Math.abs(change) > 0.003) servoShoulder.setPosition(newPosition);
   }

   ////// Probably should set up a new class to hold servos, enabled state, timer, etc.
   public static void setWristServo(double newPosition) {
      if (servoWristDisabled) {
         servoWristDisabled = false;
         parts.robot.enableServo(servoWrist);
      }
      if (isServoAtPosition(servoWrist, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerWrist = getServoSweepTimerValue(servoWrist,newPosition,wristSweepTime);  // get timer before setting position!
      servoWrist.setPosition(newPosition);
   }
   public static void setPinchServo(double newPosition) {
      if (servoPinchDisabled) {
         servoPinchDisabled = false;
         parts.robot.enableServo(servoPinch);
      }
      if (isServoAtPosition(servoPinch, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerPinch = getServoSweepTimerValue(servoPinch,newPosition,pinchSweepTime);  // get timer before setting position!
      servoPinch.setPosition(newPosition);
   }
   public static void setRotatorServo(double newPosition) {
      if (servoRotatorDisabled) {
         servoRotatorDisabled = false;
         parts.robot.enableServo(servoRotator);
      }
      if (isServoAtPosition(servoRotator, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerRotator = getServoSweepTimerValue(servoRotator,newPosition,rotatorSweepTime);  // get timer before setting position!
      servoRotator.setPosition(newPosition);
   }
   public static void setShoulderServo(double newPosition) {
      if (servoShoulderDisabled) {
         servoShoulderDisabled = false;
         parts.robot.enableServo(servoShoulder);
      }
      if (shoulderNominalPosition==newPosition) return;
      //if (isServoAtPosition(servoShoulder, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      shoulderNominalPosition = newPosition;  // special case for the shoulder servo only
      timerShoulder = getServoSweepTimerValue(servoShoulder,newPosition,shoulderSweepTime);  // get timer before setting position!
      servoShoulder.setPosition(newPosition);
   }

   public static void setLiftShoulderServo(double newPosition) {
      if (servoLiftShoulderDisabled) {
         servoLiftShoulderDisabled = false;
         parts.robot.enableServo(servoLiftShoulder);
      }
      if (isServoAtPosition(servoLiftShoulder, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerLiftShoulder = getServoSweepTimerValue(servoLiftShoulder,newPosition,liftShoulderSweepTime);  // get timer before setting position!
      servoLiftShoulder.setPosition(newPosition);
   }
   public static void setLiftPinchServo(double newPosition) {
      if (servoLiftPinchDisabled) {
         servoLiftPinchDisabled = false;
         parts.robot.enableServo(servoLiftPinch);
      }
      if (isServoAtPosition(servoLiftPinch, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerLiftPinch = getServoSweepTimerValue(servoLiftPinch,newPosition,liftPinchSweepTime);  // get timer before setting position!
      servoLiftPinch.setPosition(newPosition);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //       Status Responders
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static boolean isWristDone() {return System.currentTimeMillis() >= timerWrist;}
   public static boolean isPinchDone() {return System.currentTimeMillis() >= timerPinch;}
   public static boolean isShoulderDone() {return System.currentTimeMillis() >= timerShoulder;}
   public static boolean isRotatorDone() {return System.currentTimeMillis() >= timerRotator;}
   public static boolean isLiftShoulderDone() {return System.currentTimeMillis() >= timerLiftShoulder;}
   public static boolean isLiftPinchDone() {return System.currentTimeMillis() >= timerLiftPinch;}

   public static boolean isSlideInTolerance(int pos) {return Math.abs(motorSlide.getCurrentPosition() - pos) < toleranceSlide;}
   public static boolean isSlideInTolerance() {return isSlideInTolerance(slideTargetPosition);}
   public static boolean isSlideInsidePit() {return (motorSlide.getCurrentPosition() > positionSlidePitMin);}
   public static boolean isLiftInTolerance(int pos) {return Math.abs(motorLift.getCurrentPosition() - pos) < toleranceLift;}
   public static boolean isLiftInTolerance() {return isLiftInTolerance(liftTargetPosition);}

   public static boolean isSamplingInProcess() {
//      // ignore if slide retracted
//      if (motorSlide.getCurrentPosition() < positionSlidePitMin) return false;
//      // ignore if servo isn't lowered to an intake position
//      if (shoulderNominalPosition >= shoulderSafeIn) return false;
//      return true;
      return motorSlide.getCurrentPosition()>=positionSlidePitMin && shoulderNominalPosition<shoulderSafeIn;
   }

   // special case for state machine
   public static boolean isLiftShoulderAtTransfer() {return isServoAtPosition(servoLiftShoulder, liftShoulderTransfer, timerLiftShoulder);}

   public static boolean isServoAtPosition(Servo servo, double comparePosition, long servoTimer) {
      return isServoAtPosition(servo.getPosition(), comparePosition) && System.currentTimeMillis() >= servoTimer;
   }
   public static boolean isServoAtPosition(Servo servo, double comparePosition) {
      return isServoAtPosition(servo.getPosition(), comparePosition);
   }
   public static boolean isServoAtPosition(double servoPosition, double comparePosition) {
      return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
   }

   public static double getServoSweepChange(Servo servo, double newPosition) {
      return Math.abs(servo.getPosition()-newPosition);
   }

   public static long getServoSweepTimerValue(Servo servo, double newPosition, int sweepTime) {
      return System.currentTimeMillis() + (long)(getServoSweepChange(servo, newPosition) * (long)sweepTime);
   }

   public void initServos () {
      servoRotator.setDirection(Servo.Direction.FORWARD);
      servoShoulder.setDirection(Servo.Direction.FORWARD);
      servoWrist.setDirection(Servo.Direction.FORWARD);
      servoPinch.setDirection(Servo.Direction.FORWARD);
      servoLiftShoulder.setDirection(Servo.Direction.FORWARD);
      servoLiftPinch.setDirection(Servo.Direction.FORWARD);

      servoRotator.setPosition(rotatorCenter);
      servoShoulder.setPosition(shoulderBalanced);
      servoWrist.setPosition(wristCenter);
      servoPinch.setPosition(pinchFullOpen);
      servoLiftPinch.setPosition(liftPinchSafe);
      servoLiftShoulder.setPosition(liftShoulderSafe);
   }

   public void initMotors () {
      stopMotors();
      motorSlide.setDirection(DcMotorEx.Direction.FORWARD);
      motorLift.setDirection(DcMotorEx.Direction.REVERSE);
      motorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      slideTargetPosition = 0;
      liftTargetPosition = 0;
      motorSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      motorLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //  Actuators: Motors and Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void stopMotors() {
      stopSlide();
      stopLift();
   }

   public static void stopSlide() { motorSlide.setPower(0); }
   public static void stopLift() { motorLift.setPower(0); }

   public static void setSlidePower (double m0) { motorSlide.setPower(m0); }
   public static void setLiftPower (double m1) { motorLift.setPower(m1); }

   public static void stopSlideAndHold() { setSlidePosition(motorSlide.getCurrentPosition(),0.5); }
   public static void stopLiftAndHold() { setLiftPosition(motorLift.getCurrentPosition(),0.5); }

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
   public void manualSlideControl(double slideSpeed) {
      if (slideSpeed == 0 && !isSlideUnderManualControl) return;
      if (slideSpeed != 0) {
         int currentPos = motorSlide.getCurrentPosition();
         if (slideSpeed > 0 && currentPos > positionSlideMax) slideSpeed = 0;           //enforce upper limits
         if (slideSpeed < 0 && currentPos < positionSlidePitMin && !slideOverride) slideSpeed = 0;   //positionSlideMin          //enforce lower limits
         if (slideSpeed < 0 && slideLimit == 1) slideSpeed = 0;
//         if (slideSpeed < 0 && isLimitSwitchPressed()) slideSpeed = 0;
      }
      if (slideSpeed == 0) {  // when it drops out of manual control, hold
         isSlideUnderManualControl = false;
         stopSlide();
         isSlideHoldDeferred = true;
         return;
      }
      if (!isSlideUnderManualControl) {
         isSlideUnderManualControl = true;
         cancelStateMachines();
         stopSlide();
         motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         return;  // we'll set the speed next time... jerky if you do it immediately after changing mode
      }
      setSlidePower(slideSpeed);
   }

   public void manualLiftControl(double liftSpeed) {
      if (liftSpeed == 0 && !isLiftUnderManualControl) return;
      if (liftSpeed != 0) {
         int currentPos = motorLift.getCurrentPosition();
         if (liftSpeed > 0 && currentPos > positionLiftMax) liftSpeed = 0;           //enforce upper limits
         if (liftSpeed < 0 && currentPos < positionLiftMin && !slideOverride) liftSpeed = 0;   //positionSlideMin          //enforce lower limits
         if (liftSpeed < 0 && liftLimit == 1) liftSpeed = 0;
//         if (liftSpeed < 0 && isLimitSwitchPressed()) liftSpeed = 0;
      }
      if (liftSpeed == 0) {  // when it drops out of manual control, hold
         isLiftUnderManualControl = false;
         stopLift();
         isLiftHoldDeferred = true;
         return;
      }
      if (!isLiftUnderManualControl) {
         isLiftUnderManualControl = true;
         cancelStateMachines();
         stopLift();
         motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         return;  // we'll set the speed next time... jerky if you do it immediately after changing mode
      }
      setLiftPower(liftSpeed);
   }

   public void manualRotatorControl(double rotatorSpeed) {
      if (!isSamplingInProcess()) return;
      rotatorSpeed *= .008;
      double newRotatorPosition = servoRotator.getPosition() - rotatorSpeed;
      double newWristPosition = servoWrist.getPosition() - rotatorSpeed;
      if (newRotatorPosition<rotator45Right) return;
      if (newRotatorPosition>rotator45Left) return;
      servoRotator.setPosition(newRotatorPosition);
      if (newWristPosition<wrist90Left) return;
      if (newWristPosition>wrist90Right) return;
      servoWrist.setPosition(newWristPosition);
   }

   public void manualWristControl(double wristAngle) {
      if (!isSamplingInProcess()) return;
      double rotatorAngle = Functions.interpolate(servoRotator.getPosition(),rotator45Left,rotator45Right,45,-45);
      double newAngle = wristAngle - rotatorAngle;
      double newPosition = Functions.interpolate(newAngle, 90, -90, wrist90Left, wrist90Right);
      if (newPosition<wrist90Left) return;
      if (newPosition>wrist90Right) return;
      servoWrist.setPosition(newPosition);
   }

   public void delayedActions() {
//      if (grabOpenRequested!=-1) setGrabServo(grabOpenRequested);  // simple handling of delayed servo opening
      if (isSlideHoldDeferred) {
         stopSlideAndHold();
         isSlideHoldDeferred = false;
      }
      if (isLiftHoldDeferred) {
         stopLiftAndHold();
         isLiftHoldDeferred = false;
      }
   }

   public static void setSlidePosition(int goTo) {
      setSlidePosition(goTo, 1);
   }
   public static void setSlidePosition(int goTo, double pwr) {
      // todo: limits ignored if homing
      if (goTo < positionSlideMin || goTo > positionSlideMax) {  // something very wrong so bail
         stopSlide();
         return;
      }
      slideTargetPosition = goTo;
      stopSlide();   // todo: don't do this if wanting to move both at the same time
      motorSlide.setTargetPosition(slideTargetPosition);
      motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      setSlidePower(pwr);
   }

   public static void setLiftPosition(int goTo) {
      setLiftPosition(goTo, 1);
   }
   public static void setLiftPosition(int goTo, double pwr) {
      // todo: limits ignored if homing
      if (goTo < positionLiftMin || goTo > positionLiftMax) {  // something very wrong so bail
         stopLift();
         return;
      }
      liftTargetPosition = goTo;
      stopLift();   // todo: don't do this if wanting to move both at the same time
      motorLift.setTargetPosition(liftTargetPosition);
      motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      setLiftPower(pwr);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Actions
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   public static void action(IntakeActions action) {
      switch (action) {
         case AUTO_EXTEND_TO_GRAB:
            smGoFish.start();
            break;
         case AUTO_RETRACT:
            smRetract.start();
            break;
         case AUTO_GRAB:
            break;
         case AUTO_GRAB_AND_RETRACT:
            smGrabAndRetract.start();
            break;
         case AUTO_GRAB_AND_INSPECT:
            smGrabAndInspect.start();
            break;
         case AUTO_HOME:
            break;
         case AUTO_MAKE_SPACE:
            smMakeSpace.start();
            break;
         case AUTO_TRANSFER:
            smTransfer.start();
            break;
         case SAFE_IN:   // no checks for interference here; would need a state machine
            setRotatorServo(rotatorCenter);
            setShoulderServo(shoulderSafeIn);
            setPinchServo(pinchFullOpen);     // might want to return a sample, though?
            setWristServo(wristCenter);
            break;
         case SAFE_OUT:
            setRotatorServo(rotatorCenter);
            setShoulderServo(shoulderSafeOut);
            setWristServo(wristCenter);
            break;
         case GRAB_HOVER:
            setShoulderServo(shoulderHover);
            setPinchServo(pinchClearsSamples);
            break;
         case GRAB_OPEN:
            setPinchServo(pinchSlightOpen);
            break;
         case GRAB_WIDEOPEN:
            setPinchServo(pinchFullOpen);
            break;
         case GRAB_CLOSE:
            setShoulderServo(shoulderCapture);  //maybe?
            setPinchServo(pinchClosed);
            break;
         case SHOULDER_CAPTURE:
            setShoulderServo(shoulderCapture);
            break;
         case SHOULDER_DRAG:
            setShoulderServo(shoulderDrag);
            break;
         case SHOULDER_PUSH:
            setShoulderServo(shoulderPush);
            setPinchServo(pinchFullOpen);
            break;
         case SHOULDER_ALLBACK:
            setShoulderServo(shoulderFullBack);
            setRotatorServo(rotatorTransfer);
            setWristServo(wristTransfer);
            break;
         case GRAB_LOOSE:
            setPinchServo(pinchLoose);
            break;
         case DROP_SAMPLE:
            setLiftPinchServo(liftPinchSafe);
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
      AUTO_TRANSFER,
      AUTO_GRAB_AND_INSPECT,
      SAFE_IN,
      SAFE_OUT,
      GRAB_HOVER,
      GRAB_OPEN,
      GRAB_WIDEOPEN,
      GRAB_CLOSE,
      GRAB_LOOSE,
      SHOULDER_CAPTURE,
      SHOULDER_DRAG,
      SHOULDER_ALLBACK,
      SHOULDER_PUSH,
      DROP_SAMPLE,
      CANCEL
   }
}