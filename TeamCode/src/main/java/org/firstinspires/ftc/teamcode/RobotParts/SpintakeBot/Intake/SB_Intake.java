package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class SB_Intake implements PartsInterface {

   /* Settings */

   static final double spinnerIn                = 1;
   static final double spinnerOff               = 0.5;
   static final double spinnerOut               = 0;
   static final double spinnerSlowOut           = 0.4;  //todo: finalize number
   static final int spinnerSweepTime            = 100;  //probably not relevant

   static final double spintakeFloor            = 0.859;
   static final double spintakeAlmostFloor      = 0.844;
   static final double spintakeSafe             = 0.698;
   static final double spintakeVertical         = 0.527;
   static final double spintakeBalanced         = 0.468;
   static final double spintakeParked           = 0.275;
   static final int spintakeSweepTime           = 1500;   // spec is 1250

   static final double chuteParked              = 0.689;
   static final double chuteReady               = 0.535;
   static final double chuteDeposit             = 0.327;
   static final int chuteSweepTime              = 1500;   // spec is 1250

   static final double pinchFullOpen            = 0.364;
   static final double pinchReady               = 0.407;
   static final double pinchClosed              = 0.589;
   static final double pinchLoose               = 0.563;
   static final double pinchSuperLoose          = 0.545;
   static final int pinchSweepTime              = 1500;   // spec is 1250

   static final int positionSlideMin            = 10;
   static final int positionSlideMax            = 1500;
   static final int positionSlideStartIntake    = 450;   //todo: finalize number
   static final int positionSlidePitMin         = 250;    //todo: finalize number
   static final int toleranceSlide              = 20;

   static final int positionLiftMin             = 10;
   static final int positionLiftMax             = 4200; //4350;
   static final int positionLiftGetSpecimen     = 10;     //todo: finalize number
   static final int positionLiftHangReady       = 2500;   //todo: get number
   static final int positionLiftHangRelease     = 2000;   //todo: get number
   static final int positionLiftTransfer        = 10;
   static final int toleranceLift               = 20;

   static final int positionHangMin             = 20;
   static final int positionHangMax             = 9600; //4350;
   static final int positionHangReady           = 3000; //todo: get number
   static final int positionHangFinal           = 1000; //todo: get number
   static final int toleranceHang               = 20;

   public static boolean slideOverride          = false;

   /* Internal use */
   private static Servo servoSpinner;
   private static Servo servoSpintake;
   private static Servo servoChute;
   private static Servo servoPinch;
   private static DcMotorEx motorSlide;
   private static DcMotorEx motorLift;
   private static DcMotorEx motorHang;
   public static DigitalChannel slideLimitSwitchNO = null;
   public static DigitalChannel slideLimitSwitchNC = null;
   public static DigitalChannel liftLimitSwitchNO = null;
   public static DigitalChannel liftLimitSwitchNC = null;
   private static byte slideLimit = -1;
   private static byte liftLimit = -1;
   private static boolean servoSpinnerDisabled = false;
   private static boolean servoSpintakeDisabled = false;
   private static boolean servoChuteDisabled = false;
   private static boolean servoPinchDisabled = false;
   private static long timerSpinner = System.currentTimeMillis();
   private static long timerSpintake = System.currentTimeMillis();
   private static long timerChute = System.currentTimeMillis();
   private static long timerPinch = System.currentTimeMillis();
   private static int slideTargetPosition;
   private static int liftTargetPosition;
   private static int hangTargetPosition;
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
   public SB_Intake(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      servoSpinner = parts.robot.servo0;
      servoSpintake = parts.robot.servo2;
      servoChute = parts.robot.servo4;
      servoPinch = parts.robot.servo1;
      motorSlide = parts.robot.motor0B;
      motorLift = parts.robot.motor1B;
      motorHang = parts.robot.motor2B;
      slideLimitSwitchNO = parts.robot.digital1;
      slideLimitSwitchNC = parts.robot.digital0;
      liftLimitSwitchNO = parts.robot.digital3;
      liftLimitSwitchNC = parts.robot.digital2;
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

      smStartSampling.stateMachine();
      smGrabAndRetract.stateMachine();
      smMakeSpace.stateMachine();
      smTransfer.stateMachine();
      smGrabAndInspect.stateMachine();
      smRetract.stateMachine();

      TelemetryMgr.message(TelemetryMgr.Category.SB_INTAKE,
              "States: " +
                      "GF: " + String.format("%02d", smStartSampling.getState()) +
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
      cancelStateMachines();
      stopMotors();
      disableServos();
   }

   public void cancelStateMachines() {
      smStartSampling.mildStop();
      smGrabAndRetract.mildStop();
      smMakeSpace.mildStop();
      smTransfer.mildStop();
      smGrabAndInspect.mildStop();
      smRetract.mildStop();
   }

   public void stopStateMachines() {
      smStartSampling.stop();
      smGrabAndRetract.stop();
      smMakeSpace.stop();
      smTransfer.stop();
      smGrabAndInspect.stop();
      smRetract.stop();
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
      boolean slideLimitJustPressed = false;
      boolean liftLimitJustPressed = false;
      if (slideLimit!=1 && slideTemp==1) slideLimitJustPressed=true;
      if (liftLimit!=1 && liftTemp==1) liftLimitJustPressed=true;
      // update state variables
      slideLimit = slideTemp;
      liftLimit = liftTemp;
      // reset encoders?
      if (slideLimitJustPressed) motorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      if (liftLimitJustPressed) motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   ////// Probably should set up a new class to hold servos, enabled state, timer, etc.
   public static void setSpinnerServo(double newPosition) {
      if (servoSpinnerDisabled) {
         servoSpinnerDisabled = false;
         parts.robot.enableServo(servoSpinner);
      }
      if (isServoAtPosition(servoSpinner, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerSpinner = getServoSweepTimerValue(servoSpinner,newPosition,spinnerSweepTime);  // get timer before setting position!
      servoSpinner.setPosition(newPosition);
   }
   public static void setSpintakeServo(double newPosition) {
      if (servoSpintakeDisabled) {
         servoSpintakeDisabled = false;
         parts.robot.enableServo(servoSpintake);
      }
      if (isServoAtPosition(servoSpintake, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerSpintake = getServoSweepTimerValue(servoSpintake,newPosition,spintakeSweepTime);  // get timer before setting position!
      servoSpintake.setPosition(newPosition);
   }
   public static void setChuteServo(double newPosition) {
      if (servoChuteDisabled) {
         servoChuteDisabled = false;
         parts.robot.enableServo(servoChute);
      }
      if (isServoAtPosition(servoChute, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
      timerChute = getServoSweepTimerValue(servoChute,newPosition,chuteSweepTime);  // get timer before setting position!
      servoChute.setPosition(newPosition);
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
   public static double getServoSweepChange(Servo servo, double newPosition) {
      return Math.abs(servo.getPosition()-newPosition);
   }

   public static long getServoSweepTimerValue(Servo servo, double newPosition, int sweepTime) {
      return System.currentTimeMillis() + (long)(getServoSweepChange(servo, newPosition) * (long)sweepTime);
   }

   public static void disableServos() {
      parts.robot.disableServo(servoSpinner);
      parts.robot.disableServo(servoSpintake);
      parts.robot.disableServo(servoChute);
      parts.robot.disableServo(servoPinch);
      servoSpinnerDisabled = true;
      servoSpintakeDisabled = true;
      servoChuteDisabled = true;
      servoPinchDisabled = true;
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Motors
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
   //   Init Motors & Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public void initServos () {
      servoSpinner.setDirection(Servo.Direction.FORWARD);
      servoSpintake.setDirection(Servo.Direction.FORWARD);
      servoChute.setDirection(Servo.Direction.FORWARD);
      servoPinch.setDirection(Servo.Direction.FORWARD);

      servoSpinner.setPosition(spinnerOff);
      servoSpintake.setPosition(spintakeParked);
      servoChute.setPosition(chuteParked);
      servoPinch.setPosition(pinchFullOpen);
   }

   public void initMotors () {
      stopMotors();
      motorSlide.setDirection(DcMotorEx.Direction.FORWARD);
      motorLift.setDirection(DcMotorEx.Direction.REVERSE);
      motorHang.setDirection(DcMotorEx.Direction.FORWARD);
      motorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      motorHang.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      slideTargetPosition = 0;
      liftTargetPosition = 0;
      hangTargetPosition = 0;
      motorSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorHang.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      motorLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
      motorHang.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //       Status Responders
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static boolean isSpinnerDone() {return System.currentTimeMillis() >= timerSpinner;}
   public static boolean isSpintakeDone() {return System.currentTimeMillis() >= timerSpintake;}
   public static boolean isChuteDone() {return System.currentTimeMillis() >= timerChute;}
   public static boolean isPinchDone() {return System.currentTimeMillis() >= timerPinch;}


   public static boolean isSlideInTolerance(int pos) {return Math.abs(motorSlide.getCurrentPosition() - pos) < toleranceSlide;}
   public static boolean isSlideInTolerance() {return isSlideInTolerance(slideTargetPosition);}
   public static boolean isSlideInsidePit() {return (motorSlide.getCurrentPosition() > positionSlidePitMin);}
   public static boolean isLiftInTolerance(int pos) {return Math.abs(motorLift.getCurrentPosition() - pos) < toleranceLift;}
   public static boolean isLiftInTolerance() {return isLiftInTolerance(liftTargetPosition);}

   public static boolean isSamplingInProcess() {
//      return motorSlide.getCurrentPosition()>=positionSlidePitMin && shoulderNominalPosition<shoulderSafeIn;
      return motorSlide.getCurrentPosition()>=positionSlidePitMin && isServoAtPosition(servoSpintake, spintakeFloor, timerSpintake);
   }

//   // special case for state machine
//   public static boolean isLiftShoulderAtTransfer() {return isServoAtPosition(servoLiftShoulder, liftShoulderTransfer, timerLiftShoulder);}

   public static boolean isServoAtPosition(Servo servo, double comparePosition, long servoTimer) {
      return isServoAtPosition(servo.getPosition(), comparePosition) && System.currentTimeMillis() >= servoTimer;
   }
   public static boolean isServoAtPosition(Servo servo, double comparePosition) {
      return isServoAtPosition(servo.getPosition(), comparePosition);
   }
   public static boolean isServoAtPosition(double servoPosition, double comparePosition) {
      return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //     Manual User Controls
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public void manualSlideControl(double slideSpeed) {
      if (slideSpeed == 0 && !isSlideUnderManualControl) return;
      if (slideSpeed != 0) {
         int currentPos = motorSlide.getCurrentPosition();
         if (slideSpeed > 0 && currentPos > positionSlideMax) slideSpeed = 0;           //enforce upper limits
         if (slideSpeed < 0 && currentPos < positionSlidePitMin && !slideOverride) slideSpeed = 0;   //positionSlideMin          //enforce lower limits
         if (slideSpeed < 0 && slideLimit == 1) slideSpeed = 0;
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

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Actions
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public enum IntakeActions {
      AUTO_EXTEND_TO_GRAB,
      AUTO_RETRACT,
//      AUTO_GRAB,
//      AUTO_GRAB_AND_RETRACT,
//      AUTO_HOME,
//      AUTO_MAKE_SPACE,
      AUTO_TRANSFER,
      AUTO_GRAB_AND_INSPECT,
      SAFE_IN,
      SAFE_OUT,
      GRAB_HOVER,
      GRAB_OPEN,
      GRAB_WIDEOPEN,
      GRAB_CLOSE,
//      GRAB_LOOSE,
      SHOULDER_CAPTURE,
      SHOULDER_DRAG,
//      SHOULDER_ALLBACK,
      SHOULDER_PUSH,
      DROP_SAMPLE,
      CANCEL
   }

   public static void action(IntakeActions action) {
      switch (action) {
//         case AUTO_EXTEND_TO_GRAB:
//            smGoFish.start();
//            break;
//         case AUTO_RETRACT:
//            smRetract.start();
//            break;
//         case AUTO_GRAB:
//            break;
//         case AUTO_GRAB_AND_RETRACT:
//            smGrabAndRetract.start();
//            break;
//         case AUTO_GRAB_AND_INSPECT:
//            smGrabAndInspect.start();
//            break;
//         case AUTO_HOME:
//            break;
//         case AUTO_MAKE_SPACE:
//            smMakeSpace.start();
//            break;
//         case AUTO_TRANSFER:
//            smTransfer.start();
//            break;
//         case SAFE_IN:   // no checks for interference here; would need a state machine
//            setRotatorServo(rotatorCenter);
//            setShoulderServo(shoulderSafeIn);
//            setPinchServo(pinchFullOpen);     // might want to return a sample, though?
//            setWristServo(wristCenter);
//            break;
//         case SAFE_OUT:
//            setRotatorServo(rotatorCenter);
//            setShoulderServo(shoulderSafeOut);
//            setWristServo(wristCenter);
//            break;
//         case GRAB_HOVER:
//            setShoulderServo(shoulderHover);
//            setPinchServo(pinchClearsSamples);
//            break;
//         case GRAB_OPEN:
//            setPinchServo(pinchSlightOpen);
//            break;
//         case GRAB_WIDEOPEN:
//            setPinchServo(pinchFullOpen);
//            break;
//         case GRAB_CLOSE:
//            setShoulderServo(shoulderCapture);  //maybe?
//            setPinchServo(pinchClosed);
//            break;
//         case SHOULDER_CAPTURE:
//            setShoulderServo(shoulderCapture);
//            break;
//         case SHOULDER_DRAG:
//            setShoulderServo(shoulderDrag);
//            break;
//         case SHOULDER_PUSH:
//            setShoulderServo(shoulderPush);
//            setPinchServo(pinchFullOpen);
//            break;
//         case SHOULDER_ALLBACK:
//            setShoulderServo(shoulderFullBack);
//            setRotatorServo(rotatorTransfer);
//            setWristServo(wristTransfer);
//            break;
//         case GRAB_LOOSE:
//            setPinchServo(pinchLoose);
//            break;
//         case DROP_SAMPLE:
//            setLiftPinchServo(liftPinchSafe);
//            break;
//         case CANCEL:
//            break;
         default:
            break;
      }
   }
}