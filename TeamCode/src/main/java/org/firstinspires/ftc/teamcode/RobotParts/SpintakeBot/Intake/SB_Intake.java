package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.ServoWrapper;

public class SB_Intake implements PartsInterface {

   /* Settings */

   static final double spinnerIn                = 1;
   static final double spinnerOff               = 0.5;
   static final double spinnerOut               = 0;
   static final double spinnerSlowOut           = 0.35;  //todo: finalize number
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
   static final int positionLiftMax             = 3000;  //4200; //4350;
   static final int positionLiftGetSpecimen     = 10;     //todo: finalize number
   static final int positionLiftHangReady       = 1700;  //2500;   //todo: get number
   static final int positionLiftHangRelease     = 1400;  //2000;   //todo: get number
   static final int positionLiftTransfer        = 10;
   static final int toleranceLift               = 20;

   static final int positionHangMin             = 20;
   static final int positionHangMax             = 9600; //4350;
   static final int positionHangReady           = 3000; //todo: get number
   static final int positionHangFinal           = 1000; //todo: get number
   static final int toleranceHang               = 20;

   public static boolean slideOverride          = false;

   /* Internal use */
   private static ServoWrapper servoSpinner;
   private static ServoWrapper servoSpintake;
   private static ServoWrapper servoChute;
   private static ServoWrapper servoPinch;
   private static DcMotorEx motorSlide;
   private static DcMotorEx motorLift;
   private static DcMotorEx motorHang;
   public static DigitalChannel slideLimitSwitchNO = null;
   public static DigitalChannel slideLimitSwitchNC = null;
   public static DigitalChannel liftLimitSwitchNO = null;
   public static DigitalChannel liftLimitSwitchNC = null;
   public static NormalizedColorSensor sensorColor = null;
   private static byte slideLimit = -1;
   private static byte liftLimit = -1;
//   private static boolean servoSpinnerDisabled = false;
//   private static boolean servoSpintakeDisabled = false;
//   private static boolean servoChuteDisabled = false;
//   private static boolean servoPinchDisabled = false;
//   private static long timerSpinner = System.currentTimeMillis();
//   private static long timerSpintake = System.currentTimeMillis();
//   private static long timerChute = System.currentTimeMillis();
//   private static long timerPinch = System.currentTimeMillis();
   private static int slideTargetPosition;
   private static int liftTargetPosition;
   private static int hangTargetPosition;
   static boolean isSlideUnderManualControl = false;
   static boolean isLiftUnderManualControl = false;
   static boolean isSlideHomed = false;
   static boolean isLiftHomed = false;
   static boolean isSlideHoldDeferred = false;
   static boolean isLiftHoldDeferred = false;

   public static boolean isBlueLegal = false;
   public static boolean isRedLegal = true;
   public static boolean isYellowLegal = true;
   public static int lastHue = 0;
   public static int lastType = -2;

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
      servoSpinner = (ServoWrapper)parts.robot.servo0;
      servoSpintake = (ServoWrapper)parts.robot.servo2;
      servoChute = (ServoWrapper)parts.robot.servo4;
      servoPinch = (ServoWrapper)parts.robot.servo1;
      motorSlide = parts.robot.motor0B;
      motorLift = parts.robot.motor1B;
      motorHang = parts.robot.motor2B;
      slideLimitSwitchNO = parts.robot.digital1;
      slideLimitSwitchNC = parts.robot.digital0;
      liftLimitSwitchNO = parts.robot.digital3;
      liftLimitSwitchNC = parts.robot.digital2;
      sensorColor = parts.opMode.hardwareMap.get(NormalizedColorSensor.class, "color");
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

      smSafePark.stateMachine();
      smDeposit.stateMachine();
      smPrepareDeposit.stateMachine();
      smTransfer.stateMachine();
      smStartFishing.stateMachine();
      smAutoIntake.stateMachine();
      smSpecimenHang.stateMachine();

      TelemetryMgr.message(TelemetryMgr.Category.SB_INTAKE,
              "States: " +
                      "PK: " + String.format("%02d", smSafePark.getState()) +
                      ", GF: " + String.format("%02d", smStartFishing.getState()) +
                      ", AI: " + String.format("%02d", smAutoIntake.getState()) +
                      ", TR: " + String.format("%02d", smTransfer.getState()) +
                      ", PD: " + String.format("%02d", smPrepareDeposit.getState()) +
                      ", DE: " + String.format("%02d", smDeposit.getState()) +
                      "");
      TelemetryMgr.message(TelemetryMgr.Category.SB_INTAKE, "Last Hue", lastHue);
      TelemetryMgr.message(TelemetryMgr.Category.SB_INTAKE, "Last Type", lastType);
   }

   public void stop() {
   }

   public static void eStop() {
      stopStateMachines();
      stopMotors();
      disableServos();
   }

   public static void cancelStateMachines() {
      smSafePark.mildStop();
      smDeposit.mildStop();
      smPrepareDeposit.mildStop();
      smTransfer.mildStop();
      smStartFishing.mildStop();
      //smAutoIntake.mildStop();   // want this to run when slide is operated
      smSpecimenHang.mildStop();
   }

   public static void stopStateMachines() {
      smSafePark.stop();
      smDeposit.stop();
      smPrepareDeposit.stop();
      smTransfer.stop();
      smStartFishing.stop();
      smAutoIntake.stop();
      smSpecimenHang.stop();
   }

   public static void updateLimits() {
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

   public static void delayedActions() {
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
//      if (servoSpinnerDisabled) {
//         servoSpinnerDisabled = false;
//         parts.robot.enableServo(servoSpinner);
//      }
//      if (isServoAtPosition(servoSpinner, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
//      timerSpinner = getServoSweepTimerValue(servoSpinner,newPosition,spinnerSweepTime);  // get timer before setting position!
      servoSpinner.setPosition(newPosition);
   }
   public static void setSpintakeServo(double newPosition) {
//      if (servoSpintakeDisabled) {
//         servoSpintakeDisabled = false;
//         parts.robot.enableServo(servoSpintake);
//      }
//      if (isServoAtPosition(servoSpintake, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
//      timerSpintake = getServoSweepTimerValue(servoSpintake,newPosition,spintakeSweepTime);  // get timer before setting position!
      servoSpintake.setPosition(newPosition);
   }
   public static void setChuteServo(double newPosition) {
//      if (servoChuteDisabled) {
//         servoChuteDisabled = false;
//         parts.robot.enableServo(servoChute);
//      }
//      if (isServoAtPosition(servoChute, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
//      timerChute = getServoSweepTimerValue(servoChute,newPosition,chuteSweepTime);  // get timer before setting position!
      servoChute.setPosition(newPosition);
   }
   public static void setPinchServo(double newPosition) {
//      if (servoPinchDisabled) {
//         servoPinchDisabled = false;
//         parts.robot.enableServo(servoPinch);
//      }
//      if (isServoAtPosition(servoPinch, newPosition)) return;  // has already been set (but not necessarily done moving), no need to update timer
//      timerPinch = getServoSweepTimerValue(servoPinch,newPosition,pinchSweepTime);  // get timer before setting position!
      servoPinch.setPosition(newPosition);
   }
//   public static double getServoSweepChange(Servo servo, double newPosition) {
//      return Math.abs(servo.getPosition()-newPosition);
//   }
//
//   public static long getServoSweepTimerValue(Servo servo, double newPosition, int sweepTime) {
//      return System.currentTimeMillis() + (long)(getServoSweepChange(servo, newPosition) * (long)sweepTime);
//   }

   public static void disableServos() {
      servoSpinner.eStop();
      servoSpintake.eStop();
      servoChute.eStop();
      servoPinch.eStop();
//      parts.robot.disableServo(servoSpinner);
//      parts.robot.disableServo(servoSpintake);
//      parts.robot.disableServo(servoChute);
//      parts.robot.disableServo(servoPinch);
//      servoSpinnerDisabled = true;
//      servoSpintakeDisabled = true;
//      servoChuteDisabled = true;
//      servoPinchDisabled = true;
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

   public static void initServos () {
      servoSpinner.setDirection(Servo.Direction.FORWARD);
      servoSpintake.setDirection(Servo.Direction.FORWARD);
      servoChute.setDirection(Servo.Direction.FORWARD);
      servoPinch.setDirection(Servo.Direction.FORWARD);

      servoSpinner.setSweepTime(spinnerSweepTime).setFullPwmRange();
      servoSpintake.setSweepTime(spintakeSweepTime);
      servoChute.setSweepTime(chuteSweepTime);
      servoPinch.setSweepTime(pinchSweepTime);

//      servoSpinner.setSweepTime();
//      servoSpintake.;
//      servoChute.;
//      servoPinch.;

      servoSpinner.setPosition(spinnerOff);
      servoSpintake.setPosition(spintakeParked);
      servoChute.setPosition(chuteParked);
      servoPinch.setPosition(pinchFullOpen);
   }

   public static void initMotors () {
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

//   public static boolean isSpinnerDone() {return System.currentTimeMillis() >= timerSpinner;}
//   public static boolean isSpintakeDone() {return System.currentTimeMillis() >= timerSpintake;}
//   public static boolean isChuteDone() {return System.currentTimeMillis() >= timerChute;}
//   public static boolean isPinchDone() {return System.currentTimeMillis() >= timerPinch;}
   public static boolean isSpinnerDone() {return servoSpinner.isTimerDone();}
   public static boolean isSpintakeDone() {return servoSpintake.isTimerDone();}
   public static boolean isChuteDone() {return servoChute.isTimerDone();}
   public static boolean isPinchDone() {return servoPinch.isTimerDone();}

   public static boolean isSlideInTolerance(int pos) {return Math.abs(motorSlide.getCurrentPosition() - pos) < toleranceSlide;}
   public static boolean isSlideInTolerance() {return isSlideInTolerance(slideTargetPosition);}
   public static boolean isSlideInsidePit() {return (motorSlide.getCurrentPosition() > positionSlidePitMin);}
   public static boolean isLiftInTolerance(int pos) {return Math.abs(motorLift.getCurrentPosition() - pos) < toleranceLift;}
   public static boolean isLiftInTolerance() {return isLiftInTolerance(liftTargetPosition);}

//   public static boolean isSamplingInProcess() {
////      return motorSlide.getCurrentPosition()>=positionSlidePitMin && shoulderNominalPosition<shoulderSafeIn;
//      return motorSlide.getCurrentPosition()>=positionSlidePitMin && isServoAtPosition(servoSpintake, spintakeFloor, timerSpintake);
//   }

//   public static boolean isServoAtPosition(Servo servo, double comparePosition, long servoTimer) {
//      return isServoAtPosition(servo.getPosition(), comparePosition) && System.currentTimeMillis() >= servoTimer;
//   }
//   public static boolean isServoAtPosition(Servo servo, double comparePosition) {
//      return isServoAtPosition(servo.getPosition(), comparePosition);
//   }
//   public static boolean isServoAtPosition(double servoPosition, double comparePosition) {
//      return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
//   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //     Manual User Controls
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void manualSlideControl(double slideSpeed) {
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

   public static void manualLiftControl(double liftSpeed) {
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
      AUTO_SAFE_PARK,
      AUTO_START_SAMPLING,
      AUTO_TRANSFER,
      AUTO_INTAKE,
      AUTO_PREP_DEPOSIT,
      AUTO_DEPOSIT,
      SPECIMEN_GRAB_READY,
      SPECIMEN_GRAB,
      SPECIMEN_HANG_READY,
      SPECIMEN_HANG,
      SPINTAKE_PARK,
      SPINTAKE_FLOOR,
      SPINTAKE_ALMOSTFLOOR,
      SPINTAKE_SAFE,
      SPINTAKE_VERTICAL,
      SPINTAKE_BALANCED,
      CHUTE_PARK,
      CHUTE_READY,
      CHUTE_DROP,
      PINCH_WIDEOPEN,
      PINCH_READY,
      PINCH_TIGHT,
      PINCH_LOOSE,
      PINCH_VERYLOOSE,
      SPINNER_IN,
      SPINNER_OUT,
      SPINNER_SLOWOUT,
      SPINNER_OFF,
      SPINTAKE_DISABLE,
      SPINNER_DISABLE,
      CHUTE_DISABLE,
      PINCH_DISABLE,
      SLIDE_ZERO,
      SLIDE_RETRACT,
      LIFT_ZERO,
      LIFT_RETRACT,
      CANCEL
   }

   public static void action(IntakeActions action) {
      switch (action) {
         case AUTO_START_SAMPLING:
            smStartFishing.start();
            break;
         case AUTO_INTAKE:
            smAutoIntake.start();
            break;
         case AUTO_TRANSFER:
            smTransfer.start();
            break;
         case AUTO_SAFE_PARK:
            smSafePark.start();
            break;
         case AUTO_PREP_DEPOSIT:
            smPrepareDeposit.start();
            break;
         case AUTO_DEPOSIT:
            smDeposit.start();
            break;

         case SPECIMEN_GRAB_READY:
            setPinchServo(pinchFullOpen);
            setLiftPosition(positionLiftGetSpecimen,1);
            break;
         case SPECIMEN_GRAB:
            setPinchServo(pinchClosed);   // does this need a state machine with a lift up motion?
            break;
         case SPECIMEN_HANG_READY:
            setPinchServo(pinchLoose);
            setLiftPosition(positionLiftHangReady,1);
            break;
         case SPECIMEN_HANG:
            smSpecimenHang.start();
            break;

         case SPINTAKE_PARK:
            setSpintakeServo(spintakeParked);
            break;
         case SPINTAKE_FLOOR:
            setSpintakeServo(spintakeFloor);
            break;
         case SPINTAKE_ALMOSTFLOOR:
            setSpintakeServo(spintakeAlmostFloor);
            break;
         case SPINTAKE_SAFE:
            setSpintakeServo(spintakeSafe);
            break;
         case SPINTAKE_BALANCED:
            setSpintakeServo(spintakeBalanced);
            break;
         case SPINTAKE_VERTICAL:
            setSpintakeServo(spintakeVertical);
            break;
         case CHUTE_PARK:
            setChuteServo(chuteParked);
            break;
         case CHUTE_READY:
            setChuteServo(chuteReady);
            break;
         case CHUTE_DROP:
            setChuteServo(chuteDeposit);
            break;
         case PINCH_WIDEOPEN:
            setPinchServo(pinchFullOpen);
            break;
         case PINCH_READY:
            setPinchServo(pinchReady);
            break;
         case PINCH_TIGHT:
            setPinchServo(pinchClosed);
            break;
         case PINCH_LOOSE:
            setPinchServo(pinchLoose);
            break;
         case PINCH_VERYLOOSE:
            setPinchServo(pinchSuperLoose);
            break;
         case SPINNER_IN:
            setSpinnerServo(spinnerIn);
            break;
         case SPINNER_OUT:
            setSpinnerServo(spinnerOut);
            break;
         case SPINNER_SLOWOUT:
            setSpinnerServo(spinnerSlowOut);
            break;
         case SPINNER_OFF:
            setSpinnerServo(spinnerOff);
            break;

         case SPINTAKE_DISABLE:
//            parts.robot.disableServo(servoSpintake);
//            servoSpintakeDisabled = true;
            servoSpintake.disable();
            break;
         case SPINNER_DISABLE:
//            parts.robot.disableServo(servoSpinner);
//            servoSpinnerDisabled = true;
            servoSpinner.disable();
            break;
         case CHUTE_DISABLE:
//            parts.robot.disableServo(servoChute);
//            servoChuteDisabled = true;
            servoChute.disable();
            break;
         case PINCH_DISABLE:
//            parts.robot.disableServo(servoPinch);
//            servoPinchDisabled = true;
            servoPinch.disable();
            break;
         case SLIDE_ZERO:
            setSlidePosition(-5000,.25);
            break;
         case SLIDE_RETRACT:
            setSlidePosition(positionSlideMin,1);
            break;
         case LIFT_ZERO:
            setLiftPosition(-5000,.25);
            break;
         case LIFT_RETRACT:
            setLiftPosition(positionLiftMin,1);
            break;

         default:
            break;
      }
   }
}