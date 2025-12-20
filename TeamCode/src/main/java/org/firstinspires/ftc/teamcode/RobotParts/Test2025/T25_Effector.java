package org.firstinspires.ftc.teamcode.RobotParts.Test2025;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smAutoIntake;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smDeposit;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smPrepareDeposit;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smSafePark;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smSpecimenHang;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smStartFishing;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smTransfer;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class T25_Effector implements PartsInterface {

   /*
   Notes to self.

   Actuators:

      Servo: Hood Angle
      Servo: Turret Direction
      Motor: Launch Spinner

      Servo x3: Kickers
      Motor: Intake Spinner

   Sensors?

      Color Sensor V2
      Color Sensor V3
      Break Beam??

   Hood angle to be varied by distance to target area
   Launch spinner to be varied by distance to target area
   Turret direction to be vector angle from robot position to target
      (target direction - (minus) robot angle)
      could use a single vector? TargetX - RobotX, Target Y - Robot Y, Target R - Robot R
         (where Target R is first calculated from the Robot XY position to the target XY position)


   Is there enough latency that we need to calculate based on predicted future position of robot based on velocity vector?
   Robot rotation might be an even bigger issue!

   */


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


   static final double hoodNeutral              = 0.5;
   static final double hoodNearest              = 0.4;
   static final double hoodMiddle               = 0.5;
   static final double hoodFar                  = 0.6;
   static final int hoodSweepTime               = 1500;   // spec is 1250

   static final double turretNeutral            = 0.5;
   static final double turretZeroOffset         = 0.0;
   static final double turretRangeDegrees       = 400.00;
   static final int turretSweepTime             = 1500;   // spec is 1250

   static final double kick1Docked              = 0.5;
   static final double kick1Launch              = 0.8;
   static final int kick1SweepTime              = 1500;   // spec is 1250

   static final double kick2Docked              = 0.5;
   static final double kick2Launch              = 0.8;
   static final int kick2SweepTime              = 1500;   // spec is 1250

   static final double kick3Docked              = 0.5;
   static final double kick3Launch              = 0.8;
   static final int kick3SweepTime              = 1500;   // spec is 1250


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
   private static double spinRPMset;
   private static final double spinMultiplier = 60.0 / 28.0 * 1.0;  // seconds / ticksPerRev * gearRatio;

//   private static ServoSSR servoSpinner;
//   private static ServoSSR servoSpintake;
//   private static ServoSSR servoChute;
//   private static ServoSSR servoPinch;

   private static ServoSSR servoHood;
   private static ServoSSR servoTurret;
   private static ServoSSR servoKick1;
   private static ServoSSR servoKick2;
   private static ServoSSR servoKick3;

   private static DcMotorEx motorSpinner;
   private static DcMotorEx motorIntake;
//   private static DcMotorEx motorSlide;
//   private static DcMotorEx motorLift;
//   private static DcMotorEx motorHang;
//   public static DigitalChannel slideLimitSwitchNO = null;
//   public static DigitalChannel slideLimitSwitchNC = null;
//   public static DigitalChannel liftLimitSwitchNO = null;
//   public static DigitalChannel liftLimitSwitchNC = null;
   public static NormalizedColorSensor sensorColor = null;
   private static byte slideLimit = -1;
   private static byte liftLimit = -1;
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
   public T25_Effector(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
//      servoSpinner = new ServoSSR(parts.robotV2.getServoByName("servo0"));
//      servoSpintake = new ServoSSR(parts.robotV2.getServoByName("servo2"));
//      servoChute = new ServoSSR(parts.robotV2.getServoByName("servo4"));
//      servoPinch = new ServoSSR(parts.robotV2.getServoByName("servo1"));

      servoHood = new ServoSSR(parts.robotV2.getServoByName("servo0"));
      servoTurret = new ServoSSR(parts.robotV2.getServoByName("servo1"));
      servoKick1 = new ServoSSR(parts.robotV2.getServoByName("servo2"));
      servoKick2 = new ServoSSR(parts.robotV2.getServoByName("servo3"));
      servoKick3 = new ServoSSR(parts.robotV2.getServoByName("servo4"));

      motorSpinner = parts.robotV2.getMotorByName("motor3B");
      motorIntake = parts.robotV2.getMotorByName("motor2B");
//      motorSlide = parts.robotV2.getMotorByName("motor0B");
//      motorLift = parts.robotV2.getMotorByName("motor1B");
//      motorHang = parts.robotV2.getMotorByName("motor2B");
//      slideLimitSwitchNO = parts.robotV2.getDigitalByName("digital1");
//      slideLimitSwitchNC = parts.robotV2.getDigitalByName("digital0");
//      liftLimitSwitchNO = parts.robotV2.getDigitalByName("digital3");
//      liftLimitSwitchNC = parts.robotV2.getDigitalByName("digital2");
//      sensorColor = parts.opMode.hardwareMap.get(NormalizedColorSensor.class, "color");
      initServos();
      initMotors();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   @SuppressLint("DefaultLocale")
   public void runLoop() {
//      updateLimits();
//      delayedActions();
//
//      smSafePark.stateMachine();
//      smDeposit.stateMachine();
//      smPrepareDeposit.stateMachine();
//      smTransfer.stateMachine();
//      smStartFishing.stateMachine();
//      smAutoIntake.stateMachine();
//      smSpecimenHang.stateMachine();
//
//      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF,
//              "States: " +
//                      "PK: " + String.format("%02d", smSafePark.getState()) +
//                      ", GF: " + String.format("%02d", smStartFishing.getState()) +
//                      ", AI: " + String.format("%02d", smAutoIntake.getState()) +
//                      ", TR: " + String.format("%02d", smTransfer.getState()) +
//                      ", PD: " + String.format("%02d", smPrepareDeposit.getState()) +
//                      ", DE: " + String.format("%02d", smDeposit.getState()) +
//                      "");
//      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Last Hue", lastHue);
//      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Last Type", lastType);
   }

   public void stop() {
   }

   public static void eStop() {
      stopStateMachines();
      stopMotors();
      disableServos();
   }

   public static void cancelStateMachines() {
//      smSafePark.mildStop();
//      smDeposit.mildStop();
//      smPrepareDeposit.mildStop();
//      smTransfer.mildStop();
//      smStartFishing.mildStop();
//      //smAutoIntake.mildStop();   // want this to run when slide is operated
//      smSpecimenHang.mildStop();
   }

   public static void stopStateMachines() {
//      smSafePark.stop();
//      smDeposit.stop();
//      smPrepareDeposit.stop();
//      smTransfer.stop();
//      smStartFishing.stop();
//      smAutoIntake.stop();
//      smSpecimenHang.stop();
   }

//   public static void updateLimits() {
//      byte slideTemp;
//      byte liftTemp;
//      // Figure out current state
//      if (slideLimitSwitchNO.getState() && !slideLimitSwitchNC.getState()) slideTemp=0;
//      else if (slideLimitSwitchNC.getState() && !slideLimitSwitchNO.getState()) slideTemp=1;
//      else slideTemp = -1;
//      if (liftLimitSwitchNO.getState() && !liftLimitSwitchNC.getState()) liftTemp=0;
//      else if (liftLimitSwitchNC.getState() && !liftLimitSwitchNO.getState()) liftTemp=1;
//      else liftTemp = -1;
//      // Figure out if state changed to pressed
//      boolean slideLimitJustPressed = false;
//      boolean liftLimitJustPressed = false;
//      if (slideLimit!=1 && slideTemp==1) slideLimitJustPressed=true;
//      if (liftLimit!=1 && liftTemp==1) liftLimitJustPressed=true;
//      // update state variables
//      slideLimit = slideTemp;
//      liftLimit = liftTemp;
//      // reset encoders?
//      if (slideLimitJustPressed) motorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//      if (liftLimitJustPressed) motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//   }

//   public static void delayedActions() {
////      if (grabOpenRequested!=-1) setGrabServo(grabOpenRequested);  // simple handling of delayed servo opening
//      if (isSlideHoldDeferred) {
//         stopSlideAndHold();
//         isSlideHoldDeferred = false;
//      }
//      if (isLiftHoldDeferred) {
//         stopLiftAndHold();
//         isLiftHoldDeferred = false;
//      }
//   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//   public static void setSpinnerServo(double newPosition) {
//      servoSpinner.setPosition(newPosition);
//   }
//   public static void setSpintakeServo(double newPosition) {
//      servoSpintake.setPosition(newPosition);
//   }
//   public static void setChuteServo(double newPosition) {
//      servoChute.setPosition(newPosition);
//   }
//   public static void setPinchServo(double newPosition) {
//      servoPinch.setPosition(newPosition);
//   }

   public static void disableServos() {
      //SB_Intake.action(IntakeActions.SPINNER_OFF);  // this servo reacts poorly to to the stop?
//      servoSpintake.stop();
//      servoChute.stop();
//      servoPinch.stop();
//      servoSpinner.stop();
      servoHood.stop();
      servoTurret.stop();
      servoKick1.stop();
      servoKick2.stop();
      servoKick3.stop();
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Motors
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void stopMotors() {
//      stopSlide();
//      stopLift();
   }

//   public static void stopSlide() { motorSlide.setPower(0); }
//   public static void stopLift() { motorLift.setPower(0); }

//   public static void setSlidePower (double m0) { motorSlide.setPower(m0); }
//   public static void setLiftPower (double m1) { motorLift.setPower(m1); }

//   public static void stopSlideAndHold() { setSlidePosition(motorSlide.getCurrentPosition(),0.5); }
//   public static void stopLiftAndHold() { setLiftPosition(motorLift.getCurrentPosition(),0.5); }

//   public static void setSlidePosition(int goTo) {
//      setSlidePosition(goTo, 1);
//   }
//   public static void setSlidePosition(int goTo, double pwr) {
//      // todo: limits ignored if homing
//      if (goTo < positionSlideMin || goTo > positionSlideMax) {  // something very wrong so bail
//         stopSlide();
//         return;
//      }
//      slideTargetPosition = goTo;
//      stopSlide();   // todo: don't do this if wanting to move both at the same time
//      motorSlide.setTargetPosition(slideTargetPosition);
//      motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      setSlidePower(pwr);
//   }

//   public static void setLiftPosition(int goTo) {
//      setLiftPosition(goTo, 1);
//   }
//   public static void setLiftPosition(int goTo, double pwr) {
//      // todo: limits ignored if homing
//      if (goTo < positionLiftMin || goTo > positionLiftMax) {  // something very wrong so bail
//         stopLift();
//         return;
//      }
//      liftTargetPosition = goTo;
//      stopLift();   // todo: don't do this if wanting to move both at the same time
//      motorLift.setTargetPosition(liftTargetPosition);
//      motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      setLiftPower(pwr);
//   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //   Init Motors & Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void initServos () {
//      servoSpinner.setDirection(Servo.Direction.FORWARD);
//      servoSpintake.setDirection(Servo.Direction.FORWARD);
//      servoChute.setDirection(Servo.Direction.FORWARD);
//      servoPinch.setDirection(Servo.Direction.FORWARD);
//
//      servoSpinner.setSweepTime(spinnerSweepTime);//  .setFullPwmRange(); <== no good for CR Axon
//      servoSpintake.setSweepTime(spintakeSweepTime);
//      servoChute.setSweepTime(chuteSweepTime);
//      servoPinch.setSweepTime(pinchSweepTime);
//
//      servoSpinner.setPosition(spinnerOff);
//      servoSpintake.setPosition(spintakeParked);
//      servoChute.setPosition(chuteParked);
//      servoPinch.setPosition(pinchFullOpen);

      servoHood.setDirection(Servo.Direction.FORWARD);
      servoTurret.setDirection(Servo.Direction.FORWARD);
      servoKick1.setDirection(Servo.Direction.FORWARD);
      servoKick2.setDirection(Servo.Direction.FORWARD);
      servoKick3.setDirection(Servo.Direction.FORWARD);

      servoHood.setSweepTime(hoodSweepTime);;
      servoTurret.setSweepTime(turretSweepTime);
      servoKick1.setSweepTime(kick1SweepTime);
      servoKick2.setSweepTime(kick2SweepTime);
      servoKick3.setSweepTime(kick3SweepTime);

      servoHood.setPosition(hoodNeutral);
      servoTurret.setPosition(turretNeutral);
      servoKick1.setPosition(kick1Docked);
      servoKick2.setPosition(kick2Docked);
      servoKick3.setPosition(kick3Docked);
   }

   public static void initMotors () {
      stopMotors();

      motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
      motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

      motorSpinner.setDirection(DcMotorEx.Direction.REVERSE);
      motorSpinner.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, spinnerPID);
      motorSpinner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      motorSpinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpinner.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

//      motorSlide.setDirection(DcMotorEx.Direction.FORWARD);
//      motorLift.setDirection(DcMotorEx.Direction.REVERSE);
//      motorHang.setDirection(DcMotorEx.Direction.FORWARD);
//      motorSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//      motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//      motorHang.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//      slideTargetPosition = 0;
//      liftTargetPosition = 0;
//      hangTargetPosition = 0;
//      motorSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//      motorLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//      motorHang.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//      motorSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//      motorLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//      motorHang.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //       Status Responders
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//   public static boolean isSpinnerDone() {return servoSpinner.isDone();}
//   public static boolean isSpintakeDone() {return servoSpintake.isDone();}
//   public static boolean isChuteDone() {return servoChute.isDone();}
//   public static boolean isPinchDone() {return servoPinch.isDone();}

//   public static boolean isSlideInTolerance(int pos) {return Math.abs(motorSlide.getCurrentPosition() - pos) < toleranceSlide;}
//   public static boolean isSlideInTolerance() {return isSlideInTolerance(slideTargetPosition);}
//   public static boolean isSlideInsidePit() {return (motorSlide.getCurrentPosition() > positionSlidePitMin);}
//   public static boolean isLiftInTolerance(int pos) {return Math.abs(motorLift.getCurrentPosition() - pos) < toleranceLift;}
//   public static boolean isLiftInTolerance() {return isLiftInTolerance(liftTargetPosition);}

//   public static boolean isSamplingInProcess() {
////      return motorSlide.getCurrentPosition()>=positionSlidePitMin && shoulderNominalPosition<shoulderSafeIn;
//      return motorSlide.getCurrentPosition()>=positionSlidePitMin && isServoAtPosition(servoSpintake, spintakeFloor, timerSpintake);
//   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //     Manual User Controls
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//   public static void manualSlideControl(double slideSpeed) {
//      if (slideSpeed == 0 && !isSlideUnderManualControl) return;
//      if (slideSpeed != 0) {
//         int currentPos = motorSlide.getCurrentPosition();
//         if (slideSpeed > 0 && currentPos > positionSlideMax) slideSpeed = 0;           //enforce upper limits
//         if (slideSpeed < 0 && currentPos < positionSlidePitMin && !slideOverride) slideSpeed = 0;   //positionSlideMin          //enforce lower limits
//         if (slideSpeed < 0 && slideLimit == 1) slideSpeed = 0;
//      }
//      if (slideSpeed == 0) {  // when it drops out of manual control, hold
//         isSlideUnderManualControl = false;
//         stopSlide();
//         isSlideHoldDeferred = true;
//         return;
//      }
//      if (!isSlideUnderManualControl) {
//         isSlideUnderManualControl = true;
//         cancelStateMachines();
//         stopSlide();
//         motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         return;  // we'll set the speed next time... jerky if you do it immediately after changing mode
//      }
//      setSlidePower(slideSpeed);
//   }

//   public static void manualLiftControl(double liftSpeed) {
//      if (liftSpeed == 0 && !isLiftUnderManualControl) return;
//      if (liftSpeed != 0) {
//         int currentPos = motorLift.getCurrentPosition();
//         if (liftSpeed > 0 && currentPos > positionLiftMax) liftSpeed = 0;           //enforce upper limits
//         if (liftSpeed < 0 && currentPos < positionLiftMin && !slideOverride) liftSpeed = 0;   //positionSlideMin          //enforce lower limits
//         if (liftSpeed < 0 && liftLimit == 1) liftSpeed = 0;
//      }
//      if (liftSpeed == 0) {  // when it drops out of manual control, hold
//         isLiftUnderManualControl = false;
//         stopLift();
//         isLiftHoldDeferred = true;
//         return;
//      }
//      if (!isLiftUnderManualControl) {
//         isLiftUnderManualControl = true;
//         cancelStateMachines();
//         stopLift();
//         motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         return;  // we'll set the speed next time... jerky if you do it immediately after changing mode
//      }
//      setLiftPower(liftSpeed);
//   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Actions
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public enum EffectorActions {
//      AUTO_SAFE_PARK,
//      AUTO_START_SAMPLING,
//      AUTO_TRANSFER,
//      AUTO_INTAKE,
//      AUTO_PREP_DEPOSIT,
//      AUTO_DEPOSIT,
//      SPECIMEN_GRAB_READY,
//      SPECIMEN_GRAB,
//      SPECIMEN_HANG_READY,
//      SPECIMEN_HANG,
//      SPINTAKE_PARK,
//      SPINTAKE_FLOOR,
//      SPINTAKE_ALMOSTFLOOR,
//      SPINTAKE_SAFE,
//      SPINTAKE_VERTICAL,
//      SPINTAKE_BALANCED,
//      CHUTE_PARK,
//      CHUTE_READY,
//      CHUTE_DROP,
//      PINCH_WIDEOPEN,
//      PINCH_READY,
//      PINCH_TIGHT,
//      PINCH_LOOSE,
//      PINCH_VERYLOOSE,
//      SPINNER_IN,
//      SPINNER_OUT,
//      SPINNER_SLOWOUT,
//      SPINNER_OFF,
//      SPINTAKE_DISABLE,
//      SPINNER_DISABLE,
//      CHUTE_DISABLE,
//      PINCH_DISABLE,
//      SLIDE_ZERO,
//      SLIDE_RETRACT,
//      LIFT_ZERO,
//      LIFT_RETRACT,
//      CANCEL
   }

   public static void action(EffectorActions action) {
      switch (action) {
//         case AUTO_START_SAMPLING:
//            smStartFishing.start();
//            break;
//         case AUTO_INTAKE:
//            smAutoIntake.start();
//            break;
//         case AUTO_TRANSFER:
//            smTransfer.start();
//            break;
//         case AUTO_SAFE_PARK:
//            smSafePark.start();
//            break;
//         case AUTO_PREP_DEPOSIT:
//            smPrepareDeposit.start();
//            break;
//         case AUTO_DEPOSIT:
//            smDeposit.start();
//            break;
//
//         case SPECIMEN_GRAB_READY:
//            setPinchServo(pinchFullOpen);
//            setLiftPosition(positionLiftGetSpecimen,1);
//            break;
//         case SPECIMEN_GRAB:
//            setPinchServo(pinchClosed);   // does this need a state machine with a lift up motion? todo: yes, it does!
//            break;
//         case SPECIMEN_HANG_READY:
//            setPinchServo(pinchLoose);
//            setLiftPosition(positionLiftHangReady,1);
//            break;
//         case SPECIMEN_HANG:
//            smSpecimenHang.start();
//            break;
//
//         case SPINTAKE_PARK:
//            setSpintakeServo(spintakeParked);
//            break;
//         case SPINTAKE_FLOOR:
//            setSpintakeServo(spintakeFloor);
//            break;
//         case SPINTAKE_ALMOSTFLOOR:
//            setSpintakeServo(spintakeAlmostFloor);
//            break;
//         case SPINTAKE_SAFE:
//            setSpintakeServo(spintakeSafe);
//            break;
//         case SPINTAKE_BALANCED:
//            setSpintakeServo(spintakeBalanced);
//            break;
//         case SPINTAKE_VERTICAL:
//            setSpintakeServo(spintakeVertical);
//            break;
//         case CHUTE_PARK:
//            setChuteServo(chuteParked);
//            break;
//         case CHUTE_READY:
//            setChuteServo(chuteReady);
//            break;
//         case CHUTE_DROP:
//            setChuteServo(chuteDeposit);
//            break;
//         case PINCH_WIDEOPEN:
//            setPinchServo(pinchFullOpen);
//            break;
//         case PINCH_READY:
//            setPinchServo(pinchReady);
//            break;
//         case PINCH_TIGHT:
//            setPinchServo(pinchClosed);
//            break;
//         case PINCH_LOOSE:
//            setPinchServo(pinchLoose);
//            break;
//         case PINCH_VERYLOOSE:
//            setPinchServo(pinchSuperLoose);
//            break;
//         case SPINNER_IN:
//            setSpinnerServo(spinnerIn);
//            break;
//         case SPINNER_OUT:
//            setSpinnerServo(spinnerOut);
//            break;
//         case SPINNER_SLOWOUT:
//            setSpinnerServo(spinnerSlowOut);
//            break;
//         case SPINNER_OFF:
//            setSpinnerServo(spinnerOff);
//            break;
//
//         case SPINTAKE_DISABLE:
//            servoSpintake.disable();
//            break;
//         case SPINNER_DISABLE:
//            servoSpinner.disable();
//            break;
//         case CHUTE_DISABLE:
//            servoChute.disable();
//            break;
//         case PINCH_DISABLE:
//            servoPinch.disable();
//            break;
//         case SLIDE_ZERO:
//            setSlidePosition(-5000,.25);
//            break;
//         case SLIDE_RETRACT:
//            setSlidePosition(positionSlideMin,1);
//            break;
//         case LIFT_ZERO:
//            setLiftPosition(-5000,.25);
//            break;
//         case LIFT_RETRACT:
//            setLiftPosition(positionLiftMin,1);
//            break;

         default:
            break;
      }
   }

/* example code for angle from UserDrive */
   public Position directionTarget;

   public double targetAngle() {
      if (directionTarget==null || parts.positionMgr.noPosition()) return 0;
      double x = directionTarget.X - parts.positionMgr.robotPosition.X;
      double y = directionTarget.Y - parts.positionMgr.robotPosition.Y;
      return Math.toDegrees(Math.atan2(y,x));
   }

   static double spinnerRPM                      = 3600;
   static double spinnerTolerance                = 150;
   public static PIDFCoefficients spinnerPID           = new PIDFCoefficients(100,0,0,12.4);

   static boolean isSpinnerInTolerance() {
      //return true;  // for testing without spin motor
      return isSpinnerInTolerance(spinRPMset, spinnerTolerance);
   }
   static boolean isSpinnerInTolerance(double targetRPM, double tolerance) {
      return Math.abs(getSpinnerRPM() - targetRPM) <= tolerance;
   }

   static double getSpinnerRPM() {
      return motorSpinner.getVelocity() * spinMultiplier;
   }

   public static void spinnerOn(double rpm) {
      spinRPMset = rpm;
      motorSpinner.setVelocity(rpm / spinMultiplier);
   }
   public static void spinnerOn() {
      spinnerOn(spinnerRPM);
   }
   public static void spinnerOff() {
      motorSpinner.setPower(0);
//      isArmed = false;
   }
}