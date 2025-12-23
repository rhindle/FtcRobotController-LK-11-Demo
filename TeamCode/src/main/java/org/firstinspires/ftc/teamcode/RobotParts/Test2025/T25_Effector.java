package org.firstinspires.ftc.teamcode.RobotParts.Test2025;

import android.annotation.SuppressLint;

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
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smStartFishing;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.smTransfer;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Vector2D;
import org.firstinspires.ftc.teamcode.Tools.Functions;
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

   static final double spinNear                 = 2500;
   static final double spinMiddle               = 3500;
   static final double spinFar                  = 4300;
   static double spinnerRPM                     = 0;  //move this
   static double spinnerTolerance               = 100;
   public static PIDFCoefficients spinnerPID    = new PIDFCoefficients(100,0,0,12.4);

   static final Position targetRed              = new Position(-70.5, 70.5, 0.0);
   static final Position targetBlue             = new Position(-70.5, -70.5, 0.0);

   /* Internal use */
   private static double spinRPMset;
   private static final double spinMultiplier = 60.0 / 28.0 * 1.0;  // seconds / ticksPerRev * gearRatio;

   private static final double nearTest      = 35;
   private static final double farTest       = 137;

   private static ServoSSR servoHood;
   private static ServoSSR servoTurret;
   private static ServoSSR servoKick1;
   private static ServoSSR servoKick2;
   private static ServoSSR servoKick3;
   private static ServoSSR servoLED;

   private static DcMotorEx motorSpinner;
   private static DcMotorEx motorIntake;

   public static NormalizedColorSensor sensorColor = null;

   public static Vector2D targetVector;
   public static boolean turretArmed = false;
   public static boolean spinnerArmed = false;
   public static boolean intakeArmed = false;

   public static boolean turretOutOfRange = false;

   /* State machines */


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

      servoHood = new ServoSSR(parts.robotV2.getServoByName("servo0"));
      servoTurret = new ServoSSR(parts.robotV2.getServoByName("servo1"));
      servoKick1 = new ServoSSR(parts.robotV2.getServoByName("servo2"));
      servoKick2 = new ServoSSR(parts.robotV2.getServoByName("servo3"));
      servoKick3 = new ServoSSR(parts.robotV2.getServoByName("servo4"));
      servoLED = new ServoSSR(parts.robotV2.getServoByName("servo5"));

      motorSpinner = parts.robotV2.getMotorByName("motor3B");
      motorIntake = parts.robotV2.getMotorByName("motor2B");

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

      // update internal target vector
      targetVector = getTargetVector(targetRed);

      // update turret position
      if (turretArmed) {
         double turretAngle = getTurretAngle(targetRed);
         double turretPos = getTurretValueFromAngle(turretAngle);
         servoTurret.setPosition(turretPos);
         turretOutOfRange = turretPos == 0 || turretPos == 1;
      }

      // update hood
      if (turretArmed) {
         double hoodPos = getHoodValueFromDistance(targetVector.distance);
         servoHood.setPosition(hoodPos);
      }

      // update spinner
      if (spinnerArmed) {
         double spinnerRPM = getSpinnerRPMfromDistance(targetVector.distance);
         spinnerOn(spinnerRPM);
      }

      // update LED
      if (!turretArmed || !spinnerArmed) {
         servoLED.setPosition(rgbIndicatorColor.Off.color);
      }
      else if (turretOutOfRange) {
         servoLED.setPosition(rgbIndicatorColor.Red.color);
      }
      else if (isSpinnerInTolerance() && servoHood.isDone() && servoTurret.isDone()) {
         servoLED.setPosition(rgbIndicatorColor.Green.color);
      }
      else {
         servoLED.setPosition(rgbIndicatorColor.Yellow.color);
      }


//      updateLimits();
//      delayedActions();

      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Target Vector: ", targetVector.toString());

   }

   public void stop() {
   }

   public static void eStop() {
      stopStateMachines();
      stopMotors();
      disableServos();
      intakeArmed = false;
      spinnerArmed = false;
      turretArmed = false;
   }

   public static void cancelStateMachines() {

   }

   public static void stopStateMachines() {

   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void disableServos() {
      servoHood.stop();
      servoTurret.stop();
      servoKick1.stop();
      servoKick2.stop();
      servoKick3.stop();
      servoLED.stop();
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //            Motors
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void stopMotors() {
      stopSpinner();
      stopIntake();
      spinnerArmed = false;
      intakeArmed = false;
   }

   public static void stopSpinner() { motorSpinner.setPower(0); }
   public static void stopIntake() { motorIntake.setPower(0); }


   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //   Init Motors & Servos
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   public static void initServos () {

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
      motorSpinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpinner.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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


   public void armTurret(boolean arm) {
      turretArmed = arm;
//      if (!turretArmed) {
//         servoHood.setPosition(hoodNeutral);
//         servoTurret.setPosition(turretNeutral+turretZeroOffset);
//      }
   }
   public void armSpinner(boolean arm) {
      spinnerArmed = arm;
      if (!spinnerArmed) {
         stopSpinner();
      }
   }

   public double getTurretValueFromAngle(double angle) {
      // turret geared so 300° range = 400°
      // 0.5 = 0, 0.1 = 180, 0.9 = -180 ???
      // let's calculate with an offset of -0.5 to make calculation easier.
      angle = Functions.normalizeAngle(angle);
      double rangeFraction = 0.9;
      // 180*.9
      double q = (angle / 180) * rangeFraction;
      double qq = q/2 + 0.5;
      double qqq = qq + turretZeroOffset;
      return Functions.clamp(qqq, 0, 1.0);
      // note a value of 0 or 1 indicates beyond range!
   }

   public double getHoodValueFromDistance(double distance) {
      return Functions.interpolate(distance, nearTest, farTest, hoodNearest, hoodFar);
   }

   public double getSpinnerRPMfromDistance(double distance) {
      return Functions.interpolate(distance, nearTest, farTest, spinNear, spinFar);
   }

   public Vector2D getTargetVector(Position target) {
      if (target==null || parts.positionMgr.noPosition()) return new Vector2D();
      return new Vector2D(parts.positionMgr.robotPosition, target);
   }

   public double getTurretAngle(Position target) {
      if (target==null || parts.positionMgr.noPosition()) return 0;
      return Functions.normalizeAngle(getTargetVector(target).angle - parts.positionMgr.robotPosition.R);
   }

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
      spinnerArmed = true;
   }
   public static void spinnerOn() {
      spinnerOn(spinnerRPM);
      spinnerArmed = true;
   }
   public static void spinnerOff() {
      motorSpinner.setPower(0);
      spinnerArmed = false;
   }

   public enum rgbIndicatorColor {
      Off (0.0),
      Red (0.279),
      Orange (0.333),
      Yellow (0.388),
      Sage (0.444),
      Green (0.500),
      Azure (0.555),
      Blue (0.611),
      Indigo (0.666),
      Violet (0.715), //(0.722),
      White (1.0);

      private final double color;

      rgbIndicatorColor(double color) {
         this.color = color;
      }
   }
}