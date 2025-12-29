package org.firstinspires.ftc.teamcode.RobotParts.Test2025;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
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
   static final double hoodNearest              = 0.129;  // 1 tile diagonally
   static final double hoodMiddle               = 0.348;
   static final double hoodFar                  = 0.501;
   static final int hoodSweepTime               = 1500;   // spec is 1250

   static final double turretNeutral            = 0.5;
   static final double turretZeroOffset         = 0.0;
   static final double turretRangeDegrees       = 400.00;
   static final int turretSweepTime             = 1500;   // spec is 1250

   //kick1 is front, kick2 is center, kick3 is rear
   static final double kick1Docked              = 0.496;
   static final double kick1Launch              = 0.914; //0.846;
   static final double kick1Horizontal          = 0.846;
   static final int kick1SweepTime              = 1500;   // spec is 1250

   static final double kick2Docked              = 0.482;
   static final double kick2Launch              = 0.060; //0.137;
   static final double kick2Horizontal          = 0.137;
   static final double kick2Help                = 0.137;
   static final int kick2SweepTime              = 1500;   // spec is 1250

   static final double kick3Docked              = 0.526;
   static final double kick3Launch              = 0.108;
   static final double kick3Horizontal          = 0.196;
   static final int kick3SweepTime              = 1500;   // spec is 1250

   static final double spinNear                 = 3300;
   static final double spinMiddle               = 3900;
   static final double spinFar                  = 4500;
   static double spinnerRPM                     = 0;  //move this
   static double spinnerTolerance               = 100;
   public static PIDFCoefficients spinnerPID    = new PIDFCoefficients(100,0,0,12.4);

   static final Position targetRed              = new Position(-70.5, 70.5, 0.0);
   static final Position targetBlue             = new Position(-70.5, -70.5, 0.0);

   /* Internal use */
   private static double spinRPMset;
   private static final double spinMultiplier = 60.0 / 28.0 * 1.0;  // seconds / ticksPerRev * gearRatio;

   private static final double nearTest      = 48;  // 1 tile diagonally
   private static final double midTest       = 98;
   private static final double farTest       = 140;

   private static ServoSSR servoHood;
   private static ServoSSR servoTurret;
   private static ServoSSR servoKick1;
   private static ServoSSR servoKick2;
   private static ServoSSR servoKick3;
   private static ServoSSR servoLED;

   private static DcMotorEx motorSpinner;
   private static DcMotorEx motorIntake;

   public static StateMachine launchKick1;
   public static StateMachine launchKick1a;
   public static StateMachine launchKick2;
   public static StateMachine launchKick3;
   public static StateMachine launchAll;
   public static StateMachine resetAll;

   public static NormalizedColorSensor sensorColor = null;

   public static Vector2D targetVector;
   public static boolean turretArmed = false;
   public static boolean spinnerArmed = false;
   public static boolean intakeArmed = false;

   public static boolean turretOutOfRange = false;

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

      servoHood = new ServoSSR(parts.robotV2.getServoByName("servo2B"));
      servoTurret = new ServoSSR(parts.robotV2.getServoByName("servo4"));
      servoKick1 = new ServoSSR(parts.robotV2.getServoByName("servo0"));  // kick1 is left front
      servoKick2 = new ServoSSR(parts.robotV2.getServoByName("servo0B"));  // kick2 is center
      servoKick3 = new ServoSSR(parts.robotV2.getServoByName("servo2"));  // kick3 is left rear
      servoLED = new ServoSSR(parts.robotV2.getServoByName("servo5"));

      motorSpinner = parts.robotV2.getMotorByName("motor3B");
      motorIntake = parts.robotV2.getMotorByName("motor3");

      initServos();
      initMotors();

      buildStateMachines();

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

//      delayedActions();

      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Target Vector", targetVector.toString());
      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Spinner Speed", getSpinnerRPM());

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

      motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
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


   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //     Manual User Controls
   //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


   public static void armTurret(boolean arm) {
      turretArmed = arm;
//      if (!turretArmed) {
//         servoHood.setPosition(hoodNeutral);
//         servoTurret.setPosition(turretNeutral+turretZeroOffset);
//      }
   }
   public static void armSpinner(boolean arm) {
      spinnerArmed = arm;
      if (!spinnerArmed) {
         stopSpinner();
      }
   }

   public static double getTurretValueFromAngle(double angle) {
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

   public static double getHoodValueFromDistance(double distance) {
      return Functions.interpolate(distance, nearTest, farTest, hoodNearest, hoodFar);
   }

   public static double getSpinnerRPMfromDistance(double distance) {
      return Functions.interpolate(distance, nearTest, farTest, spinNear, spinFar);
   }

   public static Vector2D getTargetVector(Position target) {
      if (target==null || parts.positionMgr.noPosition()) return new Vector2D();
      return new Vector2D(parts.positionMgr.robotPosition, target);
   }

   public static double getTurretAngle(Position target) {
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
   }
   public static void spinnerOff() {
      motorSpinner.setPower(0);
      spinnerArmed = false;
   }

   public static void spinnerSlowTest() {
      spinnerOn(1500);
      spinnerArmed = false;  // so it won't auto adjust for distance
   }

   public static void intakeToggle() {
      intakeArmed = !intakeArmed;
      if (intakeArmed) intakeOn();
      else intakeOff();
   }

   public static void intakeOn() {
      intakeArmed = true;
      //dock all servos!
      resetAll.restart();
      motorIntake.setPower(1);
   }

   public static void intakeOff() {
      intakeArmed = false;
      motorIntake.setPower(0);
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

   /* State Machines */

   static void buildStateMachines() {
      StateMachine task;

      launchKick1 = new StateMachine("kick1");
      task = launchKick1;
      //task.setGroups("launcher", "green");
      //task.setStopGroups("launcher", "green");    // groups to kill
      task.setMemberGroups("all", "kick1");  // will be killed by
      task.setAutoRestart(false);
      //task.setStopRunnable( () -> {} );
      //task.setTimeLimit(5000);
      //task.setTimeoutRunnable( () -> {} );
      //task.setEndCriteria( () -> false );
      //task.setEndCriteriaRunnable( () -> {} );
      task.addRunOnce(T25_Effector::intakeOff);
      task.addRunOnce(() -> servoKick1.setPosition(kick1Launch));
      task.addWaitFor(() -> servoKick1.isDone());
//      task.addDelayOf(500);
      task.addRunOnce(() -> servoKick1.setPosition(kick1Docked));
      task.addWaitFor(() -> servoKick1.isDone());

      // special test for no ball in center position
      launchKick1a = new StateMachine("kick1a");
      task = launchKick1a;
      task.setMemberGroups("all", "kick1");  // will be killed by
      task.setAutoRestart(false);
      task.addRunOnce(T25_Effector::intakeOff);
      task.addRunOnce(() -> servoKick1.setPosition(kick1Launch));
      task.addDelayOf(50);
      task.addRunOnce(() -> servoKick2.setPosition(kick2Help));
      task.addWaitFor(() -> servoKick1.isDone());
      task.addWaitFor(() -> servoKick2.isDone());
      task.addRunOnce(() -> servoKick2.setPosition(kick2Docked));
      task.addDelayOf(50);
      task.addRunOnce(() -> servoKick1.setPosition(kick1Docked));
      task.addWaitFor(() -> servoKick2.isDone());
      task.addWaitFor(() -> servoKick1.isDone());

      launchKick2 = new StateMachine("kick2");
      task = launchKick2;
      task.setMemberGroups("all", "kick2");  // will be killed by
      task.setAutoRestart(false);
      task.addRunOnce(T25_Effector::intakeOff);
      task.addRunOnce(() -> servoKick2.setPosition(kick2Launch));
      task.addWaitFor(() -> servoKick2.isDone());
//      task.addDelayOf(500);
      task.addRunOnce(() -> servoKick2.setPosition(kick2Docked));
      task.addWaitFor(() -> servoKick2.isDone());

      launchKick3 = new StateMachine("kick3");
      task = launchKick3;
      task.setMemberGroups("all", "kick3");  // will be killed by
      task.setAutoRestart(false);
      task.addRunOnce(T25_Effector::intakeOff);
      task.addRunOnce( () -> servoKick3.setPosition(kick3Launch));
      task.addWaitFor( () -> servoKick3.isDone());
//      task.addDelayOf( 500);
      task.addRunOnce( () -> servoKick3.setPosition(kick3Docked));
      task.addWaitFor( () -> servoKick3.isDone());

      launchAll = new StateMachine("all");
      task = launchAll;
      task.setStopGroups("kick1", "kick2", "kick3");    // groups to kill
      task.setMemberGroups("reset");  // will be killed by
      task.setAutoRestart(false);
      task.addRunOnce(T25_Effector::intakeOff);
      task.addRunOnce(launchKick3::restartNoStop);
      task.addWaitFor(launchKick3::isDone);
      task.addRunOnce(launchKick1::restartNoStop);
      task.addWaitFor(launchKick1::isDone);
      task.addRunOnce(launchKick2::restartNoStop);
      task.addWaitFor(launchKick2::isDone);

      resetAll = new StateMachine("reset");
      task = resetAll;
      task.setStopGroups("kick1", "kick2", "kick3");    // groups to kill
      //task.setMemberGroups("blue");  // will be killed by
      task.setAutoRestart(false);
      task.addRunOnce( ()-> {
         servoKick1.setPosition(kick1Docked);
         servoKick2.setPosition(kick2Docked);
         servoKick3.setPosition(kick3Docked);
      });

   }
}