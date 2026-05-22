package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Vector2D;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterfaceStatic;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class TB_Turret implements PartsInterfaceStatic {

   public static Parts parts;

   public static ServoSSR servoHood;
   public static ServoSSR servoTurretL;
   public static ServoSSR servoTurretR;
   public static ServoSSR servoLED;

   public static DcMotorEx motorSpin1;
   public static DcMotorEx motorSpin2;

   static String motorSpin1Name = "motor2B";
   static String motorSpin2Name = "motor3B";
   static String servoHoodName = "servo2B";
   static String servoTurretLName = "servo5";
   static String servoTurretRName = "servo5B";
   static String servoLEDName = "servo4B";

   static double spinTicks = 28.0;
   static int spinMotorRPM = 6000;

   static double servoHoodPos = 0.5;
   static double spinnerTargetSpeed = 1500;
   static double spinnerIdleSpeed = 500;
   public static PIDFCoefficients spinnerPID = new PIDFCoefficients(100,0,0,12.4);

   static final double spinSmallChange = .0001;
   static final double spinLargeChange = .005;
   static final double hoodChange = .002;

   public static Vector2D targetVector;
   public static boolean turretArmed = false;
   public static boolean spinnerArmed = false;
   public static boolean turretOutOfRange = false;

   public static int spinnerTolerance = 50; // 75;
   public static int spinnerUndershoot = 100;

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize(){
      servoHood = new ServoSSR(parts.robotV2.getServoByName(servoHoodName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1200);
      servoTurretL = new ServoSSR(parts.robotV2.getServoByName(servoTurretLName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1500);
      servoTurretR = new ServoSSR(parts.robotV2.getServoByName(servoTurretRName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1500);
      servoLED = new ServoSSR(parts.robotV2.getServoByName(servoLEDName));

      motorSpin1 = parts.robotV2.getMotorByName(motorSpin1Name);
      motorSpin2 = parts.robotV2.getMotorByName(motorSpin2Name);

      motorSpin1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpin2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpin1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      motorSpin2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      motorSpin1.setDirection(DcMotorEx.Direction.REVERSE);
      motorSpin2.setDirection(DcMotorEx.Direction.REVERSE);
      motorSpin1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, spinnerPID);
      motorSpin2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, spinnerPID);

   }

   public static void preInit() {
   }

   public static void initLoop() {
   }

   public static void preRun() {
   }

   public static void runLoop() {

      // update internal target vector
      targetVector = getTargetVector(TB_Misc.targetCurrent);

      // update turret position
      if (turretArmed) {
         double turretAngle = getTurretAngle(TB_Misc.targetCurrent);
         double turretPos = getTurretValueFromAngle(turretAngle);
         servoTurretL.setPosition(turretPos);
         servoTurretR.setPosition(turretPos);
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
         setSpinnerTargetSpeed(spinnerRPM);
      }

      // update LED
      if (!turretArmed || !spinnerArmed) {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Off.color);
      }
      else if (turretOutOfRange) {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Red.color);
      }
      else if (isSpinnerInTolerance() && servoHood.isDone() && servoTurretL.isDone()) {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Green.color);
      }
      else {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Yellow.color);
      }

      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Target Vector", targetVector.toString());
      TelemetryMgr.message(TelemetryMgr.Category.T25_EFF, "Spinner Speed", getSpinnerRPM());
   }

   public static void stop() {
      StateMachine.stopGroups("spinner");
      motorSpin1.setPower(0);
      motorSpin2.setPower(0);
      servoHood.disable();
      servoTurretL.disable();
      servoTurretR.disable();
   }

   public static void setMotorSpinSpeed() {
      setSpinnerTargetSpeed(spinnerTargetSpeed);
   }

   public static void setSpinnerTargetSpeed(double rpm) {
      if (rpm < 0 || rpm > 5500) return;
      double velocity = rpm / (60.0 / spinTicks);
      motorSpin1.setVelocity(velocity);
      motorSpin2.setVelocity(velocity);
   }

   public static void spinIdle() {
      // better to use the spindown task than call this directly
      setSpinnerTargetSpeed(spinnerIdleSpeed);
   }

   public static void spinOff() {
      motorSpin1.setPower(0);
      motorSpin2.setPower(0);
   }

   public static int getMotorSpinSpeed(DcMotorEx m) {
      return (int) (m.getVelocity() * 60.0 / spinTicks);
   }
   public static int getSpinnerRPM() {
      return (int) (getMotorSpinSpeed(motorSpin1) + getMotorSpinSpeed(motorSpin2)) / 2;
   }

   public static double getTurretValueFromAngle(double angle) {
//      // turret geared so 300° range = 400°
//      // 0.5 = 0, 0.1 = 180, 0.9 = -180 ???
//      // let's calculate with an offset of -0.5 to make calculation easier.
//      angle = Functions.normalizeAngle(angle);
//      double rangeFraction = 0.9;
//      // 180*.9
//      double q = (angle / 180) * rangeFraction;
//      double qq = q/2 + 0.5;
//      double qqq = qq + turretZeroOffset;
//      return Functions.clamp(qqq, 0, 1.0);
//      // note a value of 0 or 1 indicates beyond range!
      return 0;  //major rework needed
   }

   /* ************  REPLACE THIS INTERPOLATION STUFF */
   private static final double nearTest      = 48;  // 1 tile diagonally
   private static final double midTest       = 98;
   private static final double farTest       = 140;
   static final double hoodNearest              = 0.129;  // 1 tile diagonally
   static final double hoodFar                  = 0.501;
   static final double spinNear                 = 3300;
   static final double spinFar                  = 4500;

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
      return isSpinnerInTolerance(spinnerTargetSpeed, spinnerTolerance);
   }
   static boolean isSpinnerInTolerance(double targetRPM, double tolerance) {
      return Math.abs(getSpinnerRPM() - targetRPM) <= tolerance;
   }

   public boolean isSpinnerInToleranceV2() {
      // todo: figure out if undershoot is consistent or should be a multiplier
      double target = spinnerTargetSpeed - spinnerUndershoot;  // adjusted target speed accounting for undershoot
      double diff = Math.abs(getSpinnerRPM() - target);  // difference between actual and target
      return diff <= spinnerTolerance;
   }


}