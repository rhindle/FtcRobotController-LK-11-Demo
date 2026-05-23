package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import android.annotation.SuppressLint;

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

   public static double servoTurretLOffset = 0.0;
   public static double servoTurretROffset = 0.0;

   public static double turretSweepRangeL = 110;
   public static double turretSweepRangeR = 110;
   public static double turretTurn90 = -0.33;  //todo: measure this
   public static double turret1Degree = turretTurn90 / 90;

   public static DcMotorEx motorSpin1;
   public static DcMotorEx motorSpin2;

   static String motorSpin1Name = "motor2B";
   static String motorSpin2Name = "motor3B";
   static String servoHoodName = "servo2B";
   static String servoTurretLName = "servo5";
   static String servoTurretRName = "servo5B";
   static String servoLEDName = "servo0";

   static double spinTicks = 28.0;
   static int spinMotorRPM = 6000;

   static double servoHoodPos = 0.5;
   static double turretAngle;
   static double turretPos;
   static double spinnerTargetSpeed = 1500;
   static double spinnerManualSpeed = 1500;
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

   static final double[][] turretTable = {{50, 0.012, 2700},   // distance, hood position, spinner speed
                                         {75, 0.260, 3000},
                                         {100, 0.310, 3300},
                                         {120, 0.32, 3500},
                                         {140, 0.482, 4000}};

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize(){
      servoHood = new ServoSSR(parts.robotV2.getServoByName(servoHoodName))
              .setDirectionSSR(Servo.Direction.FORWARD)
              .setSweepTime(1200);
      servoTurretL = new ServoSSR(parts.robotV2.getServoByName(servoTurretLName))
              .setDirectionSSR(Servo.Direction.FORWARD)
              .setOffset(servoTurretLOffset)
              .setSweepTime(1500);
      servoTurretR = new ServoSSR(parts.robotV2.getServoByName(servoTurretRName))
              .setDirectionSSR(Servo.Direction.FORWARD)
              .setOffset(servoTurretROffset)
              .setSweepTime(1500);
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

   @SuppressLint("DefaultLocale")
   public static void runLoop() {

      // update internal target vector
      targetVector = getTargetVector(TB_Misc.targetCurrent);

      // update turret position
      if (turretArmed) {
         turretAngle = getTurretAngle(TB_Misc.targetCurrent);
         turretOutOfRange = turretAngle > turretSweepRangeL || turretAngle < -turretSweepRangeR;
         turretPos = getTurretValueFromAngle(turretAngle);
         if (!Double.isNaN(turretPos)) {  //todo: Does this prevent the error on 5/23/2026? If so, how does this become NaN?
            servoTurretL.setPosition(turretPos);
            servoTurretR.setPosition(turretPos);
         }
      }

      // update hood
      if (turretArmed) {
         double hoodPos = calcHoodForDistance(targetVector.distance);
         servoHood.setPosition(hoodPos);
         servoHoodPos = hoodPos;
      }

      // update spinner
      if (spinnerArmed) {
         double spinnerRPM = calcRpmForDistance(targetVector.distance);
         setSpinnerTargetSpeed(spinnerRPM);
      }

      // update LED
      if (!turretArmed || !spinnerArmed) {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Off.color);
      }
      else if (turretOutOfRange) {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Red.color);
      }
      else if (isSpinnerInToleranceV2() && servoHood.isDone() && servoTurretL.isDone()) {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Green.color);
      }
      else {
         servoLED.setPosition(TB_Misc.rgbIndicatorColor.Yellow.color);
      }

      TelemetryMgr.message(TelemetryMgr.Category.TB_TURRET, "Target Vector", targetVector.toString());
      TelemetryMgr.message(TelemetryMgr.Category.TB_TURRET, "Turret Angle",
              String.format("%.3f", turretPos) + " (" +
              String.format("%.3f", turretAngle) + "°)");
      TelemetryMgr.message(TelemetryMgr.Category.TB_TURRET, "Spinner Speed",
              String.format("%05d", getSpinnerRPM()) + " (" +
              String.format("%05d", (int)spinnerTargetSpeed) + ") (" +
              String.format("%05d", (int)spinnerManualSpeed) +")");
      TelemetryMgr.message(TelemetryMgr.Category.TB_TURRET, "Hood Pos", servoHoodPos);
   }

   public static void stop() {
      StateMachine.stopGroups("spinner");
      motorSpin1.setPower(0);
      motorSpin2.setPower(0);
      servoHood.disable();
      servoTurretL.disable();
      servoTurretR.disable();
      turretArmed = false;
      spinnerArmed = false;
   }

   public static void armTurret(boolean arm) {
      turretArmed = arm;
      if (!turretArmed) {
         servoTurretL.setPosition(0.5);
         servoTurretR.setPosition(0.5);
      }
   }

   public static void armSpinner(boolean arm) {
      spinnerArmed = arm;
      if (!spinnerArmed) {
         spinOff();
      }
   }

   public static void setSpinnerTargetSpeed(double rpm) {
      if (rpm < 0 || rpm > 5500) return;
      spinnerTargetSpeed = rpm;
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
//      return 0.5 + turret1Degree * Functions.clamp(angle, -turretSweepRangeR, turretSweepRangeL);
      return Functions.clamp(0.5 + turret1Degree * Functions.clamp(angle, -turretSweepRangeR, turretSweepRangeL),0,1);
   }

   public static double calcHoodForDistance(double distance) {
      return interpolateUsingTable(turretTable, distance, 1);
   }

   public static double calcRpmForDistance(double distance) {
      return interpolateUsingTable(turretTable, distance, 2);
   }

   public static double interpolateUsingTable(double[][] table, double value, int index) {
      // index must be 1 or 2 for turretTable.  Probably should sanity check.
      // This does not extrapolate beyond the ends of the table; uses the first or last value instead.
      int i;
      for (i = 0; i < table.length; i++) {
         if (value <= table[i][0]) break;
      }
      if (i==0) return table[0][index];
      if (i==table.length) return table[table.length-1][index];
      return Functions.interpolate(value, table[i-1][0], table[i][0], table[i-1][index], table[i][index]);
   }

   public static Vector2D getTargetVector(Position target) {
      //todo: make an improved version for shoot-on-the-move target adjustment
      //   which will need to account for:
      //      - change in position of the robot based on velocity and the time to transfer
      //      - velocity of the robot imparted to the ball (and the time of flight)
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

   static public boolean isSpinnerInToleranceV2() {
      // todo: figure out if undershoot is consistent or should be a multiplier
      double target = spinnerTargetSpeed - spinnerUndershoot;  // adjusted target speed accounting for undershoot
      double diff = Math.abs(getSpinnerRPM() - target);  // difference between actual and target
      return diff <= spinnerTolerance;
   }


}