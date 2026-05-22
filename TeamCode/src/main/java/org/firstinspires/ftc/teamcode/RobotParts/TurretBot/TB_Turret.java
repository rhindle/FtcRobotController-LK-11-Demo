package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.PartsInterfaceStatic;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class TB_Turret implements PartsInterfaceStatic {

   public static Parts parts;

   public static ServoSSR servoHood;
   public static ServoSSR servoTurretL;
   public static ServoSSR servoTurretR;

   public static DcMotorEx motorSpin1;
   public static DcMotorEx motorSpin2;

   static String motorSpin1Name = "motor2B";
   static String motorSpin2Name = "motor3B";
   static String servoHoodName = "servo2B";
   static String servoTurretLName = "servo3B";
   static String servoTurretRName = "servo4B";

   static double spinTicks = 28.0;
   static int spinRPM = 6000;

   static double servoHoodPos = 0.5;
   static double motorSpinSpeed                = 1500;
   static double motorSpinIdleSpeed            = 500;
   final boolean motor1reverse             = false;
   final boolean motor2reverse             = false;
   public static PIDFCoefficients launchSpinPID = new PIDFCoefficients(100,0,0,12.4);

   static final double spinSmallChange = .0001;
   static final double spinLargeChange = .005;
   static final double hoodChange = .002;

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize(){
      servoHood = new ServoSSR(parts.robotV2.getServoByName(servoHoodName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1200);
      servoTurretL = new ServoSSR(parts.robotV2.getServoByName(servoTurretLName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1500);
      servoTurretR = new ServoSSR(parts.robotV2.getServoByName(servoTurretRName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1500);

      motorSpin1 = parts.robotV2.getMotorByName(motorSpin1Name);
      motorSpin2 = parts.robotV2.getMotorByName(motorSpin2Name);

      motorSpin1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpin2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpin1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      motorSpin2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      motorSpin1.setDirection(DcMotorEx.Direction.REVERSE);
      motorSpin2.setDirection(DcMotorEx.Direction.REVERSE);
      motorSpin1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);
      motorSpin2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);

   }

   public static void preInit() {
   }

   public static void initLoop() {
   }

   public static void preRun() {
   }

   public static void runLoop() {
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
      setMotorSpinSpeed(motorSpinSpeed);
   }

   public static void setMotorSpinSpeed(double rpm) {
      if (rpm < 0 || rpm > 5500) return;
      double velocity = rpm / (60.0 / spinTicks);
      motorSpin1.setVelocity(velocity);
      motorSpin2.setVelocity(velocity);
   }

   public static void spinIdle() {
      // better to use the spindown task than call this directly
      setMotorSpinSpeed(motorSpinIdleSpeed);
   }

   public static void spinOff() {
      motorSpin1.setPower(0);
      motorSpin2.setPower(0);
   }

   public static int getMotorSpinSpeed(DcMotorEx m) {
      return (int) (m.getVelocity() * 60.0 / spinTicks);
   }
}