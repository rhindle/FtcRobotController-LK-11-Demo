package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.PartsInterfaceStatic;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class TB_Intake implements PartsInterfaceStatic {

   public static Parts parts;

   public static ServoSSR servoGateL;
   public static ServoSSR servoGateR;

   public static DcMotorEx motorIntake;
   public static DcMotorEx motorTransfer;

   static String motorIntakeName = "motor0B";
   static String motorTransferName = "motor1B";
   static String servoGateLName = "servo0B";
   static String servoGateRName = "servo1B";

   static final double servoGateLOpen = 1;
   static final double servoGateLClosed = 0;
   static final double servoGateROpen = 1;
   static final double servoGateRClosed = 0;

   static final int reverseTicks = 10; //10;
   static boolean intakeRunning = false;
   static boolean transferRunning = false;
   static boolean gateOpen = false;

   static boolean disableIntakeSensors = false;
   static String sensorBall1Name = "digital0B";
   static String sensorBall2Name = "digital0";
   static String sensorBall3Name = "digital2";
   public static DigitalChannel sensorBall1, sensorBall2, sensorBall3;
   public static boolean ball1, ball2, ball3;
   public static boolean atLeast1, probably3, definitely3, probably2, probably0;

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize() {
      servoGateL = new ServoSSR(parts.robotV2.getServoByName(servoGateLName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(250);
      servoGateR = new ServoSSR(parts.robotV2.getServoByName(servoGateRName)).setDirectionSSR(Servo.Direction.REVERSE).setSweepTime(250);

      motorIntake = parts.robotV2.getMotorByName(motorIntakeName);
      motorTransfer = parts.robotV2.getMotorByName(motorTransferName);

      motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorTransfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      motorTransfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
      motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
      motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);

      sensorBall1 = parts.robotV2.getDigitalByName(sensorBall1Name);
      sensorBall2 = parts.robotV2.getDigitalByName(sensorBall2Name);
      sensorBall3 = parts.robotV2.getDigitalByName(sensorBall3Name);

      sensorBall1.setMode(DigitalChannel.Mode.INPUT);
      sensorBall2.setMode(DigitalChannel.Mode.INPUT);
      sensorBall3.setMode(DigitalChannel.Mode.INPUT);

      disableIntakeSensors = false;

   }

   public static void preInit() {
   }

   public static void initLoop() {
   }

   public static void preRun() {
   }

   public static void runLoop() {
      if (!disableIntakeSensors) {
         readSensors();
         if (transferRunning) interpretSensorsTransfer();
         else if (intakeRunning) {
            interpretSensorsIntake();
            // do things here, or use tasks?
         }
      }

      String telString = (ball1 ? "T " : "F ") +
              (ball2 ? "T " : "F ") +
              (ball3 ? "T " : "F ") + " | " +
              "0?" + (probably0 ? "T " : "F ") +
              "1+" + (atLeast1 ? "T " : "F ") +
              "2?" + (probably2 ? "T " : "F ") +
              "3?" + (probably3 ? "T " : "F ") +
              "3*" + (definitely3 ? "T " : "F ");
      TelemetryMgr.message(TelemetryMgr.Category.TB_INTAKE, "Sensor ", telString);
   }

   public static void stop() {
      StateMachine.stopGroups("intake", "transfer");
      intakeOff();
      servoGateL.disable();
      servoGateR.disable();
   }

   public static void intakeOn() {
      clearSensorFlags();
      intakeRunning = true;
      motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorIntake.setPower(1);
   }

   public static void intakeOff() {
      intakeRunning = false;
      transferRunning = false;
      motorIntake.setPower(0);
      motorTransfer.setPower(0);
   }

   public static void intakeReverse() {
      motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorTransfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorIntake.setPower(-1);
      motorTransfer.setPower(-1);
   }

   public static void transferOn() {
      clearSensorFlags();
      transferRunning = true;
      intakeRunning = true;
      motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorTransfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorIntake.setPower(1);
      motorTransfer.setPower(1);
   }

   public static void transferOff() {
      transferRunning = false;
      motorTransfer.setPower(0);
   }

   public static void gateOpen() {
      gateOpen = true;
      servoGateL.setPosition(servoGateLOpen);
      servoGateR.setPosition(servoGateROpen);
   }

   public static void gateClose() {
      gateOpen = false;
      servoGateL.setPosition(servoGateLClosed);
      servoGateR.setPosition(servoGateRClosed);
   }

   public static void readSensors() {
      if (!disableIntakeSensors) {
         ball1 = sensorBall1.getState();
         ball2 = sensorBall2.getState();
         ball3 = sensorBall3.getState();
      }
   }

   public static void disableSensors() {
      disableIntakeSensors = true;
      clearSensorFlags();
   }

   public static void clearSensorFlags() {
      ball1 = false;
      ball2 = false;
      ball3 = false;
      atLeast1 = false;
      probably3 = false;
      definitely3 = false;
      probably2 = false;
      probably0 = false;
   }

   public static void interpretSensorsIntake() {
      if (!disableIntakeSensors) {
         if (!ball1 && !ball2 && !ball3) {
            clearSensorFlags();
            probably0 = true;
         }
         if (ball1 || ball2 || ball3) atLeast1 = true;
         if (ball3) probably3 = true;
         if (ball1 && ball2 && ball3) definitely3 = true;
         if (ball1 && ball2) probably2 = true;
      }
   }

   public static void interpretSensorsTransfer() {
      interpretSensorsIntake();
   }

}