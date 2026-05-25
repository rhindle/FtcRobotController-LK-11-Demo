package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
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
      StateMachine.stopGroups("intake", "transfer");
      intakeOff();
      servoGateL.disable();
      servoGateR.disable();
   }

   public static void intakeOn() {
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

}