package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.PartsInterfaceStatic;
import org.firstinspires.ftc.teamcode.ZZ.ZZ_ServoSSR;

public class TB_Turret implements PartsInterfaceStatic {

   public static Parts parts;

   public static ZZ_ServoSSR servoHood;
   public static ZZ_ServoSSR servoTurretL;
   public static ZZ_ServoSSR servoTurretR;

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

   final double spinSmallChange = .0001;
   final double spinLargeChange = .005;
   final double hoodChange = .002;

   public static void setup(Parts p) {
      parts = p;
   }

   public static void initialize(){
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
   }

   public static int getMotorSpinSpeed(DcMotorEx m) {
      return (int) (m.getVelocity() * 60.0 / spinTicks);
   }
}