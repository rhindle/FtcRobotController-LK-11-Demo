package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class Controls implements PartsInterface {

   public Parts parts;
   public Gamepad gamepad1;
   public Gamepad gamepad2;
   public ButtonMgr buttonMgr;
   public DriveData driveData;
   public double liftSpeed;

   /* Constructor */
   public Controls(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
      this.gamepad1 = parts.opMode.gamepad1;
      this.gamepad2 = parts.opMode.gamepad2;
      this.buttonMgr = parts.buttonMgr;
   }

   public void initialize() {
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      userInput();
   }

   public void stop() {
   }

   public void userInput() {
   }

}
