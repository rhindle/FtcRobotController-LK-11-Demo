package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;

public class ControlsTB extends Controls {

   // settings
   double speedSlow = 0.25;
   double speedNormal = 0.5;
   double speedFast = 1.0;

   // working variables
   boolean isStopped = false;
//   boolean fullSpeed = false;
//   float speedFactor = 1;    // this is float on purpose so drivedata overload is correct!
   float speedFactor = (float)speedNormal;

   public ControlsTB(Parts parts) {
      super(parts);
   }

   @Override
   public void runLoop() {
      driveData = new DriveData();
      userInput();
      parts.userDrive.setUserDriveSettings(driveData);
   }

   @Override
   public void userInput() {
      switch (controlMode) {
         case 0:
            NormalControls();
            break;
         case 1:
            TestControls();
            break;
         case 100:
            AutoControls();
            break;
      }
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~ NORMAL CONTROLS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

   public void NormalControls() {

      if (buttonMgr.getState(1, Buttons.right_bumper, State.wasPressed)) {
         TB_Misc.controlsFullSpeed = !TB_Misc.controlsFullSpeed;
         if (TB_Misc.controlsFullSpeed) speedFactor = (float)speedFast;
         else speedFactor = (float)speedNormal;
         parts.userDrive.setSpeedMaximum(speedFactor);
      }

      if (!parts.useForzaControls) {
         driveData = new DriveData(gamepad1.left_stick_x * speedFactor,
                 gamepad1.left_stick_y * speedFactor,
                 gamepad1.right_stick_x * speedFactor);
      }
      else {
         driveData = new DriveData(gamepad1.left_trigger * speedFactor,
                 gamepad1.right_trigger * speedFactor,
                 gamepad1.right_stick_x * speedFactor * (float)0.7,   // forced lower speed @ AV request
                 gamepad1.left_stick_x * speedFactor);
      }

      // turret test stuff
      if (buttonMgr.getState(1, Buttons.x, State.wasSingleTapped)) {
         TB_Turret.armTurret(false);
      }
      if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)) {
         TB_Turret.armTurret(true);
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasSingleTapped)) {
         TB_Turret.armSpinner(false);
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
         TB_Turret.armSpinner(true);
      }
      if (buttonMgr.getState(1, Buttons.a, State.wasPressed)) {
         // toggle intake on  (close gate, start intake)
         TB_Intake.intakeRunning = !TB_Intake.intakeRunning;
         if (TB_Intake.intakeRunning) TB_Tasks.smIntakeOn.restart();
         else TB_Tasks.smIntakeOff.restart();
      }
      if (buttonMgr.getState(1, Buttons.b, State.wasPressed)) {
         //intake+transfer   (open gate, start both motors)  [only if in range]
         if (!TB_Turret.turretOutOfRange) {
            TB_Intake.intakeRunning = false;
            TB_Tasks.smLaunch.restart();
         }
      }

      // emergency overrides with left bumper
      if (buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {
         // modify spin speed
         TB_Turret.spinnerManualSpeed += gamepad2.left_stick_y * -TB_Turret.spinLargeChange * TB_Turret.spinMotorRPM;
         TB_Turret.spinnerManualSpeed += gamepad2.right_stick_y * -TB_Turret.spinSmallChange * TB_Turret.spinMotorRPM;
         TB_Turret.spinnerManualSpeed = Math.max(-TB_Turret.spinMotorRPM, Math.min(TB_Turret.spinMotorRPM, TB_Turret.spinnerManualSpeed));

         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasPressed)) {
            TB_Turret.manualOverride(100);
         }
         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasPressed)) {
            TB_Turret.manualOverride(75);
         }
         if (buttonMgr.getState(2, Buttons.dpad_down, State.wasPressed)) {
            TB_Turret.manualOverride(140);
         }
         if (buttonMgr.getState(2, Buttons.dpad_right, State.wasPressed)) {
            TB_Turret.manualOverride(120);
         }
         if (buttonMgr.getState(2, Buttons.b, State.wasPressed)) {
            TB_Intake.disableSensors();
         }
         if (buttonMgr.getState(2, Buttons.y, State.wasPressed)) {
            TB_LL.applyTransform();
         }
         if (buttonMgr.getState(2, Buttons.x, State.wasPressed)) {
            TB_LL.toggleAuto();
         }

      }
      //normal operation
      else {
         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasPressed)) {
            TB_Intake.gateOpen();
         }
         if (buttonMgr.getState(2, Buttons.dpad_right, State.wasPressed)) {
            TB_Intake.gateClose();
         }
         if (buttonMgr.getState(2, Buttons.dpad_up, State.isPressed)) {
            TB_Turret.servoHoodPos += TB_Turret.hoodChange;
            TB_Turret.servoHoodPos = Math.max(0, Math.min(1, TB_Turret.servoHoodPos));
            TB_Turret.servoHood.setPosition(TB_Turret.servoHoodPos);
         }
         if (buttonMgr.getState(2, Buttons.dpad_down, State.isPressed)) {
            TB_Turret.servoHoodPos -= TB_Turret.hoodChange;
            TB_Turret.servoHoodPos = Math.max(0, Math.min(1, TB_Turret.servoHoodPos));
            TB_Turret.servoHood.setPosition(TB_Turret.servoHoodPos);
         }
         if (buttonMgr.getState(2, Buttons.a, State.wasPressed)) {
            // toggle intake on  (close gate, start intake)
            TB_Intake.intakeRunning = !TB_Intake.intakeRunning;
            if (TB_Intake.intakeRunning) TB_Tasks.smIntakeOn.restart();
            else TB_Tasks.smIntakeOff.restart();
         }
         if (buttonMgr.getState(2, Buttons.x, State.isPressed)) {
            // spin up
            StateMachine.stopGroups("spinner");
            TB_Turret.setSpinnerTargetSpeed(TB_Turret.spinnerManualSpeed);
         }
         if (buttonMgr.getState(2, Buttons.y, State.wasPressed)) {
            //intake+transfer   (open gate, start both motors)  [only if in range]
            if (!TB_Turret.turretOutOfRange) {
               TB_Intake.intakeRunning = false;
               TB_Tasks.smLaunch.restart();
            }
         }
         if (buttonMgr.getState(2, Buttons.y, State.wasHeld)) {
            //intake+transfer   (open gate, start both motors)  [override]
            TB_Intake.intakeRunning = false;
            TB_Tasks.smLaunch.restart();
         }
         if (buttonMgr.getState(2, Buttons.b, State.wasPressed)) {
            // idle
            TB_Tasks.smSpinDown.restart();
            TB_Intake.intakeOff();
            TB_Intake.gateClose();
         }
         if (buttonMgr.getState(2, Buttons.right_bumper, State.wasPressed)) {
            // reverse
            TB_Intake.intakeReverse();
         }
         if (buttonMgr.getState(2, Buttons.right_bumper, State.wasReleased)) {
            TB_Intake.intakeOff();
            TB_Intake.clearSensorFlags();
         }
      }

      if (buttonMgr.getState(2, Buttons.left_trigger, State.isPressed) &&
              buttonMgr.getState(2, Buttons.right_trigger, State.isPressed)) {
         TB_Turret.manualTurretOffest = 0;
      }
      else if (buttonMgr.getState(2, Buttons.left_trigger, State.wasPressed)) {
         TB_Turret.manualTurretOffest += 1;
      }
      else if (buttonMgr.getState(2, Buttons.right_trigger, State.wasPressed)) {
         TB_Turret.manualTurretOffest -= 1;
      }

      if (buttonMgr.getState(2, Buttons.start, State.wasPressed)) {
         // stop spin
         StateMachine.stopGroups("spinner");
         TB_Turret.spinOff();
      }
      // stop all
      if (buttonMgr.getState(2, Buttons.back, State.wasPressed) || buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
         StateMachine.stopAll();
         TB_Turret.stop();
         TB_Intake.stop();
         parts.drivetrain.eStop();
         parts.autoDrive.eStop();
         parts.userDrive.eStop();
         // add drivetrain, etc
      }


//      // Toggle FCD
//      if (buttonMgr.getState(1, Buttons.start, State.wasDoubleTapped)) {
//         parts.userDrive.toggleFieldCentricDrive();
//      }
//
//      // Toggle HeadingHold
//      if (buttonMgr.getState(1, Buttons.back, State.wasDoubleTapped)) {
//         parts.userDrive.toggleHeadingHold();
//      }
//
//      // Store heading correction
//      if (buttonMgr.getState(1, Buttons.right_stick_button, State.wasReleased)) {
//         parts.userDrive.setDeltaHeading();
//      }
//
//      // Toggle PositionHold
//      if (buttonMgr.getState(1, Buttons.left_stick_button, State.wasReleased))  {
//         parts.userDrive.togglePositionHold();
//      }

   }

   //~~~~~~~~~~~~~~~~~~~~~~~~ AUTO CONTROLS ~~~~~~~~~~~~~~~~~~~~~~~~//

   public void AutoControls() {

      //        public static StateMachine smAutoTestGotoCenter;
//        public static StateMachine smAutoTestGotoNearShoot;
//        public static StateMachine smAutoTestGotoSpike1;
//        public static StateMachine smAutoTestGotoSpike2;
//        public static StateMachine smAutoTestDriveToGate;
//        public static StateMachine smAutoTestOperateGate;


      if (buttonMgr.getState(1, Buttons.dpad_right, State.wasPressed)) {
         TB_TasksAuto.smAutoTestGotoCenter.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasPressed)) {
         TB_TasksAuto.smAutoTestGotoNearShoot.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_left, State.wasPressed)) {
         TB_TasksAuto.smAutoTestGotoSpike1.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_down, State.wasPressed)) {
         TB_TasksAuto.smAutoTestGotoSpike2.restart();
      }
      if (buttonMgr.getState(1, Buttons.left_bumper, State.wasPressed)) {
         TB_TasksAuto.smAutoTestGotoSpike3.restart();
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasPressed)) {
         TB_TasksAuto.smAutoTestDriveToGate.restart();
      }
      if (buttonMgr.getState(1, Buttons.x, State.wasPressed)) {
         TB_TasksAuto.smAutoTestOperateGate.restart();
      }
      if (buttonMgr.getState(1, Buttons.a, State.wasPressed)) {
         TB_Tasks.smLaunch.restart();
      }
      if (buttonMgr.getState(1, Buttons.b, State.wasPressed)) {
         TB_Turret.armTurret(true);
         TB_Turret.armSpinner(true);
      }
      if (buttonMgr.getState(1, Buttons.right_bumper, State.wasPressed)) {
         TB_Turret.armTurret(false);
         TB_Turret.armSpinner(false);
      }
      if (buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
         StateMachine.stopAll();
         TB_Turret.stop();
         TB_Intake.stop();
      }
      if (buttonMgr.getState(1, Buttons.left_trigger, State.wasPressed)) {
         TB_TasksAuto.smAutoNearOrchestrator.restart();
      }

      // Delete this test - position queue
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld) &&
              buttonMgr.getState(1,Buttons.right_trigger, State.wasDoubleTapped)) {
//         parts.dsAuto.testAutoMethod4();
         TB_TasksAuto.smAutoTestTransitions.restart();
      }

   }


   //~~~~~~~~~~~~~~~~~~~~~~~~ TEST CONTROLS ~~~~~~~~~~~~~~~~~~~~~~~~//

   public void TestControls() {

//      if (buttonMgr.getState(1, Buttons.right_bumper, State.wasPressed)) {
//         fullSpeed = !fullSpeed;
//         if (fullSpeed) speedFactor = (float)speedFast;
//         else speedFactor = (float)speedNormal;
//         parts.userDrive.setSpeedMaximum(speedFactor);
//      }
//
//      if (!TB_Misc.forza) {
//         driveData = new DriveData(gamepad1.left_stick_x * speedFactor,
//                 gamepad1.left_stick_y * speedFactor,
//                 gamepad1.right_stick_x * speedFactor);
//      }
//      else {
//         driveData = new DriveData(gamepad1.left_trigger * speedFactor,
//                 gamepad1.right_trigger * speedFactor,
//                 gamepad1.right_stick_x * speedFactor,
//                 gamepad1.left_stick_x * speedFactor);
//      }
//
//      // turret test stuff
//      if (buttonMgr.getState(1, Buttons.x, State.wasSingleTapped)) {
//         TB_Turret.armTurret(false);
//      }
//      if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)) {
//         TB_Turret.armTurret(true);
//      }
//      if (buttonMgr.getState(1, Buttons.y, State.wasSingleTapped)) {
//         TB_Turret.armSpinner(false);
//      }
//      if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
//         TB_Turret.armSpinner(true);
//      }
//
//
//      if (buttonMgr.getState(2, Buttons.dpad_left, State.wasPressed)) {
//         TB_Intake.gateOpen();
//      }
//      if (buttonMgr.getState(2, Buttons.dpad_right, State.wasPressed)) {
//         TB_Intake.gateClose();
//      }
//      if (buttonMgr.getState(2, Buttons.dpad_up, State.isPressed)) {
//         TB_Turret.servoHoodPos += TB_Turret.hoodChange;
//         TB_Turret.servoHoodPos = Math.max(0, Math.min(1, TB_Turret.servoHoodPos));
//         TB_Turret.servoHood.setPosition(TB_Turret.servoHoodPos);
//      }
//      if (buttonMgr.getState(2, Buttons.dpad_down, State.isPressed)) {
//         TB_Turret.servoHoodPos -= TB_Turret.hoodChange;
//         TB_Turret.servoHoodPos = Math.max(0, Math.min(1, TB_Turret.servoHoodPos));
//         TB_Turret.servoHood.setPosition(TB_Turret.servoHoodPos);
//      }
//      if (buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {
//         // modify spin speed
//         TB_Turret.spinnerManualSpeed += gamepad2.left_stick_y * -TB_Turret.spinLargeChange * TB_Turret.spinMotorRPM;
//         TB_Turret.spinnerManualSpeed += gamepad2.right_stick_y * -TB_Turret.spinSmallChange * TB_Turret.spinMotorRPM;
//         TB_Turret.spinnerManualSpeed = Math.max(-TB_Turret.spinMotorRPM, Math.min(TB_Turret.spinMotorRPM, TB_Turret.spinnerManualSpeed));
//      }
//
//      if (buttonMgr.getState(2, Buttons.a, State.wasPressed)) {
//         // toggle intake on  (close gate, start intake)
//         TB_Intake.intakeRunning = !TB_Intake.intakeRunning;
//         if (TB_Intake.intakeRunning) TB_Tasks.smIntakeOn.restart();
//         else TB_Tasks.smIntakeOff.restart();
//      }
//
//      if (buttonMgr.getState(2, Buttons.x, State.isPressed)) {
//         // spin up
//         StateMachine.stopGroups("spinner");
//         TB_Turret.setSpinnerTargetSpeed(TB_Turret.spinnerManualSpeed);
//      }
//      if (buttonMgr.getState(2, Buttons.y, State.wasPressed)) {
//         //intake+transfer   (open gate, start both motors)
//         TB_Intake.intakeRunning = false;
//         TB_Tasks.smLaunch.restart();
//      }
//      if (buttonMgr.getState(2, Buttons.b, State.wasPressed)) {
//         // idle
//         TB_Tasks.smSpinDown.restart();
//         TB_Intake.intakeOff();
//         TB_Intake.gateClose();
//      }
//      if (buttonMgr.getState(2, Buttons.right_bumper, State.wasPressed)) {
//         // reverse
//         TB_Intake.intakeReverse();
//      }
//      if (buttonMgr.getState(2, Buttons.right_bumper, State.wasReleased)) {
//         TB_Intake.intakeOff();
//      }
//      if (buttonMgr.getState(2, Buttons.start, State.wasPressed)) {
//         // stop spin
//         StateMachine.stopGroups("spinner");
//         TB_Turret.spinOff();
//      }
//      // stop all
//      if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
//         StateMachine.stopAll();
//         TB_Turret.stop();
//         TB_Intake.stop();
//         // add drivetrain, etc
//      }
//
//      // Toggle FCD
//      if (buttonMgr.getState(1, Buttons.start, State.wasDoubleTapped)) {
//         parts.userDrive.toggleFieldCentricDrive();
//      }
//
//      // Toggle HeadingHold
//      if (buttonMgr.getState(1, Buttons.back, State.wasDoubleTapped)) {
//         parts.userDrive.toggleHeadingHold();
//      }
//
//      // Store heading correction
//      if (buttonMgr.getState(1, Buttons.right_stick_button, State.wasReleased)) {
//         parts.userDrive.setDeltaHeading();
//      }
//
//      // Toggle PositionHold
//      if (buttonMgr.getState(1, Buttons.left_stick_button, State.wasReleased))  {
//         parts.userDrive.togglePositionHold();
//      }

   }

   public void stopEverything() {
      if (!isStopped) {
         // stop parts that cause motion
         parts.drivetrain.eStop();  // note: drivedata is already zeroed in the runloop
//         parts.autoDrive.eStop();
         parts.userDrive.eStop();
         // set internal variables
         isStopped = true;
      }
   }
}
