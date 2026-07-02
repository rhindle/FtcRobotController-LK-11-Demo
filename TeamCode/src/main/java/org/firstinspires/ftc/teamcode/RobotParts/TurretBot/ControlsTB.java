package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;

public class ControlsTB extends Controls {

   // settings
   /* speed settings moved to TB_Misc */
//   double speedSlow = 0.25;
//   double speedNormal = 0.5;
//   double speedFast = 1.0;

   // working variables
   boolean isStopped = false;
//   boolean fullSpeed = false;
//   float speedFactor = 1;    // this is float on purpose so drivedata overload is correct!
//   float speedFactor = (float)TB_Misc.speedNormal;

   boolean teamOK = false;
   boolean guestDriveOK = false;
   boolean guestShootOK = false;

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
         case 10:
            DemoControls();
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
         if (TB_Misc.controlsFullSpeed) TB_Misc.speedFactor = (float)TB_Misc.speedFast;
         else TB_Misc.speedFactor = (float)TB_Misc.speedNormal;
         parts.userDrive.setSpeedMaximum(TB_Misc.speedFactor);
      }

      if (!parts.useForzaControls) {
         driveData = new DriveData(gamepad1.left_stick_x * TB_Misc.speedFactor,
                 gamepad1.left_stick_y * TB_Misc.speedFactor,
                 gamepad1.right_stick_x * TB_Misc.speedFactor);
      }
      else {
         driveData = new DriveData(gamepad1.left_trigger * TB_Misc.speedFactor,
                 gamepad1.right_trigger * TB_Misc.speedFactor,
                 gamepad1.right_stick_x * TB_Misc.speedFactor * (float)0.7,   // forced lower speed @ AV request
                 gamepad1.left_stick_x * TB_Misc.speedFactor);
      }

      // turret test stuff
      if (buttonMgr.getState(1, Buttons.x, State.wasSingleTapped)) {
         TB_Turret.armTurret(true);
      }
      if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)) {
         TB_Turret.armTurret(false);
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasSingleTapped)) {
         TB_Turret.armSpinner(true);
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
         TB_Turret.armSpinner(false);
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
      if (buttonMgr.getState(1, Buttons.b, State.wasHeld)) {
         //intake+transfer   (open gate, start both motors)  [override]
         TB_Intake.intakeRunning = false;
         TB_Tasks.smLaunch.restart();
      }

      //normal operation
      if (!buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {

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
         if (buttonMgr.getState(2, Buttons.x, State.wasPressed)) {
            TB_LL.toggleAuto();
         }
         if (buttonMgr.getState(2, Buttons.y, State.wasPressed)) {
            TB_LL.applyTransform();
         }
         if (buttonMgr.getState(2, Buttons.a, State.wasPressed)) {
            // toggle intake on  (close gate, start intake)
            TB_Intake.intakeRunning = !TB_Intake.intakeRunning;
            if (TB_Intake.intakeRunning) TB_Tasks.smIntakeOn.restart();
            else TB_Tasks.smIntakeOff.restart();
         }
         if (buttonMgr.getState(2, Buttons.b, State.wasPressed)) {
            //intake+transfer   (open gate, start both motors)  [only if in range]
            if (!TB_Turret.turretOutOfRange) {
               TB_Intake.intakeRunning = false;
               TB_Tasks.smLaunch.restart();
            }
         }
         if (buttonMgr.getState(2, Buttons.b, State.wasHeld)) {
            //intake+transfer   (open gate, start both motors)  [override]
            TB_Intake.intakeRunning = false;
            TB_Tasks.smLaunch.restart();
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
      // emergency overrides with left bumper
      else {
         // modify spin speed
         TB_Turret.spinnerManualSpeed += gamepad2.left_stick_y * -TB_Turret.spinLargeChange * TB_Turret.spinMotorRPM;
         TB_Turret.spinnerManualSpeed += gamepad2.right_stick_y * -TB_Turret.spinSmallChange * TB_Turret.spinMotorRPM;
         TB_Turret.spinnerManualSpeed = Math.max(-TB_Turret.spinMotorRPM, Math.min(TB_Turret.spinMotorRPM, TB_Turret.spinnerManualSpeed));

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
         if (buttonMgr.getState(2, Buttons.x, State.isPressed)) {
            // spin up
            StateMachine.stopGroups("spinner");
            TB_Turret.setSpinnerTargetSpeed(TB_Turret.spinnerManualSpeed);
         }
         if (buttonMgr.getState(2, Buttons.y, State.wasPressed)) {
            TB_Intake.disableSensors(true);
         }
         if (buttonMgr.getState(2, Buttons.y, State.wasHeld)) {
            TB_Intake.disableSensors(false);
         }

//         if (buttonMgr.getState(2, Buttons.a, State.wasPressed)) {
//            // idle
//            TB_Tasks.smSpinDown.restart();
//            TB_Intake.intakeOff();
//            TB_Intake.gateClose();
//         }
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

      if (buttonMgr.getState(1, Buttons.dpad_right, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoTestGotoCenter.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearGotoShoot.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_left, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearSpike1.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_down, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearSpike2.restart();
      }
      if (buttonMgr.getState(1, Buttons.left_bumper, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearSpike3.restart();
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearGotoGate.restart();
      }
      if (buttonMgr.getState(1, Buttons.x, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearOperateGate.restart();
      }
      if (buttonMgr.getState(1, Buttons.a, State.wasSingleTapped)) {
         TB_Tasks.smLaunch.restart();
      }
      if (buttonMgr.getState(1, Buttons.b, State.wasSingleTapped)) {
         TB_Turret.armTurret(true);
         TB_Turret.armSpinner(true);
      }
      if (buttonMgr.getState(1, Buttons.right_bumper, State.wasSingleTapped)) {
         TB_Turret.armTurret(false);
         TB_Turret.armSpinner(false);
      }
      if (buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
         StateMachine.stopAll();
         TB_Turret.stop();
         TB_Intake.stop();
      }
      if (buttonMgr.getState(1, Buttons.left_trigger, State.wasSingleTapped)) {
         TB_TasksAuto.smAutoNearOrchestrator.restart();
      }

      //Far specific

      if (buttonMgr.getState(1, Buttons.left_trigger, State.wasDoubleTapped)) {
         TB_TasksAutoFar.smAutoFarOrchestrator.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_right, State.wasDoubleTapped)) {
         TB_TasksAutoFar.smAutoFarGotoShoot180.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasDoubleTapped)) {
         TB_TasksAutoFar.smAutoFarGotoShoot90.restart();
      }

      if (buttonMgr.getState(1, Buttons.dpad_left, State.wasDoubleTapped)) {
         TB_TasksAutoFar.smAutoFarSpike3.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_down, State.wasDoubleTapped)) {
         TB_TasksAutoFar.smAutoFarHumanArea.restart();
      }


      // Delete this test - position queue
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld) &&
              buttonMgr.getState(1,Buttons.right_trigger, State.wasDoubleTapped)) {
         TB_TasksAuto.smAutoTestTransitions.restart();
      }

   }

   //~~~~~~~~~~~~~~~~~~~~~~~~ DEMO CONTROLS ~~~~~~~~~~~~~~~~~~~~~~~~//

   /* ==========================================================================================

   Controller 1 is Team Controller
      left bumper allows Guest to drive
      right bumper allows Guest to shoot (and drive)
      Team is faster

   Controller 2 is Guest Controller
      Guest is slower

   Stored in TB_Misc:
      TB_Misc.demoTeamSpeedFast = 1.0;
      TB_Misc.demoTeamSpeedSlow = 0.5;
      TB_Misc.demoGuestSpeed = 0.3;

   ============================================================================================= */

   public void DemoControls() {

//      TB_Misc.speedFactor = (float)TB_Misc.demoTeamSpeedFast; // main driver speed always "fast" unless a control added

      // ~~~~ dead-man controls
      guestDriveOK = false;
      guestShootOK = false;
      if (buttonMgr.getState(1, Buttons.left_bumper, State.isPressed)) {
         teamOK = true;
         guestDriveOK = true;
      }
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
         teamOK = true;
         guestDriveOK = true;
         guestShootOK = true;
      }


      // ~~~~ drivetrain
         // if driver 1 is doing anything, override; otherwise accept driver 2 input
      if (teamOK) {
         parts.userDrive.setSpeedMaximum(TB_Misc.speedFactor);
         driveData = new DriveData(gamepad1.left_stick_x * TB_Misc.speedFactor,
                 gamepad1.left_stick_y * TB_Misc.speedFactor,
                 gamepad1.right_stick_x * TB_Misc.speedFactor * (float) 0.7);
      }
      if (guestDriveOK && driveData.driveSpeed == 0 && driveData.rotate == 0) {
         parts.userDrive.setSpeedMaximum(TB_Misc.demoGuestSpeed);
         driveData = new DriveData(gamepad2.left_stick_x * (float)TB_Misc.demoGuestSpeed,
                 gamepad2.left_stick_y * (float)TB_Misc.demoGuestSpeed,
                 gamepad2.right_stick_x * (float)TB_Misc.demoGuestSpeed * (float) 0.7);
      }

      // ~~~~ guest controls
      if (guestDriveOK) {
         if (buttonMgr.getState(2, Buttons.a, State.wasPressed)) {
            // toggle intake on  (close gate, start intake)
            TB_Intake.intakeRunning = !TB_Intake.intakeRunning;
            if (TB_Intake.intakeRunning) TB_Tasks.smIntakeOn.restart();
            else TB_Tasks.smIntakeOff.restart();
         }
      }
      if (guestShootOK) {
         if (buttonMgr.getState(2, Buttons.b, State.wasPressed)) {
            //intake+transfer   (open gate, start both motors)  [only if in range]
            if (!TB_Turret.turretOutOfRange) {
               TB_Intake.intakeRunning = false;
               TB_Tasks.smLaunch.restart();
            }
         }
         if (buttonMgr.getState(2, Buttons.b, State.wasHeld)) {  // is this wanted?
            //intake+transfer   (open gate, start both motors)  [override]
            TB_Intake.intakeRunning = false;
            TB_Tasks.smLaunch.restart();
         }
      }

      // ~~~~ team controls
      // ~~ always available, but unshifted
      if (!buttonMgr.getState(1, Buttons.back, State.isPressed)) {
         if (buttonMgr.getState(1, Buttons.dpad_left, State.wasPressed)) teamOK = true;
         if (buttonMgr.getState(1, Buttons.dpad_right, State.wasPressed)) teamOK = false;
         if (buttonMgr.getState(1, Buttons.dpad_up, State.wasPressed))
            TB_Misc.speedFactor = (float)TB_Misc.demoTeamSpeedFast;
         if (buttonMgr.getState(1, Buttons.dpad_down, State.wasPressed))
            TB_Misc.speedFactor = (float)TB_Misc.demoTeamSpeedSlow;
      }

      // ~~ unshifted (back button not pressed)
      if (teamOK && !buttonMgr.getState(1, Buttons.back, State.isPressed)) {
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
         if (buttonMgr.getState(1, Buttons.b, State.wasHeld)) {
            //intake+transfer   (open gate, start both motors)  [override]
            TB_Intake.intakeRunning = false;
            TB_Tasks.smLaunch.restart();
         }

         if (buttonMgr.getState(1, Buttons.x, State.wasSingleTapped)) {
            TB_Turret.armTurret(true);
         }
         if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)) {
            TB_Turret.armTurret(false);
         }
         if (buttonMgr.getState(1, Buttons.y, State.wasSingleTapped)) {
            TB_Turret.armSpinner(true);
         }
         if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
            TB_Turret.armSpinner(false);
         }

         if (buttonMgr.getState(1, Buttons.back, State.wasDoubleTapped)) {
            // stop spin
            StateMachine.stopGroups("spinner");
            TB_Turret.spinOff();
         }

         if (buttonMgr.getState(1, Buttons.left_trigger, State.isPressed) &&
                 buttonMgr.getState(1, Buttons.right_trigger, State.isPressed)) {
            TB_Turret.manualTurretOffest = 0;
         } else if (buttonMgr.getState(1, Buttons.left_trigger, State.wasPressed)) {
            TB_Turret.manualTurretOffest += 1;
         } else if (buttonMgr.getState(1, Buttons.right_trigger, State.wasPressed)) {
            TB_Turret.manualTurretOffest -= 1;
         }
      }

      // ~~ shifted (back button is pressed)
      else if (teamOK) {
         if (buttonMgr.getState(1, Buttons.dpad_up, State.wasPressed)) {
            TB_Turret.manualOverride(100);
         }
         if (buttonMgr.getState(1, Buttons.dpad_left, State.wasPressed)) {
            TB_Turret.manualOverride(75);
         }
         if (buttonMgr.getState(1, Buttons.dpad_down, State.wasPressed)) {
            TB_Turret.manualOverride(140);
         }
         if (buttonMgr.getState(1, Buttons.dpad_right, State.wasPressed)) {
            TB_Turret.manualOverride(120);
         }
         if (buttonMgr.getState(1, Buttons.x, State.wasPressed)) {
            TB_LL.toggleAuto();
         }
         if (buttonMgr.getState(1, Buttons.y, State.wasPressed)) {
            TB_LL.applyTransform();
         }
         if (buttonMgr.getState(1, Buttons.a, State.wasPressed)) {
            // reverse
            TB_Intake.intakeReverse();
         }
         if (buttonMgr.getState(1, Buttons.a, State.wasReleased)) {
            TB_Intake.intakeOff();
            TB_Intake.clearSensorFlags();
         }
         if (buttonMgr.getState(1, Buttons.b, State.wasPressed)) {
            TB_Intake.disableSensors(true);
         }
         if (buttonMgr.getState(1, Buttons.b, State.wasHeld)) {
            TB_Intake.disableSensors(false);
         }
      }

      // ~~~~ stop all (always works regardless of dead man flags)
      if (buttonMgr.getState(1, Buttons.start, State.wasPressed) || buttonMgr.getState(2, Buttons.start, State.wasPressed)) {
         StateMachine.stopAll();
         TB_Turret.stop();
         TB_Intake.stop();
         parts.drivetrain.eStop();
         parts.autoDrive.eStop();
         parts.userDrive.eStop();
         // add drivetrain, etc
      }
   }

   //~~~~~~~~~~~~~~~~~~~~~~~~ TEST CONTROLS ~~~~~~~~~~~~~~~~~~~~~~~~//

   public void TestControls() {

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
