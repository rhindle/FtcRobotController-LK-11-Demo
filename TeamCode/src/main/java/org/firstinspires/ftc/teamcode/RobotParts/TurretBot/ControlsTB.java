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
   float speedFine = (float) 0.2;   // float on purpose!
   public boolean forza = false;
   
   // working variables
   boolean isStopped = false;
   float speedFactor = 1;    // this is float on purpose so drivedata overload is correct!
   double slideSpeed = 0;
   double liftSpeed = 0;
   float fineX = 0;
   float fineY = 0;
   float fineR = 0;

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

      speedFactor = (float)speedNormal;
      parts.userDrive.setSpeedMaximum(speedNormal);
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
         speedFactor = (float)speedFast;
         parts.userDrive.setSpeedMaximum(speedFast);
      }
      if (buttonMgr.getState(1, Buttons.left_bumper, State.isPressed)) {
         speedFactor = (float)speedSlow;
         parts.userDrive.setSpeedMaximum(speedSlow);
      }

      if (!forza) {
         driveData = new DriveData(gamepad1.left_stick_x * speedFactor,
                 gamepad1.left_stick_y * speedFactor,
                 gamepad1.right_stick_x * speedFactor);
      }
      else {
         driveData = new DriveData(gamepad1.left_trigger * speedFactor,
                 gamepad1.right_trigger * speedFactor, 
                 gamepad1.left_stick_x * speedFactor, 
                 gamepad1.right_stick_x * speedFactor);
      }


//      fineX = gamepad2.left_stick_x;
//      fineY = gamepad2.left_stick_y;
//      // if the intake driver is trying to control the robot, override the primary driver controls
//      if (Math.abs(Functions.mathHypotenuse(fineX, fineY))>0.125 || Math.abs(fineR)>0.125) {
//         driveData = new DriveData(fineX * speedFine,
//                 fineY * speedFine,
//                 fineR * speedFine);
//      }

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
      if (buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {
         // modify spin speed
         TB_Turret.spinnerManualSpeed += gamepad2.left_stick_y * -TB_Turret.spinLargeChange * TB_Turret.spinMotorRPM;
         TB_Turret.spinnerManualSpeed += gamepad2.right_stick_y * -TB_Turret.spinSmallChange * TB_Turret.spinMotorRPM;
         TB_Turret.spinnerManualSpeed = Math.max(-TB_Turret.spinMotorRPM, Math.min(TB_Turret.spinMotorRPM, TB_Turret.spinnerManualSpeed));
      }

      if (buttonMgr.getState(2, Buttons.a, State.wasPressed)) {
         // toggle intake on  (close gate, start intake)
         TB_Intake.intakeRunning = !TB_Intake.intakeRunning;
         if (TB_Intake.intakeRunning) TB_Tasks.smIntakeOn.restart();
         else TB_Tasks.smIntakeOff.restart();
//                smIntakeOn.restart();
      }
//            if (buttonMgr.getState(1, Buttons.a, State.wasReleased)) {
//                // toggle intake off  (stop intake, release pressure)
//                smIntakeOff.restart();
//            }
      if (buttonMgr.getState(2, Buttons.x, State.isPressed)) {
         // spin up
         StateMachine.stopGroups("spinner");
//         TB_Turret.setMotorSpinSpeed();
         TB_Turret.setSpinnerTargetSpeed(TB_Turret.spinnerManualSpeed);
      }
      if (buttonMgr.getState(2, Buttons.y, State.wasPressed)) {
         //intake+transfer   (open gate, start both motors)
         TB_Intake.intakeRunning = false;
         TB_Tasks.smLaunch.restart();
      }
      if (buttonMgr.getState(2, Buttons.b, State.wasPressed)) {
         // idle
//                motorSpin1.setVelocity(spinnerIdleSpeed / (60.0 / spinTicks));
//                motorSpin2.setVelocity(spinnerIdleSpeed / (60.0 / spinTicks));
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
      }
      if (buttonMgr.getState(2, Buttons.start, State.wasPressed)) {
         // stop spin
         StateMachine.stopGroups("spinner");
         TB_Turret.spinOff();
      }
      // stop all
      if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
         StateMachine.stopAll();
         TB_Turret.stop();
         TB_Intake.stop();
         // add drivetrain, etc
      }
//
//      if (!buttonMgr.getState(2,Buttons.left_bumper, State.isPressed)) {  // left bumper not pressed
//         if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.AUTO_SAFE_PARK);
//         }
//         if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.AUTO_START_SAMPLING);
//         }
//         if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.AUTO_INTAKE);
//         }
//         if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.AUTO_TRANSFER);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.AUTO_PREP_DEPOSIT);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.AUTO_DEPOSIT);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_down, State.wasSingleTapped)) {
//            SB_Intake.action(IntakeActions.SPECIMEN_GRAB_READY);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_down, State.wasDoubleTapped)) {
//            SB_Intake.action(IntakeActions.SPECIMEN_GRAB);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasSingleTapped)) {
//            SB_Intake.action(IntakeActions.SPECIMEN_HANG_READY);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasDoubleTapped)) {
//            SB_Intake.action(IntakeActions.SPECIMEN_HANG);
//         }
//      }
//      else {  // left bumper is pressed
//         if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.SPINNER_IN);
//         }
//         if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.SPINNER_OFF);
//         }
//         if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.SPINNER_OUT);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.SPINTAKE_SAFE);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.SPINTAKE_ALMOSTFLOOR);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
//            SB_Intake.action(IntakeActions.SPINTAKE_DISABLE);
//         }
//      }

      // Toggle pivot
      if (buttonMgr.getState(1, Buttons.b, State.wasHeld)) {
         parts.userDrive.setLockRear(!parts.userDrive.getLockRear());
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasHeld)) {
         parts.userDrive.setLockFront(!parts.userDrive.getLockFront());
      }

      // Toggle FCD
      if (buttonMgr.getState(1, Buttons.start, State.wasDoubleTapped)) {
         parts.userDrive.toggleFieldCentricDrive();
      }

      // Toggle HeadingHold
      if (buttonMgr.getState(1, Buttons.back, State.wasDoubleTapped)) {
         parts.userDrive.toggleHeadingHold();
      }

      // Store heading correction
      if (buttonMgr.getState(1, Buttons.right_stick_button, State.wasReleased)) {
         parts.userDrive.setDeltaHeading();
      }

      // Toggle PositionHold
      if (buttonMgr.getState(1, Buttons.left_stick_button, State.wasReleased))  {
         parts.userDrive.togglePositionHold();
      }

      // Delete this test - position queue
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld) &&
              buttonMgr.getState(1,Buttons.right_trigger, State.isHeld) &&
              buttonMgr.getState(1,Buttons.left_trigger, State.wasDoubleTapped)) {
//         parts.dsAuto.testAutoMethod();
      }
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
