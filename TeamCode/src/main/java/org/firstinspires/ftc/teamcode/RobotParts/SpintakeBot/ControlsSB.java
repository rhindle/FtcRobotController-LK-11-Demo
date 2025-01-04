package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.Functions;

public class ControlsSB extends Controls {

   boolean isStopped = false;
   float speedFactor = 1;

   double slideSpeed = 0;
   double liftSpeed = 0;
//   double rotatorSpeed = 0;
//   double wristAngle = 0;
//   double wristMagnitude = 0;

   public ControlsSB(Parts parts) {
      super(parts);
   }

   @Override
   public void runLoop() {
      driveData = new DriveData();
      userInput();
      parts.userDrive.setUserDriveSettings(driveData);
      SB_Intake.manualSlideControl(slideSpeed*0.5);
      SB_Intake.manualLiftControl(liftSpeed*0.5);
//      parts.sb_Intake.manualRotatorControl(rotatorSpeed);
//      if (Math.abs(wristMagnitude)>0.5) parts.sb_Intake.manualWristControl(wristAngle);
   }

   @Override
   public void userInput() {

      speedFactor = 0.5f;
      parts.userDrive.setSpeedMaximum(0.5);
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
         speedFactor=1f;
         parts.userDrive.setSpeedMaximum(1);
      }
      if (buttonMgr.getState(1, Buttons.left_bumper, State.isPressed)) {
         speedFactor=0.25f;
         parts.userDrive.setSpeedMaximum(0.25);
      }

      driveData = new DriveData(gamepad1.left_stick_x * speedFactor,
              gamepad1.left_stick_y * speedFactor,
              gamepad1.right_stick_x * speedFactor);
      //forza
      //DriveData driveDataForza = new DriveData(gamepad1.left_stick_y, 0, gamepad1.left_stick_x, gamepad1.right_stick_x);

      if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
         SB_Intake.action(IntakeActions.AUTO_SAFE_PARK);
      }
      if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
         SB_Intake.action(IntakeActions.AUTO_START_SAMPLING);
      }
      if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
         SB_Intake.action(IntakeActions.AUTO_INTAKE);
      }
      if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
         SB_Intake.action(IntakeActions.AUTO_TRANSFER);
      }
      if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
         SB_Intake.action(IntakeActions.AUTO_PREP_DEPOSIT);
      }
      if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
         SB_Intake.action(IntakeActions.AUTO_DEPOSIT);
      }
      if (buttonMgr.getState(2, Buttons.dpad_down, State.wasSingleTapped)) {
         SB_Intake.action(IntakeActions.SPECIMEN_GRAB_READY);
      }
      if (buttonMgr.getState(2, Buttons.dpad_down, State.wasDoubleTapped)) {
         SB_Intake.action(IntakeActions.SPECIMEN_GRAB);
      }
      if (buttonMgr.getState(2, Buttons.dpad_left, State.wasSingleTapped)) {
         SB_Intake.action(IntakeActions.SPECIMEN_HANG_READY);
      }
      if (buttonMgr.getState(2, Buttons.dpad_left, State.wasDoubleTapped)) {
         SB_Intake.action(IntakeActions.SPECIMEN_HANG);
      }
      if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
         SB_Intake.eStop();
      }

      slideSpeed = -gamepad2.left_stick_y;
      liftSpeed = gamepad2.right_trigger - gamepad2.left_trigger;
//      rotatorSpeed = gamepad2.left_stick_x;
//      wristAngle = Math.toDegrees(Math.atan2(-gamepad2.right_stick_x, -gamepad2.right_stick_y));
//      wristMagnitude = Functions.mathHypotenuse(gamepad2.right_stick_x, gamepad2.right_stick_y);

      if (buttonMgr.getState(2, Buttons.start, State.isPressed)) SB_Intake.slideOverride = true;
      else SB_Intake.slideOverride = false;

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
