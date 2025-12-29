package org.firstinspires.ftc.teamcode.RobotParts.Test2025;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;

public class ControlsT25 extends Controls {

   // settings
   double speedSlow = 0.25;
   double speedNormal = 0.5;
   double speedFast = 1.0;

   // working variables
   boolean isStopped = false;
   float speedFactor = 1;    // this is float on purpose so drivedata overload is correct!

   public ControlsT25(Parts parts) {
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

      driveData = new DriveData(gamepad1.left_stick_x * speedFactor,
              gamepad1.left_stick_y * speedFactor,
              gamepad1.right_stick_x * speedFactor);
      //forza
      //DriveData driveDataForza = new DriveData(gamepad1.left_stick_y, 0, gamepad1.left_stick_x, gamepad1.right_stick_x);

      if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
//         SB_Intake.eStop();
      }

      // turret test stuff
      if (buttonMgr.getState(2, Buttons.x, State.wasSingleTapped)) {
         parts.t25_Effector.armTurret(false);
      }
      if (buttonMgr.getState(2, Buttons.x, State.wasDoubleTapped)) {
         parts.t25_Effector.armTurret(true);
      }
      if (buttonMgr.getState(2, Buttons.y, State.wasSingleTapped)) {
         parts.t25_Effector.armSpinner(false);
      }
      if (buttonMgr.getState(2, Buttons.y, State.wasDoubleTapped)) {
         parts.t25_Effector.armSpinner(true);
      }

      // kicker test stuff
      if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
         parts.t25_Effector.intakeToggle();
      }
      if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
         parts.t25_Effector.launchKick1.restart();
      }
      if (buttonMgr.getState(1, Buttons.x, State.wasHeld)) {
         parts.t25_Effector.launchKick1a.restart();
      }
      if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
         parts.t25_Effector.launchKick2.restart();
      }
      if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
         parts.t25_Effector.launchKick3.restart();
      }
      if (buttonMgr.getState(1, Buttons.a, State.wasHeld)) {
         parts.t25_Effector.launchAll.restart();
      }
      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
         parts.t25_Effector.spinnerSlowTest();
      }
      if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
         parts.t25_Effector.spinnerOff();
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
