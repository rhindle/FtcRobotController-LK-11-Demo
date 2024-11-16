package org.firstinspires.ftc.teamcode.RobotParts.Test2024;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber.IntakeActions;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;

public class ControlsT24 extends Controls {

   boolean isStopped = false;
   float speedFactor = 1;

   public ControlsT24(Parts parts) {
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

//      if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
//         parts.t24Grabber.grabberSafe();
//      }
//      if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
//         parts.t24Grabber.grabberArmed();
//      }
//      if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
//         parts.t24Grabber.grabberStartGrab();
//      }
//      if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
//         parts.t24Grabber.grabberGrab();
//      }
//      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
//         parts.t24Grabber.grabberVertical();
//      }
//      if (buttonMgr.getState(1, Buttons.dpad_right, State.wasTapped)) {
//         parts.t24Grabber.grabberMaxBack();
//      }
//      if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
//         parts.t24Grabber.grabberRelease();
//      }

      if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
         T24MultiGrabber.action(IntakeActions.SAFE_IN);
      }
      if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
         T24MultiGrabber.action(IntakeActions.SAFE_OUT);
      }
      if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
         T24MultiGrabber.action(IntakeActions.GRAB_HOVER);
      }
      if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
         T24MultiGrabber.action(IntakeActions.GRAB_OPEN);
      }
      if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
         T24MultiGrabber.action(IntakeActions.GRAB_CLOSE);
      }
      if (buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {
         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasTapped)) {
            T24MultiGrabber.action(IntakeActions.AUTO_EXTEND_TO_GRAB);
         }
         if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
            T24MultiGrabber.action(IntakeActions.AUTO_GRAB_AND_RETRACT);
         }
         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
            T24MultiGrabber.action(IntakeActions.AUTO_MAKE_SPACE);
         }
      }

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
