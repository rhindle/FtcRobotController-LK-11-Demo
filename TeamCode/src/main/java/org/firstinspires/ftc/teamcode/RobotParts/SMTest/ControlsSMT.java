package org.firstinspires.ftc.teamcode.RobotParts.SMTest;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;

public class ControlsSMT extends Controls {

   boolean isStopped = false;
//   float speedFactor = 1;
//
//   double slideSpeed = 0;
//   double liftSpeed = 0;
//   double rotatorSpeed = 0;
//   double wristAngle = 0;
//   double wristMagnitude = 0;

   public ControlsSMT(Parts parts) {
      super(parts);
   }

   @Override
   public void runLoop() {
//      driveData = new DriveData();
      userInput();
//      parts.userDrive.setUserDriveSettings(driveData);
//      parts.t24MultiGrabber.manualSlideControl(slideSpeed*0.5);
//      parts.t24MultiGrabber.manualLiftControl(liftSpeed*0.5);
//      parts.t24MultiGrabber.manualRotatorControl(rotatorSpeed);
//      if (Math.abs(wristMagnitude)>0.5) parts.t24MultiGrabber.manualWristControl(wristAngle);
   }

   @Override
   public void userInput() {

      if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
         parts.smt_LED.machine1.start();
      }
      if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
         parts.smt_LED.machine2.start();
      }
      if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
         parts.smt_LED.machine3.start();
      }
      if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
         parts.smt_LED.machine4.start();
      }
      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
         parts.smt_LED.machine5.restart();
      }
      if (buttonMgr.getState(1, Buttons.back, State.wasTapped)) {
         parts.smt_LED.stop();
      }

//      speedFactor = 0.5f;
//      parts.userDrive.setSpeedMaximum(0.5);
//      if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
//         speedFactor=1f;
//         parts.userDrive.setSpeedMaximum(1);
//      }
//      if (buttonMgr.getState(1, Buttons.left_bumper, State.isPressed)) {
//         speedFactor=0.25f;
//         parts.userDrive.setSpeedMaximum(0.25);
//      }
//
//      driveData = new DriveData(gamepad1.left_stick_x * speedFactor,
//              gamepad1.left_stick_y * speedFactor,
//              gamepad1.right_stick_x * speedFactor);
//      //forza
//      //DriveData driveDataForza = new DriveData(gamepad1.left_stick_y, 0, gamepad1.left_stick_x, gamepad1.right_stick_x);
//
////      if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
////         parts.t24Grabber.grabberSafe();
////      }
////      if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
////         parts.t24Grabber.grabberArmed();
////      }
////      if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
////         parts.t24Grabber.grabberStartGrab();
////      }
////      if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
////         parts.t24Grabber.grabberGrab();
////      }
////      if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
////         parts.t24Grabber.grabberVertical();
////      }
////      if (buttonMgr.getState(1, Buttons.dpad_right, State.wasTapped)) {
////         parts.t24Grabber.grabberMaxBack();
////      }
////      if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
////         parts.t24Grabber.grabberRelease();
////      }
//
//      slideSpeed = -gamepad2.left_stick_y;
//      liftSpeed = gamepad2.right_trigger - gamepad2.left_trigger;
//      rotatorSpeed = gamepad2.left_stick_x;
//      wristAngle = Math.toDegrees(Math.atan2(-gamepad2.right_stick_x, -gamepad2.right_stick_y));
//      wristMagnitude = Functions.mathHypotenuse(gamepad2.right_stick_x, gamepad2.right_stick_y);
//
//      if (buttonMgr.getState(2, Buttons.left_bumper, State.isPressed)) {
//         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
//            T24MultiGrabber.action(IntakeActions.AUTO_EXTEND_TO_GRAB);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
//            T24MultiGrabber.action(IntakeActions.AUTO_GRAB_AND_INSPECT);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
//            T24MultiGrabber.action(IntakeActions.AUTO_RETRACT);
//         }
////         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasSingleTapped)) {
////            T24MultiGrabber.action(IntakeActions.AUTO_MAKE_SPACE);
////         }
//         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasDoubleTapped)) {
//            T24MultiGrabber.action(IntakeActions.AUTO_TRANSFER);
//         }
////         if (buttonMgr.getState(2,Buttons.dpad_down, State.wasTapped)) {
////            T24MultiGrabber.action(IntakeActions.SHOULDER_ALLBACK);
////         }
//      } else {
//         if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
//            T24MultiGrabber.action(IntakeActions.SAFE_IN);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
//            T24MultiGrabber.action(IntakeActions.SAFE_OUT);
//         }
//         if (buttonMgr.getState(2, Buttons.dpad_left, State.wasTapped)) {
//            T24MultiGrabber.action(IntakeActions.DROP_SAMPLE);
//         }
//      }
//      if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
//         T24MultiGrabber.action(IntakeActions.GRAB_HOVER);
//      }
//      if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
//         T24MultiGrabber.action(IntakeActions.GRAB_OPEN);
//      }
//      if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
//         T24MultiGrabber.action(IntakeActions.GRAB_CLOSE);
//      }
//      if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
//         T24MultiGrabber.action(IntakeActions.SHOULDER_PUSH);
//      }
//      if (buttonMgr.getState(2, Buttons.start, State.isPressed)) T24MultiGrabber.slideOverride = true;
//      else T24MultiGrabber.slideOverride = false;
//
//      // Toggle pivot
//      if (buttonMgr.getState(1, Buttons.b, State.wasHeld)) {
//         parts.userDrive.setLockRear(!parts.userDrive.getLockRear());
//      }
//      if (buttonMgr.getState(1, Buttons.y, State.wasHeld)) {
//         parts.userDrive.setLockFront(!parts.userDrive.getLockFront());
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
//
//      // Delete this test - position queue
//      if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld) &&
//              buttonMgr.getState(1,Buttons.right_trigger, State.isHeld) &&
//              buttonMgr.getState(1,Buttons.left_trigger, State.wasDoubleTapped)) {
////         parts.dsAuto.testAutoMethod();
//      }
   }

   public void stopEverything() {
      if (!isStopped) {
//         // stop parts that cause motion
//         parts.drivetrain.eStop();  // note: drivedata is already zeroed in the runloop
////         parts.autoDrive.eStop();
//         parts.userDrive.eStop();
//         // set internal variables
         isStopped = true;
      }
   }
}
