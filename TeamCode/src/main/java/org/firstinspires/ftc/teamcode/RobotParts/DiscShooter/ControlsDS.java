package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

public class ControlsDS extends Controls {

   boolean isStopped = false;
   boolean guestOK, teamOK;
   boolean toggleIntake = false;

   public ControlsDS(Parts parts) {
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

      DriveData driveDataTeam = new DriveData(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
      DriveData driveDataGuest = new DriveData(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);
      //forza - can't work with dead man switch, so simulate
      //DriveData driveDataTeam = new DriveData(gamepad1.left_stick_y, 0, gamepad1.left_stick_x, gamepad1.right_stick_x);

      /* Controls will be in the style of dead man switches */
      guestOK = buttonMgr.isPressed(1,Buttons.left_trigger);
      teamOK = buttonMgr.isPressed(1,Buttons.right_trigger);

      /* If neither dead man is pressed, stop everything (if needed) and proceed no further */
      if (!guestOK && !teamOK) {
         stopEverything();
         parts.dsLed.updateGraphic('3', Color.rgb(20,0,0));
         return;
      }

      /* If we made it here, things aren't necessarily stopped any more (this affect the e-stop method */
      isStopped = false;
      if (guestOK && teamOK) {
         parts.dsLed.updateGraphic('3', Color.rgb(0,0,20));
      }
      else if (guestOK) {
         parts.dsLed.updateGraphic('3', Color.rgb(0,20,0));
      }
      else {
         parts.dsLed.updateGraphic('3', Color.rgb(5,0,15));
      }

      /* If guest is allowed, start with their drive input */
      if (guestOK) {
         driveData = driveDataGuest.clone();
      }

      /* If team is allowed, override with their drive input if not 0 */
      if (teamOK) {
         if (driveDataTeam.driveSpeed > 0) {
            driveData.driveSpeed = driveDataTeam.driveSpeed;
            driveData.driveAngle = driveDataTeam.driveAngle;
         }
         if (driveDataTeam.rotate != 0) {
            driveData.rotate = driveDataTeam.rotate;
         }
      }

      /* With the most dangerous things out of the way (i.e., drivertrain motion), we can move on to the other controls */

      if (eitherGuestOrTeam(Buttons.dpad_left, State.wasSingleTapped)) {
         DSShooter.disarmShooter();
         parts.dsLed.displayMessage('S', 3);
      }

      if (eitherGuestOrTeam(Buttons.dpad_right, State.wasSingleTapped)) {
         DSShooter.armShooter();
         parts.dsLed.displayMessage('S', 2);
      }

      if (eitherGuestOrTeam(Buttons.dpad_left, State.wasDoubleTapped)) {
         DSShooter.extendPusher();
      }

      if (eitherGuestOrTeam(Buttons.dpad_right, State.wasDoubleTapped)) {
         DSShooter.retractPusher();
      }

      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasSingleTapped)) {
         DSShooter.openGate();
         parts.dsLed.displayMessage('G', 2);
      }

      if (eitherGuestOrTeam(Buttons.dpad_down, State.wasSingleTapped)) {
         DSShooter.closeGate();
         parts.dsLed.displayMessage('G', 3);
      }

      if (eitherGuestOrTeam(Buttons.left_bumper, State.wasSingleTapped)) {
         toggleIntake = !toggleIntake;
         if (toggleIntake) {
            parts.dsShooter.intakeReverse();
            parts.dsLed.displayMessage('I', 4);
            DSShooter.disarmShooter();
         }
         else {
            parts.dsShooter.intakeOff();
            parts.dsLed.displayMessage('I', 3);
         }
      }

      if (eitherGuestOrTeam(Buttons.right_bumper, State.wasSingleTapped)) {
         toggleIntake = !toggleIntake;
         if (toggleIntake) {
            parts.dsShooter.intakeOn();
            parts.dsLed.displayMessage('I', 2);
            DSShooter.disarmShooter();
         }
         else {
            parts.dsShooter.intakeOff();
            parts.dsLed.displayMessage('I', 3);
         }
      }

      if (eitherGuestOrTeam(Buttons.back, State.isHeld)) {
         parts.dsShooter.cancelStateMachines();
         parts.dsLed.displayMessage('X', 3);
      }

      if (eitherGuestOrTeam(Buttons.a, State.wasTapped)) {  // todo: Do we want team to be able to push without being armed?
         parts.dsShooter.startPushIfArmed();
      }

      if (eitherGuestOrTeam(Buttons.b, State.wasSingleTapped)) {
         parts.dsShooter.startShoot1();
         parts.dsLed.displayMessage('1', 4);
      }

      if (eitherGuestOrTeam(Buttons.x, State.wasSingleTapped)) {
         parts.dsShooter.startShoot3();
         parts.dsLed.displayMessage('3', 4);
      }

      if (eitherGuestOrTeam(Buttons.y, State.wasSingleTapped)) {
         parts.dsShooter.startFullAuto();
         parts.dsLed.displayMessage('A', 4);
      }

      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasHeld)) {
         parts.autoDrive.setNavTarget(new NavigationTarget(DSMisc.autoLaunchPos, parts.dsMisc.toleranceHigh));
         parts.userDrive.directionTarget = DSMisc.aimPosition;
         parts.userDrive.useTargetDirection = true;
         parts.dsLed.displayMessage('A', 2);
      }

      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasDoubleTapped)) {
         parts.autoDrive.setNavTarget(new NavigationTarget(DSMisc.tagReadPos, parts.dsMisc.toleranceHigh));
         parts.userDrive.useTargetDirection = false;
         parts.dsLed.displayMessage('#', 4);
      }

      if (eitherGuestOrTeam(Buttons.dpad_down, State.wasHeld)) {
         parts.userDrive.directionTarget = DSMisc.aimPosition;
         parts.dsLed.displayMessage('T', parts.userDrive.toggleUseTargetDirection());
      }

      // Toggle FCD
      if (eitherGuestOrTeam(Buttons.start, State.wasDoubleTapped)) {
         parts.dsLed.displayMessage('F', parts.userDrive.toggleFieldCentricDrive());
      }

      // Toggle HeadingHold
      if (eitherGuestOrTeam(Buttons.back, State.wasDoubleTapped)) {
         parts.dsLed.displayMessage('H', parts.userDrive.toggleHeadingHold());
      }

      // Store heading correction
      if (eitherGuestOrTeam(Buttons.right_stick_button, State.wasReleased)) {
         parts.userDrive.setDeltaHeading();
         parts.dsLed.displayMessage('D', 1);
      }

      if (eitherGuestOrTeam(Buttons.left_stick_button, State.wasReleased))  {
         parts.dsLed.displayMessage('P', parts.userDrive.togglePositionHold());
      }
   }

   public boolean eitherGuestOrTeam(Buttons button, State state) {
      // if either was enabled and activated the control, return true
      boolean team = teamOK && buttonMgr.getState(1, button, state);
      boolean guest = guestOK && buttonMgr.getState(2, button, state);
      return team || guest;
   }

   public void stopEverything() {
      if (!isStopped) {
         // stop parts that cause motion
         parts.drivetrain.eStop();  // note: drivedata is already zeroed in the runloop
         parts.dsShooter.eStop();
         parts.autoDrive.eStop();
         parts.userDrive.eStop();
         // set internal variables
         toggleIntake = false;
         isStopped = true;
      }
   }
}
