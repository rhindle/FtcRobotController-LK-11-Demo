package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSLed.MessageColor;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

public class ControlsDS extends Controls {

   boolean isStopped = false;
   boolean guestOK, teamOK;

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
      guestOK = buttonMgr.isPressed(1,Buttons.left_bumper);
      teamOK = buttonMgr.isPressed(1,Buttons.right_bumper);

      /* If neither dead man is pressed, stop everything (if needed) and proceed no further */
      if (!guestOK && !teamOK) {
         stopEverything();
         parts.dsLed.updateGraphic('3', MessageColor.G_RED);
         return;
      }

      /* If we made it here, things aren't necessarily stopped any more (this affect the e-stop method */
      isStopped = false;
      if (guestOK && teamOK) {
         parts.dsLed.updateGraphic('3', MessageColor.G_BLUE);
      }
      else if (guestOK) {
         parts.dsLed.updateGraphic('3', MessageColor.G_GREEN);
      }
      else {
         parts.dsLed.updateGraphic('3', MessageColor.G_PURPLE);
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
         parts.dsLed.displayMessage('S', DSLed.MessageColor.RED);
      }

      if (eitherGuestOrTeam(Buttons.dpad_right, State.wasSingleTapped)) {
         DSShooter.armShooter();
         parts.dsLed.displayMessage('S', DSLed.MessageColor.GREEN);
      }

      if (eitherGuestOrTeam(Buttons.left_trigger, State.wasTapped)) {
         if (DSShooter.intakeState == 0) {
            parts.dsShooter.intakeReverse();
            parts.dsLed.displayMessage('I', DSLed.MessageColor.YELLOW);
            DSShooter.disarmShooter();
         }
         else {
            parts.dsShooter.intakeOff();
            parts.dsLed.displayMessage('I', DSLed.MessageColor.RED);
         }
      }

      if (eitherGuestOrTeam(Buttons.right_trigger, State.wasTapped)) {
         if (DSShooter.intakeState == 0) {
            parts.dsShooter.intakeOn();
            parts.dsLed.displayMessage('I', DSLed.MessageColor.GREEN);
            DSShooter.disarmShooter();
         }
         else {
            parts.dsShooter.intakeOff();
            parts.dsLed.displayMessage('I', DSLed.MessageColor.RED);
         }
      }

      if (eitherGuestOrTeam(Buttons.back, State.isHeld)) {
         parts.dsShooter.cancelStateMachines();
      }

      // Drive to shoot position and activate target following
      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasHeld)) {
         parts.autoDrive.setNavTarget(new NavigationTarget(DSMisc.autoLaunchPos, parts.dsMisc.toleranceHigh));
         parts.userDrive.directionTarget = DSMisc.aimPosition;
         parts.userDrive.useTargetDirection = true;
         parts.dsLed.displayMessage('A', DSLed.MessageColor.GREEN);
      }

      // Drive to AprilTag read position
      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasDoubleTapped)) {
         parts.autoDrive.setNavTarget(new NavigationTarget(DSMisc.tagReadPos, parts.dsMisc.toleranceHigh));
         parts.userDrive.useTargetDirection = false;
         parts.dsLed.displayMessage('#', DSLed.MessageColor.BLUE);
         DSMisc.firstLock = true;
      }

      /* All shooting functions require additional permission: Guest+Team (or just Team) */

      if (teamOrTeamAndGuest(Buttons.a, State.wasTapped)) {
         parts.dsShooter.startPushIfArmed();
      }

      if (teamOrTeamAndGuest(Buttons.b, State.wasTapped)) {
         parts.dsShooter.startShoot1();
         parts.dsLed.displayMessage('1', DSLed.MessageColor.BLUE);
      }

      if (teamOrTeamAndGuest(Buttons.x, State.wasTapped)) {
         parts.dsShooter.startShoot3();
         parts.dsLed.displayMessage('3', DSLed.MessageColor.BLUE);
      }

      if (teamOrTeamAndGuest(Buttons.y, State.wasTapped)) {
         parts.dsShooter.startFullAuto();
         if (DSShooter.getStateFullAuto()>0) parts.dsLed.displayMessage('A', MessageColor.BLUE);
         else parts.dsLed.displayMessage('A', MessageColor.YELLOW);
      }

      /* Special controls only available to Team, not Guest */

      if (teamControl(Buttons.dpad_left, State.wasDoubleTapped)) {
         DSShooter.extendPusher();
      }

      if (teamControl(Buttons.dpad_right, State.wasDoubleTapped)) {
         DSShooter.retractPusher();
      }

      if (teamControl(Buttons.dpad_up, State.wasSingleTapped)) {
         DSShooter.openGate();
         parts.dsLed.displayMessage('G', DSLed.MessageColor.GREEN);
      }

      if (teamControl(Buttons.dpad_down, State.wasSingleTapped)) {
         DSShooter.closeGate();
         parts.dsLed.displayMessage('G', DSLed.MessageColor.RED);
      }

      // Toggle TargetDirection aiming
      if (teamControl(Buttons.dpad_down, State.wasHeld)) {
         parts.userDrive.directionTarget = DSMisc.aimPosition;
         parts.dsLed.displayMessage('T', parts.userDrive.toggleUseTargetDirection());
      }

      // Toggle FCD
      if (teamControl(Buttons.start, State.wasDoubleTapped)) {
         parts.dsLed.displayMessage('F', parts.userDrive.toggleFieldCentricDrive());
      }

      // Toggle HeadingHold
      if (teamControl(Buttons.back, State.wasDoubleTapped)) {
         parts.dsLed.displayMessage('H', parts.userDrive.toggleHeadingHold());
      }

      // Store heading correction
      if (teamControl(Buttons.right_stick_button, State.wasReleased)) {
         parts.userDrive.setDeltaHeading();
         parts.dsLed.displayMessage('D', DSLed.MessageColor.GRAY);
      }

      // Toggle PositionHold
      if (teamControl(Buttons.left_stick_button, State.wasReleased))  {
         parts.dsLed.displayMessage('P', parts.userDrive.togglePositionHold());
      }

      // Delete this test - position queue
      if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld) &&
              buttonMgr.getState(1,Buttons.right_trigger, State.isHeld) &&
              buttonMgr.getState(1,Buttons.left_trigger, State.wasDoubleTapped)) {
         parts.autoDrive.addNavTargets(new NavigationTarget[]{
                 new NavigationTarget(new Position(-28,-1,0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
                 new NavigationTarget(new Position(-15, -1, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
                 new NavigationTarget(new Position(-12, -13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,false),
                 new NavigationTarget(new Position(-30, -13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
                 new NavigationTarget(new Position(-12, 13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
                 new NavigationTarget(new Position(-24, 13, 0), parts.dsMisc.toleranceTransition, 1.0,5000,true),
                 new NavigationTarget(new Position(-33, -4, 0), parts.dsMisc.toleranceHigh, 1.0,5000,false),
                 });
//   NavigationTarget test = new NavigationTarget(new Position(-24,0,0), toleranceTransition,1.0,5000,true);
      }
   }

   public boolean eitherGuestOrTeam(Buttons button, State state) {
      // if either was enabled and activated the control, return true
      boolean team = teamOK && buttonMgr.getState(1, button, state);
      boolean guest = guestOK && buttonMgr.getState(2, button, state);
      return team || guest;
   }

   public boolean teamOrTeamAndGuest(Buttons button, State state) {
      // requires either team or team+guest (additional lockout for shooting functions)
      boolean team = teamOK && buttonMgr.getState(1, button, state);
      boolean guest = guestOK && teamOK && buttonMgr.getState(2, button, state);
      return team || guest;
   }

   public boolean teamControl(Buttons button, State state) {
      return teamOK && buttonMgr.getState(1, button, state);
   }

   public void stopEverything() {
      if (!isStopped) {
         // stop parts that cause motion
         parts.drivetrain.eStop();  // note: drivedata is already zeroed in the runloop
         parts.dsShooter.eStop();
         parts.autoDrive.eStop();
         parts.userDrive.eStop();
         parts.dsAuto.eStop();
         // set internal variables
         isStopped = true;
      }
   }
}
