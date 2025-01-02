package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smGrabAndRetract {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        if (state == 1) {                 // make sure shoulder is ready
//            SB_Intake.action(IntakeActions.SHOULDER_CAPTURE);
//            state++;
//        }
//        if (state == 2) {                 // activate the grabber
//            if (SB_Intake.isShoulderDone()) {
//                state++;
//                SB_Intake.action(IntakeActions.GRAB_CLOSE);
//            }
//        }
//        if (state == 3) {                 // when grabber is done, put in safe position and retract to safe point
//            if (SB_Intake.isPinchDone()) {
//                state++;
//                SB_Intake.action(IntakeActions.SAFE_OUT);
//                SB_Intake.setSlidePosition(SB_Intake.positionSlidePitMin,1);
//            }
//        }
//        if (state == 4) {                 // if shoulder is in safe position, retract the rest of the way
//            if (SB_Intake.isShoulderDone()) {
//                state++;
//                SB_Intake.setSlidePosition(SB_Intake.positionSlideMin,1);
//            }
//        }
//        if (state == 5) {                 // wait until slide is retracted
//            if (SB_Intake.isSlideInTolerance()) {
//
//                state++;
//            }
//        }
//        if (state == 6) {                 // all done
//            complete = true;
//        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
    }

    public static void stop() {
        // make it safe?
        SB_Intake.action(IntakeActions.SAFE_OUT);
        state = -1;
    }

    public static void mildStop() {
        state = -1;
    }

    public static boolean isRunning() {
        return (state>0);
    }

    public static boolean isComplete() {
        return complete;
    }

    public static int getState() {
        return state;
    }
}
