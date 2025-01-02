package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smStartSampling {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        if (state == 1) {                 // put servos in safe positions
//            SB_Intake.action(IntakeActions.SAFE_IN);
//            state++;
//        }
//        if (state == 2) {                 // wait for servos to complete, extend
//            if (SB_Intake.isShoulderDone() && SB_Intake.isPinchDone()) {
//                state++;
//                SB_Intake.setSlidePosition(SB_Intake.positionSlideStartIntake,1);
//            }
//        }
//        if (state == 3) {                // wait for slide to extend paste safe point
//            if (SB_Intake.isSlideInTolerance()) { //   isSlideInsidePit()) {  // i don't like this anymore
//                state++;
//                SB_Intake.action(IntakeActions.GRAB_HOVER);
//            }
//        }
//        if (state == 4) {                // wait for all important actions to complete
//            if (SB_Intake.isSlideInTolerance() && SB_Intake.isShoulderDone()) {
//                state++;
//            }
//        }
//        if (state == 5) {               // all done
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
