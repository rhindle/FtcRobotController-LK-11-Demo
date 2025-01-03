package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smSafePark {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        Open Pincher to Max
//        Shoulder to Park
//        Lift to 0
//        Spintake to Park
//        Spinner off
//        Slide to 0

        if (state == 1) {                 // put it in position for safe retraction
                state++;
                SB_Intake.action(IntakeActions.PINCH_WIDEOPEN);
                SB_Intake.action(IntakeActions.SPINTAKE_PARK);
                SB_Intake.action(IntakeActions.CHUTE_PARK);
                SB_Intake.action(IntakeActions.SPINNER_OFF);
                SB_Intake.action(IntakeActions.SLIDE_RETRACT);
                SB_Intake.action(IntakeActions.LIFT_RETRACT);
        }
        if (state == 2) {                 // wait until slide is retracted
            if (SB_Intake.isSlideInTolerance()) {
                state++;
            }
        }
        if (state == 3) {                 // wait until lift is retracted
            if (SB_Intake.isLiftInTolerance()) {
                state++;
            }
        }
        if (state == 4) {
            SB_Intake.action(IntakeActions.SPINTAKE_DISABLE);
            SB_Intake.action(IntakeActions.CHUTE_DISABLE);
            complete = true;
        }
    }
    //----State Machine End-----

    public static void start() {
        SB_Intake.cancelStateMachines();
        complete = false;
        state = 1;
    }

    public static void stop() {
        // make it safe?
//        SB_Intake.action(IntakeActions.SAFE_OUT);
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
