package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smPrepareDeposit {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        Spinner must be off
//        (if not, stop and wait ___ ms)
//        Banana to near level
//        Lift to high position
//        wait for user confirmation? ...

        if (state == 1) {                 // be sure the spinner is off
                state++;
                SB_Intake.action(IntakeActions.SPINNER_OFF);
        }
        if (state == 2) {                 // wait until spinner is surely stopped
            if (SB_Intake.isSpintakeDone()) {
                state++;
                SB_Intake.action(IntakeActions.CHUTE_READY);
                SB_Intake.setLiftPosition(SB_Intake.positionLiftMax,1);
            }
        }
        if (state == 3) {                 // wait until lift is done moving
            if (SB_Intake.isLiftInTolerance()) {
                state++;
            }
        }
        if (state == 4) {                 // ready for drop!
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
        // make it safer somehow?
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
