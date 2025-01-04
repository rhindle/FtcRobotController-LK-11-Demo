package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smTransfer {

    private static int state = 0;
    private static boolean complete = false;
    private static long waitTimer;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        // This starts like park (could combine these), then...

//        Spinner Reverse for ___ ms
//        Spinner off

        if (state == 1) {                 // put it in position for safe retraction
            state++;
            SB_Intake.action(IntakeActions.PINCH_WIDEOPEN);
            SB_Intake.action(IntakeActions.SPINTAKE_PARK);
            SB_Intake.action(IntakeActions.CHUTE_PARK);
            SB_Intake.action(IntakeActions.SPINNER_OFF);
            SB_Intake.action(IntakeActions.SLIDE_RETRACT);
            SB_Intake.action(IntakeActions.LIFT_RETRACT);
        }
        if (state == 2) {                 // wait until slides are retracted
            if (SB_Intake.isSlideInTolerance() && SB_Intake.isLiftInTolerance()) {
                state++;
            }
        }
        if (state == 3) {                 // wait until servos are done moving
            if (SB_Intake.isSpintakeDone() && SB_Intake.isChuteDone()) {
                state++;
                SB_Intake.action(IntakeActions.SPINNER_OUT);
                waitTimer = System.currentTimeMillis() + 1500;   //todo: find a good time for this
            }
        }
        if (state == 4) {                 // wait for sample to transfer (no way to detect)
            if (System.currentTimeMillis() > waitTimer) {
                state++;
                SB_Intake.action(IntakeActions.SPINNER_OFF);
                SB_Intake.action(IntakeActions.SPINTAKE_DISABLE);
                SB_Intake.action(IntakeActions.CHUTE_DISABLE);
            }
        }
        if (state == 5) {                 // done
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
