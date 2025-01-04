package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smDeposit {

    private static int state = 0;
    private static boolean complete = false;
    private static long waitTimer;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        // This should already be ready, but just in case, walk through everything again!

//        Spinner must be off
//        (if not, stop and wait ___ ms)
//        Banana to near level
//        Lift to high position
//        NEW STUFF ... then
//        Banana to high position for ___ ms
//        SAFE

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
        if (state == 3) {                 // wait until lift is done moving and chute is levelled
            if (SB_Intake.isLiftInTolerance() && SB_Intake.isChuteDone()) {
                state++;
                SB_Intake.action(IntakeActions.CHUTE_DROP);
            }
        }
        if (state == 4) {                 // wait for chute to turn
            if (SB_Intake.isChuteDone()) {
                state++;
                waitTimer = System.currentTimeMillis() + 2000;   //todo: find a good time for this
            }
        }
        if (state == 5) {                 // wait for sample to slide
            if (System.currentTimeMillis() > waitTimer) {
                state++;
            }
        }
        if (state == 6) {                 // park
            complete = true;
            SB_Intake.action(IntakeActions.AUTO_SAFE_PARK);
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
