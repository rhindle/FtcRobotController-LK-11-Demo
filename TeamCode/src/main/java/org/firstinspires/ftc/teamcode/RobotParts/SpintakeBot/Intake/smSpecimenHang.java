package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smSpecimenHang {

    private static int state = 0;
    private static boolean complete = false;
    private static long waitTimer;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        Hang specimen     // should already be in position, but run through anyway
//
//        Pincher must be closed
//        Lift to pipe height
//    ...
//        (pincher powered off???)
//        Lift to specimen release height
//        Pincher open (short timer before driving?)

        if (state == 1) {                 // make sure it's in starting position
            state++;
            SB_Intake.setPinchServo(SB_Intake.pinchLoose);
            SB_Intake.setLiftPosition(SB_Intake.positionLiftHangReady,1);
        }
        if (state == 2) {                 // wait parts in position
            if (SB_Intake.isLiftInTolerance() && SB_Intake.isPinchDone()) {
                state++;
                SB_Intake.setPinchServo(SB_Intake.pinchSuperLoose);
                SB_Intake.setLiftPosition(SB_Intake.positionLiftHangRelease,1);
            }
        }
        if (state == 3) {                 // wait until lift at release position
            if (SB_Intake.isLiftInTolerance()) {
                state++;
                SB_Intake.setPinchServo(SB_Intake.pinchFullOpen);
                waitTimer = System.currentTimeMillis() + 250;   //todo: find a good time for this
            }
        }
        if (state == 4) {                 // wait for pinch to release enough
            if (System.currentTimeMillis() > waitTimer) {
                state++;
            }
        }
        if (state == 5) {                 // done
            complete = true;
//            SB_Intake.action(IntakeActions.AUTO_SAFE_PARK);
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
