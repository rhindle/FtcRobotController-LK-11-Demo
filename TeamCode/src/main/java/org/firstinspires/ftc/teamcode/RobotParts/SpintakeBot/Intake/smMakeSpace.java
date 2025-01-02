package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smMakeSpace {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        if (state == 1) {                 // make sure shoulder is ready
//            SB_Intake.action(IntakeActions.GRAB_WIDEOPEN);
//            SB_Intake.action(IntakeActions.SHOULDER_DRAG);
//            state++;
//        }
//        if (state == 2) {                 // sweep left
//            if (SB_Intake.isShoulderDone()) {
//                state++;
//                SB_Intake.setRotatorServo(SB_Intake.rotator45Left);
//            }
//        }
//        if (state == 3) {                 // sweep right
//            if (SB_Intake.isRotatorDone()) {
//                state++;
//                SB_Intake.setRotatorServo(SB_Intake.rotator45Right);
//            }
//        }
//        if (state == 4) {                 // sweep center
//            if (SB_Intake.isRotatorDone()) {
//                state++;
//                SB_Intake.setRotatorServo(SB_Intake.rotatorCenter);
//            }
//        }
//        if (state == 5) {                 // wait until centered
//            if (SB_Intake.isRotatorDone()) {
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
