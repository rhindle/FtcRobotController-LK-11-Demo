package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smRetract {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        if (state == 1) {                 // put it in position for safe retraction
//                state++;
//                SB_Intake.action(IntakeActions.SAFE_OUT);
//                SB_Intake.setSlidePosition(SB_Intake.positionSlidePitMin,1);
//        }
//        if (state == 2) {                 // if shoulder is in safe position, retract the rest of the way
//            if (SB_Intake.isShoulderDone()) {
//                state++;
//                SB_Intake.setSlidePosition(SB_Intake.positionSlideMin,1);
//            }
//        }
//        if (state == 3) {                 // wait until slide is retracted
//            if (SB_Intake.isSlideInTolerance()) {
//                state++;
//            }
//        }
//        if (state == 4) {                 // all done
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
