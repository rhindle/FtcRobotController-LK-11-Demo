package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smStartFishing {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        Start Fishing
//
//        Spintake to safe horizontal position
//        Slide out to minimum fishing distance
//        Spintake to floor
//                (spintake powered off???)
//        ? Spinner running

        if (state == 1) {                 // put spintake in safe position, start slide
            SB_Intake.action(IntakeActions.SPINTAKE_SAFE);
            SB_Intake.setSlidePosition(SB_Intake.positionSlideStartIntake, 1);
            state++;
        }
        if (state == 2) {                 // wait for slide to finish (maybe only need to go part way?)
            if (SB_Intake.isSlideInTolerance()) {
                state++;
                SB_Intake.action(IntakeActions.SPINTAKE_ALMOSTFLOOR);
            }
        }
        if (state == 3) {                // assure movement is complete
            if (SB_Intake.isSpintakeDone()) {
                state++;
                // consider disabling it here?  Especially if add wheels
            }
        }
        if (state == 4) {               // all done
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
