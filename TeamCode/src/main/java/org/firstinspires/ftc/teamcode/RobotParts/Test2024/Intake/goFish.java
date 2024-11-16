package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber.IntakeActions;

public class goFish {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        if (state == 1) {                 // put servos in safe positions
            T24MultiGrabber.action(IntakeActions.SAFE_IN);
            state++;
        }
        if (state == 2) {                 // wait for servos to complete, extend
            if (T24MultiGrabber.isShoulderDone() && T24MultiGrabber.isPinchDone()) {
                state++;
                T24MultiGrabber.setSlidePosition(T24MultiGrabber.positionSlideStartIntake,0.25);
            }
        }
        if (state == 3) {                // wait for slide to extend paste safe point
            if (T24MultiGrabber.isSlideInsidePit()) {
                state++;
                T24MultiGrabber.action(IntakeActions.GRAB_HOVER);
            }
        }
        if (state == 4) {                // wait for all important actions to complete
            if (T24MultiGrabber.isSlideInTolerance() && T24MultiGrabber.isShoulderDone()) {
                state++;
            }
        }
        if (state == 5) {               // all done
            complete = true;
        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
    }

    public static void stop() {
        // make it safe?
        T24MultiGrabber.action(IntakeActions.SAFE_OUT);
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
