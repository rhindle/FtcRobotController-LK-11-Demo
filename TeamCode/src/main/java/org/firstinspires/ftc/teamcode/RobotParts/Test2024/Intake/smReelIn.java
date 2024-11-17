package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber.IntakeActions;

public class smReelIn {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        if (state == 1) {                 // make sure shoulder is ready
            T24MultiGrabber.action(IntakeActions.SHOULDER_CAPTURE);
            state++;
        }
        if (state == 2) {                 // activate the grabber
            if (T24MultiGrabber.isShoulderDone()) {
                state++;
                T24MultiGrabber.action(IntakeActions.GRAB_CLOSE);
            }
        }
        if (state == 3) {                 // when grabber is done, put in safe position and retract to safe point
            if (T24MultiGrabber.isPinchDone()) {
                state++;
                T24MultiGrabber.action(IntakeActions.SAFE_OUT);
                T24MultiGrabber.setSlidePosition(T24MultiGrabber.positionSlidePitMin,1);
            }
        }
        if (state == 4) {                 // if shoulder is in safe position, retract he rest of the way
            if (T24MultiGrabber.isShoulderDone()) {
                state++;
                T24MultiGrabber.setSlidePosition(T24MultiGrabber.positionSlideMin,1);
            }
        }
        if (state == 5) {                 // wait until slide is retracted
            if (T24MultiGrabber.isSlideInTolerance()) {

                state++;
            }
        }
        if (state == 6) {                 // all done
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
