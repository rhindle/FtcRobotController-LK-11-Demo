package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber.IntakeActions;

public class smGrabAndInspect {

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
        if (state == 3) {                 // when grabber is done, put in safe position to inspect
            if (T24MultiGrabber.isPinchDone()) {
                state++;
//                T24MultiGrabber.action(IntakeActions.SAFE_OUT);
                T24MultiGrabber.setShoulderServo(T24MultiGrabber.shoulderSafeOut);
            }
        }
        if (state == 4) {                 // if shoulder is in safe position, we are done
            if (T24MultiGrabber.isShoulderDone()) {
                state++;
            }
        }
        if (state == 5) {                 // all done
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
