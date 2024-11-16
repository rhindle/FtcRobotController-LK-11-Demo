package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber.IntakeActions;

public class makeSpace {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        if (state == 1) {                 // make sure shoulder is ready
            T24MultiGrabber.action(IntakeActions.GRAB_WIDEOPEN);
            T24MultiGrabber.action(IntakeActions.SHOULDER_DRAG);
            state++;
        }
        if (state == 2) {                 // sweep left
            if (T24MultiGrabber.isShoulderDone()) {
                state++;
                T24MultiGrabber.setRotatorServo(T24MultiGrabber.rotator45Left);
            }
        }
        if (state == 3) {                 // sweep right
            if (T24MultiGrabber.isRotatorDone()) {
                state++;
                T24MultiGrabber.setRotatorServo(T24MultiGrabber.rotator45Right);
            }
        }
        if (state == 4) {                 // sweep center
            if (T24MultiGrabber.isRotatorDone()) {
                state++;
                T24MultiGrabber.setRotatorServo(T24MultiGrabber.rotatorCenter);
            }
        }
        if (state == 5) {                 // wait until centered
            if (T24MultiGrabber.isRotatorDone()) {
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
