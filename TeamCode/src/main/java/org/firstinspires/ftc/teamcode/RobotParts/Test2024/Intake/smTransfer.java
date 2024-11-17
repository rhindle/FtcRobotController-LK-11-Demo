package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber.IntakeActions;

public class smTransfer {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        if (state == 1) {    // start everything in motion
            state++;
            T24MultiGrabber.setShoulderServo(T24MultiGrabber.shoulderFullBack);
            T24MultiGrabber.setRotatorServo(T24MultiGrabber.rotatorTransfer);
            T24MultiGrabber.setWristServo(T24MultiGrabber.wristTransfer);
            T24MultiGrabber.setPinchServo(T24MultiGrabber.pinchLoose);
            T24MultiGrabber.setSlidePosition(T24MultiGrabber.positionSlideMin, 1);

            T24MultiGrabber.setLiftPinchServo(T24MultiGrabber.liftPinchSafe);
            //if (!T24MultiGrabber.isLiftShoulderAtTransfer()) T24MultiGrabber.setLiftShoulderServo(T24MultiGrabber.liftShoulderSafe);
            T24MultiGrabber.setLiftShoulderServo(T24MultiGrabber.liftShoulderSafe);
            T24MultiGrabber.setLiftPosition(T24MultiGrabber.positionLiftTransfer, 1);
        }
        if (state == 2) {    // get the pincher safely past the lift
            if (T24MultiGrabber.isLiftShoulderDone()) {
                state++;
                T24MultiGrabber.setLiftShoulderServo(T24MultiGrabber.liftShoulderTransfer);
                T24MultiGrabber.setLiftPinchServo(T24MultiGrabber.liftPinchWide);
            }
        }
        if (state == 3) {    // make sure the intake servos are done
            if (T24MultiGrabber.isShoulderDone() &&
                    T24MultiGrabber.isRotatorDone() &&
                    T24MultiGrabber.isWristDone()) {
                state++;
                T24MultiGrabber.setPinchServo(T24MultiGrabber.pinchClosed);
            }
        }
        if (state == 4) {    // make sure the slides are done
            if (T24MultiGrabber.isSlideInTolerance() && T24MultiGrabber.isLiftInTolerance()) {
                state++;
            }
        }
        if (state == 5) {    // make sure the extake servos are done and pinch
            if (T24MultiGrabber.isLiftShoulderDone() && T24MultiGrabber.isLiftPinchDone()) {
                state++;
                T24MultiGrabber.setLiftPinchServo(T24MultiGrabber.liftPinchTransfer);
            }
        }
        if (state == 6) {    // wait for lift pinch to complete, then release intake pinch
            if (T24MultiGrabber.isLiftPinchDone()) {
                state++;
                T24MultiGrabber.setPinchServo(T24MultiGrabber.pinchSlightOpen);
            }
        }
        if (state == 7) {    // wait for intake pinch to open, then prep for drop
            if (T24MultiGrabber.isPinchDone()) {
                state++;
                T24MultiGrabber.setLiftShoulderServo(T24MultiGrabber.liftShoulderBack);
            }
        }
        if (state == 8) {    // wait for lift shoulder to complete
            if (T24MultiGrabber.isLiftShoulderDone()) {
                state++;
            }
        }
        if (state == 9) {
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
