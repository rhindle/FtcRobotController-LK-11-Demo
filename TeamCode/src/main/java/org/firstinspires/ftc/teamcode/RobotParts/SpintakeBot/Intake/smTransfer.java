package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smTransfer {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        if (state == 1) {    // start everything in motion
//            state++;
//            SB_Intake.setShoulderServo(SB_Intake.shoulderFullBack);
//            SB_Intake.setRotatorServo(SB_Intake.rotatorTransfer);
//            SB_Intake.setWristServo(SB_Intake.wristTransfer);
//            SB_Intake.setPinchServo(SB_Intake.pinchLoose);
//            SB_Intake.setSlidePosition(SB_Intake.positionSlideMin, 1);
//
//            SB_Intake.setLiftPinchServo(SB_Intake.liftPinchSafe);
//            //if (!SB_Intake.isLiftShoulderAtTransfer()) SB_Intake.setLiftShoulderServo(SB_Intake.liftShoulderSafe);
//            SB_Intake.setLiftShoulderServo(SB_Intake.liftShoulderSafe);
//            SB_Intake.setLiftPosition(SB_Intake.positionLiftTransfer, 1);
//        }
//        if (state == 2) {    // get the pincher safely past the lift
//            if (SB_Intake.isLiftShoulderDone()) {
//                state++;
//                SB_Intake.setLiftShoulderServo(SB_Intake.liftShoulderTransfer);
//                SB_Intake.setLiftPinchServo(SB_Intake.liftPinchWide);
//            }
//        }
//        if (state == 3) {    // make sure the intake servos are done
//            if (SB_Intake.isShoulderDone() &&
//                    SB_Intake.isRotatorDone() &&
//                    SB_Intake.isWristDone()) {
//                state++;
//                SB_Intake.setPinchServo(SB_Intake.pinchClosed);
//            }
//        }
//        if (state == 4) {    // make sure the slides are done
//            if (SB_Intake.isSlideInTolerance() && SB_Intake.isLiftInTolerance()) {
//                state++;
//            }
//        }
//        if (state == 5) {    // make sure the extake servos are done and pinch
//            if (SB_Intake.isLiftShoulderDone() && SB_Intake.isLiftPinchDone()) {
//                state++;
//                SB_Intake.setLiftPinchServo(SB_Intake.liftPinchTransfer);
//            }
//        }
//        if (state == 6) {    // wait for lift pinch to complete, then release intake pinch
//            if (SB_Intake.isLiftPinchDone()) {
//                state++;
//                SB_Intake.setPinchServo(SB_Intake.pinchSlightOpen);
//            }
//        }
//        if (state == 7) {    // wait for intake pinch to open, then prep for drop
//            if (SB_Intake.isPinchDone()) {
//                state++;
//                SB_Intake.setLiftShoulderServo(SB_Intake.liftShoulderBack);
//            }
//        }
//        if (state == 8) {    // wait for lift shoulder to complete
//            if (SB_Intake.isLiftShoulderDone()) {
//                state++;
//            }
//        }
//        if (state == 9) {
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
