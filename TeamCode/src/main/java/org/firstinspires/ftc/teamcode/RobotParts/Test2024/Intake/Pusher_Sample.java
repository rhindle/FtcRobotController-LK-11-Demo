package org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake;

public class Pusher_Sample {

    private static int state = 0;
    private static boolean complete = false;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

        if (state == 1) {                 // extend pusher
//            DSShooter.extendPusher();
            state++;
        }
        if (state == 2) {                 // wait for complete, retract
//            if (DSShooter.isPusherExtended()) {
//                DSShooter.retractPusher();
//                state++;
//            }
        }
        if (state == 3) {                // wait for complete
//            if (DSShooter.isPusherRetracted()) {
//                complete = true;
//                DSShooter.disarmTimer = System.currentTimeMillis() + DSShooter.disarmTimeAfterFire;
//            }
        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
    }

    public static void stop() {
//        DSShooter.retractPusher();
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
