package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

class Shoot3 {

    private static int state = 0;
    private static int cycleCount = 0;
    private static boolean complete = false;
    private static long cancelTimer;
    private static final long timeLimit = 10000;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running
        if  (System.currentTimeMillis() >= cancelTimer) stop();

        if (state == 1) {                                       // cancel other state machines if needed
            if (Shoot1.isRunning()) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot1.softStop();
            }
            if (FullAuto.isRunning()) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                FullAuto.softStop();
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) state++;
        }
        if (state == 2) {                 // open gate, start spinner    // todo: there's no "if" here, so these tasks could move to state==1
            Pusher.stop();   // cancel any ongoing pusher movement
            cycleCount = 0;
            DSShooter.armShooter();
            state++;
        }
        if (state == 3) {                 // wait for gate up, spinner at rpm
            if (DSShooter.isGateOpen() && DSShooter.isPusherRetracted() && DSShooter.isSpinnerInTolerance()) {
                cycleCount++;
                Pusher.start();    // start the pusher state machine
                state++;
            }
        }
        if (state == 4) {                 // wait for pusher machine to complete
            if (Pusher.isComplete()) {
                if (cycleCount == DSShooter.pusherAutoCycles) {
                    complete = true;
                    DSShooter.disarmTimer = System.currentTimeMillis() + DSShooter.disarmTimeAfterFire;
                }
                else if (DSShooter.isSpinnerInTolerance()) {
                    cycleCount++;
                    Pusher.start();      //restart pusher
                }
            }
            else if (!Pusher.isRunning()) stop();   //cancel if problem
        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
        cancelTimer = System.currentTimeMillis() + timeLimit;
        DSShooter.disarmTimer = System.currentTimeMillis() + timeLimit + DSShooter.disarmTimeAfterFire;
    }

    public static void startForFullAuto() {
        complete = false;
        state = 2;   // skip the checks for stopping other state machines
        cancelTimer = System.currentTimeMillis() + timeLimit;
        DSShooter.disarmTimer = System.currentTimeMillis() + timeLimit + DSShooter.disarmTimeAfterFire;
    }

    public static void stop() {
        DSShooter.spinnerOff();
        DSShooter.retractPusher();
        // do we want to close the gate?  Ring might be in there...
        state = -1;
    }

    public static void softStop() {
        // don't spin down!
        DSShooter.retractPusher();
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
