package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSMisc;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;

class FullAuto {

    private static int state = 0;
    private static boolean complete = false;
    private static long cancelTimer;
    private static final long timeLimit = 20000;


    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running
        if  (System.currentTimeMillis() >= cancelTimer) stop();

        if (!DSShooter.parts.positionMgr.hasPosition()) stop();    // cancel running if no navigation

        if (state == 1) {
            if (Shoot1.isRunning()) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot1.softStop();
            }
            if (Shoot3.isRunning()) {
                DSShooter.cancelTimer = System.currentTimeMillis() + 1000;
                Shoot3.softStop();
            }
            if (System.currentTimeMillis() >= DSShooter.cancelTimer) state++;
        }

        if (state == 2) {                                    // navigate to launch position
            DSShooter.parts.autoDrive.setNavTarget(new NavigationTarget(DSMisc.autoLaunchPos, DSShooter.parts.dsMisc.toleranceHigh));
            DSShooter.armShooter();
            state++;
        }
        if (state == 3) {                                   // wait until reach position and start blasting
            if (DSShooter.parts.autoDrive.onTargetByAccuracy) {
                Shoot3.start();
                state++;
            }
        }
        if (state == 4) {
            if (Shoot3.isComplete()) {
                DSShooter.parts.autoDrive.setAutoDrive(false);
                DSShooter.disarmShooter();
                complete = true;
            }
        }
    }
    //----State Machine End-----

    public static void start() {
        complete = false;
        state = 1;
        cancelTimer = System.currentTimeMillis() + timeLimit;
        DSShooter.disarmTimer = System.currentTimeMillis() + timeLimit + 5000;
    }

    public static void stop() {
        DSShooter.parts.autoDrive.setAutoDrive(false);
        state = -1;
    }

    public boolean isRunning() {
        return (state>0);
    }

    public boolean isComplete() {
        return complete;
    }

    public static int getState() {
        return state;
    }
}
