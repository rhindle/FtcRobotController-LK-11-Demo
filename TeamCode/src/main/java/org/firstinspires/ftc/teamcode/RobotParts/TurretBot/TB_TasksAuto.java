package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.ArcPath;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;

public class TB_TasksAuto {

    public static Parts parts;

    public static StateMachine smAutoTestGotoCenter;
    public static StateMachine smAutoNearGotoShoot;
    public static StateMachine smAutoNearGotoShoot45;
    public static StateMachine smAutoNearSpike1;
    public static StateMachine smAutoNearSpike2;
    public static StateMachine smAutoNearSpike3;
    public static StateMachine smAutoNearGotoGate;
    public static StateMachine smAutoNearOperateGate;
    public static StateMachine smAutoNearOrchestrator;
    public static StateMachine smAutoTestTransitions;

    public static double autoSpeed = 1.0;
    static PIDCoefficients HighPID = new PIDCoefficients(0.12,0.0012,0.006);  // is 0.03 in PartsTB

    public static void setup(Parts p) {
        parts = p;
    }

    public static void initialize(){
        buildStateMachines();
    }

    static void buildStateMachines() {
        StateMachine task;

        // reminder: X is toward audience, Y is away from red team

        // This robot is fast so it might pass right through a transition point without registering
        // (and then reverse to go back to it)
        // The trick will be to find tolerance parameters and map the navigation points
        // in such a way as to avoid this problem.
        // Or don't use noSlow but make large tolerance values to minimize slowing.
        // Or: (new) use a higher PID in the NavTarget if you don't care about overshoot

        /*
        
        Auto near:
        
        1. Move to shoot point, shoot  [3]
        2. Get 2nd spike
        3. Shoot point, shoot  [6]
        4. Get balls at gate
        5. Shoot point, shoot  [9]
        6. Repeat 4,5 as time allows  [2 more -> 15]
             (w/o lots of optimization, may be stuck at 2 iterations only -> 12)
        7. Get 1st spike  [18]
        8. Shoot point, shoot
            
        Auto far:
        
        1. Move to shoot point, shoot  [3]
        2. Get human player
            (may take some fancy driving)
        3. Shoot point, shoot  [6]
        4. Get 3rd spike
        5. Shoot point, shoot  [9]
        6. Human player attempts
          a. If not full, move over and try again (need sensors)
          b. If no balls, move over and try yet again
          c. If still no balls, start 6 over
          d. Shoot point, shoot  [12 15]
        
        */

        // suggestion:  Make different actions different tasks with an orchestrator task that
        // runs them all and can adjust for time remaining, etc.
        // Do not give the orchestrator task the same group name or it will be stopped by the subtasks.

        Position p_center                   = TB_Misc.redOrBlue(0,0,180);
//        Position p_nearStart                = redOrBlue(-40, -56, 180);  // This isn't defined here; see partsTB
        Position p_start_move1              = TB_Misc.redOrBlue(-40,-48,180);   // move away from the wall first
        Position p_nearShoot                = TB_Misc.redOrBlue(-12, -23, -135);  // -12, -17

        Position p_nearShoot45              = TB_Misc.redOrBlue(-12, -23, -45);

        Position p_spike1_pre               = TB_Misc.redOrBlue(-13, -27, -90); //-15
        Position p_spike1_fin               = TB_Misc.redOrBlue(-13, -53, -90);
        Position p_spike1_shoot             = TB_Misc.redOrBlue(-45, -20, -66);

        Position p_spike2_pre               = TB_Misc.redOrBlue(12, -29, -90);  //13
        Position p_spike2_fin               = TB_Misc.redOrBlue(12, -60, -90);
        Position p_spike2_leave1            = TB_Misc.redOrBlue(10.36, -50, -68.4);  //-43.31
        //Position p_spike2_leave2            = TB_Misc.redOrBlue(0.46, -26.57, -46.76);

        Position p_spike3_pre               = p_spike2_pre.withX(35.5); //36.5
        Position p_spike3_fin               = p_spike2_fin.withX(35.5);
        Position p_spike3_leave             = p_spike3_pre.clone();

        Position p_gate_pre1                = TB_Misc.redOrBlue(5, -29, -90);
        Position p_gate_pre2                = TB_Misc.redOrBlue(7, -46, -90);
        //Position p_gate_tap                 = TB_Misc.redOrBlue(7, -54, -90);
        Position p_gate_gather              = TB_Misc.redOrBlue(14, -59.75, -120);  // -58.75
        Position p_gate_leave1            = TB_Misc.redOrBlue(10.36, -50, -68.4);  //-43.31
        //Position p_gate_leave2            = TB_Misc.redOrBlue(0.46, -26.57, -46.76);

        autoSpeed = 1.0; //0.75;  //0.5;  // todo: remember to change this back to 1

        smAutoTestGotoCenter = new StateMachine("ATCenter");
        task = smAutoTestGotoCenter;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetAccurate(p_center)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.stop());

        smAutoNearGotoShoot = new StateMachine("ANShoot");
        task = smAutoNearGotoShoot;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_nearShoot)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.stop() );  // so it doesn't try to "hold" back to position

        smAutoNearGotoShoot45 = new StateMachine("ANShoot45");
        task = smAutoNearGotoShoot45;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.stop() );  // so it doesn't try to "hold" back to position

        smAutoNearSpike1 = new StateMachine("ANSpike1");
        task = smAutoNearSpike1;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike1_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike1_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_spike1_shoot)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.stop());
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );

        smAutoNearSpike2 = new StateMachine("ANSpike2");
        task = smAutoNearSpike2;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike2_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike2_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateReturnArc(p_spike2_leave1, p_nearShoot45, autoSpeed, 0.171, 1000)) );
        task.addDelayOf(750); // try to make sure the last ball is intaken
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoNearSpike3 = new StateMachine("ANSpike3");
        task = smAutoNearSpike3;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike3_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike3_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike3_leave)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoNearGotoGate = new StateMachine("ANGotoGate");
        task = smAutoNearGotoGate;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(p_gate_pre1, toleranceTransition, 0.5,1000, true)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(p_gate_pre2, toleranceLow, autoSpeed,2000, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoNearOperateGate = new StateMachine("ANUseGate");
        task = smAutoNearOperateGate;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_gather)) );
        task.addWaitFor(() -> TB_Intake.definitely3, 3000);
        task.addRunOnce(() -> parts.autoDrive.stop());  // in case it's still trying to navigate
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateReturnArc(p_gate_leave1, p_nearShoot45, autoSpeed, 0.171, 1000)) );
        task.addDelayOf(750); // try to make sure the last ball is intaken // 300
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoNearOrchestrator = new StateMachine("ANOrch");
        task = smAutoNearOrchestrator;
        task.setGroups("orchestrator");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            TB_Turret.armTurret(true);
            TB_Turret.armSpinner(true);
        });

        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_start_move1)) );
        task.addRunOnce(smAutoNearGotoShoot::restart);
        task.addWaitFor(smAutoNearGotoShoot::isDone);
        task.addWaitFor(TB_Turret::isSpinnerInToleranceV2, 2000);  //added for the fist shot only?
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoNearSpike2::restart);
        task.addWaitFor(smAutoNearSpike2::isDone);
        task.addRunOnce(smAutoNearGotoShoot45::restart);
        task.addWaitFor(smAutoNearGotoShoot45::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoNearGotoGate::restart);
        task.addWaitFor(smAutoNearGotoGate::isDone);
        task.addRunOnce(smAutoNearOperateGate::restart);
        task.addWaitFor(smAutoNearOperateGate::isDone);
        task.addRunOnce(smAutoNearGotoShoot45::restart);
        task.addWaitFor(smAutoNearGotoShoot45::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoNearGotoGate::restart);
        task.addWaitFor(smAutoNearGotoGate::isDone);
        task.addRunOnce(smAutoNearOperateGate::restart);
        task.addWaitFor(smAutoNearOperateGate::isDone);
        task.addRunOnce(smAutoNearGotoShoot45::restart);
        task.addWaitFor(smAutoNearGotoShoot45::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        // suggest checking if enough time is left and skip if not
//        task.addRunOnce(smAutoNearGotoGate::restart);
//        task.addWaitFor(smAutoNearGotoGate::isDone);
//        task.addRunOnce(smAutoNearOperateGate::restart);
//        task.addWaitFor(smAutoNearOperateGate::isDone);
//        task.addRunOnce(smAutoNearGotoShoot45::restart);
//        task.addWaitFor(smAutoNearGotoShoot45::isDone);
//        task.addRunOnce(TB_Tasks.smLaunch::restart);
//        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoNearSpike1::restart);
        task.addWaitFor(smAutoNearSpike1::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);


        /* Below is a test machine for making circles with transition tolerances. */

        Position posStart = new Position(-24,0,-90);
        Position posOpp   = new Position(24,0,90);
        Position posCenter = new Position(0,0,0);
        int timeLimit = 5000;
        double speed = 0.75;

        smAutoTestTransitions = new StateMachine("ATTransitions");
        task = smAutoTestTransitions;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);

        // Make a circle driving forward
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(posStart, toleranceMedium, speed, timeLimit, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(2000);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.FORWARD)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(1000);

        // Make a circle driving backward
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(posStart.withR(90), toleranceMedium, speed, timeLimit, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(500);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.BACKWARD)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(1000);

        // Make a circle aiming inward
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(posStart.withR(0), toleranceMedium, speed, timeLimit, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(500);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.INWARD)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(1000);

        // Make a circle aiming right (-90)
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(posStart.withR(-90), toleranceMedium, speed, timeLimit, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(500);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.RIGHT)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(1000);

        // Make a circle aiming at the target (DSMisc.aimPosition)
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(posStart.withR(0), toleranceMedium, speed, timeLimit, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(500);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.TARGET)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(1000);

        // Make a circle while smoothly changing heading
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTarget(posStart.withR(90), toleranceMedium, speed, timeLimit, false)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(500);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(generateNavCircle(posStart, posOpp, speed, timeLimit, circleVar.SMOOTHCHANGE)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addDelayOf(1000);

    }

    static NavigationTarget toTargetTransition (Position pos) {
        return toTargetTransition (pos, autoSpeed, 2000, true);   //giving up on noslow for now 5000
    }
    static NavigationTarget toTargetTransition (Position pos, double speed, long time, boolean noSlow) {
        return new NavigationTarget(pos, toleranceHigh, speed, time, noSlow);  // 0.5
    }

    static NavigationTarget toTargetAccurate (Position pos) {
        return toTargetAccurate(pos, autoSpeed, 5000);
    }
    static NavigationTarget toTargetAccurate (Position pos, double speed, long time) {
        return new NavigationTarget(pos, toleranceHigh, speed, time, false);  // 0.5
    }

    static NavigationTarget toTargetMedium (Position pos) {
        return toTargetMedium(pos, autoSpeed, 5000);
    }
    static NavigationTarget toTargetMedium (Position pos, double speed, long time) {
        return new NavigationTarget(pos, toleranceMedium, speed, time, false);
    }

    static NavigationTarget toTargetLow (Position pos) {
        return toTargetLow(pos, autoSpeed, 5000);
    }
    static NavigationTarget toTargetLow (Position pos, double speed, long time) {
        return new NavigationTarget(pos, toleranceLow, speed, time, false);
    }

    static NavigationTarget toTargetLowHighPID (Position pos) {
        return toTargetLowHighPID(pos, autoSpeed, 5000);
    }
    static NavigationTarget toTargetLowHighPID (Position pos, double speed, long time) {
        return new NavigationTarget(pos, toleranceLow, speed, time, false, HighPID);
    }

    static NavigationTarget toTarget (Position pos, PositionTolerance tolerance, double maxSpeed, long timeLimit, boolean noSlow) {
        return new NavigationTarget(pos, tolerance, maxSpeed, timeLimit, noSlow);
    }
    static NavigationTarget toTarget (Position pos, PositionTolerance tolerance, double maxSpeed, long timeLimit,
                                      boolean noSlow, PIDCoefficients PID) {
        return new NavigationTarget(pos, tolerance, maxSpeed, timeLimit, noSlow, PID);
    }
    static NavigationTarget toTarget (Position pos, PositionTolerance tolerance, double maxSpeed, long timeLimit,
                                      boolean noSlow, PIDCoefficients PIDmove, PIDCoefficients PIDrotate) {
        return new NavigationTarget(pos, tolerance, maxSpeed, timeLimit, noSlow, PIDmove, PIDrotate);
    }

    // These could also be stored in TB_Misc (and in fact they are!)
    static PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
    static PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
    static PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
    static PositionTolerance toleranceLow = new PositionTolerance (4.0, 5.0, 0);
//    static PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
    static PositionTolerance toleranceTransition = new PositionTolerance(6.0,90.0,0);


    static public NavigationTarget[] generateReturnArc (Position posStart, Position posEnd, double speed, double depth, int timeLimit) {
        // return from Spike 2 and gate 2
        // calculate an arc path with an appropriate depth (sagitta), and direction based on alliance.
        Position[] arc1 = ArcPath.calculateArcPathWithDepth(posStart, posEnd, depth, TB_Misc.isAllianceBlue() ? 1 : -1, 5);
        // adjust the direction (position.R) through the path to match the start and end
        arc1 = ArcPath.adjustArcPathHeadingStartEnd(arc1, posStart.R, posEnd.R);
        return ArcPath.buildNavTargetArray(
                arc1,
                toleranceTransition,  // start tolerance
                toleranceTransition,  // mid tolerance
                toleranceTransition,  // end tolerance
                speed,
                timeLimit,            // per step
                true);                // no slow at end
    }


    // added for testing transition paths 20260603
    // leaving because it provides examples of how to use ArcPath
    static public NavigationTarget[] generateNavCircle (Position posStart, Position posMid, double speed, int timeLimit, circleVar var){
        Position[] arc1 = ArcPath.calculateArcPathWithDepth(posStart, posMid, 1, 1, 11);
        Position[] arc2 = ArcPath.calculateArcPathWithDepth(posMid, posStart, 1, 1, 11);
        Position[] circle = ArcPath.combinePaths(arc1, arc2, true);
        switch (var) {
            case FORWARD:
                break;
            case BACKWARD:
                circle = ArcPath.adjustArcPathHeadingRelative(circle, 180);
                break;
            case INWARD:
                circle = ArcPath.adjustArcPathHeadingRelative(circle, 90);
                break;
            case RIGHT:
                circle = ArcPath.adjustArcPathHeadingConstant(circle, -90);
                break;
            case TARGET:
                circle = ArcPath.adjustArcPathHeadingTarget(circle, new Position (72, 0, 0));  //new Position (0, -4, 0)
                break;
            case SMOOTHCHANGE:
                circle = ArcPath.combinePaths(
                        ArcPath.adjustArcPathHeadingStartEnd(arc1, 90, -45),
                        ArcPath.adjustArcPathHeadingStartEnd(arc2, -45, 90),
                        true);
                break;
            default: break;
        }
        return ArcPath.buildNavTargetArray(
                circle,
                toleranceTransition,
                toleranceMedium,
                speed,
                timeLimit,
                false);
    }

    public enum circleVar {
        FORWARD,
        BACKWARD,
        INWARD,
        RIGHT,
        TARGET,
        SMOOTHCHANGE;
    }

}
