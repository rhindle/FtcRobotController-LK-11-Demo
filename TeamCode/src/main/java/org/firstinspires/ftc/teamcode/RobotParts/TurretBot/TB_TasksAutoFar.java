package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;

public class TB_TasksAutoFar {

    public static Parts parts;

    public static StateMachine smAutoFarGotoShoot180;
    public static StateMachine smAutoFarGotoShoot90;
    public static StateMachine smAutoFarGotoSpike3;
    public static StateMachine smAutoFarOrchestrator;
    public static StateMachine smAutoFarHumanArea;

    public static double autoSpeed = 1.0;

    //        autoDrive.PIDmovement = new PIDCoefficients(0.03,0.0012,0.006); //.12 0 .035  0.06
    static PIDCoefficients HighPID = new PIDCoefficients(0.12,0.0012,0.006);  // is 0.03 is PartsTB


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

//        public static final Position fieldStartPositionBlueFar  = new Position(63, -16, 180);  // todo: update this
        Position p_center                   = TB_Misc.redOrBlue(0,0,180);
        Position p_farShoot180              = TB_Misc.redOrBlue(60, -12, 180);
        Position p_farShoot90               = TB_Misc.redOrBlue(60, -12, -90);

        Position p_spike3_pre               = TB_Misc.redOrBlue(35.5, -29, -90);
        Position p_spike3_fin               = TB_Misc.redOrBlue(35.5, -60, -90);
        Position p_spike3_leave             = p_spike3_pre.clone();

        Position p_human_wall               = TB_Misc.redOrBlue(60, -60, -90);
        Position p_human_retry              = TB_Misc.redOrBlue(60, -45, -90);
        Position p_human_wall2              = TB_Misc.redOrBlue(48, -60, -90);
        Position p_human_retry2             = TB_Misc.redOrBlue(48, -45, -90);
        Position p_human_wall3              = TB_Misc.redOrBlue(36, -60, -90);
        Position p_human_slide_start        = TB_Misc.redOrBlue(56, -59.75, -120);
        Position p_human_slide_end          = TB_Misc.redOrBlue(32, -59.75, -120);


//        Position p_gate_pre1                = TB_Misc.redOrBlue(5, -29, -90);
//        Position p_gate_pre2                = TB_Misc.redOrBlue(7, -46, -90);
//        Position p_gate_tap                 = TB_Misc.redOrBlue(7, -54, -90);
//        Position p_gate_gather              = TB_Misc.redOrBlue(14, -59.75, -120);  // -58.75
//        Position p_gate_leave1            = TB_Misc.redOrBlue(10.36, -50, -68.4);  //-43.31
//        Position p_gate_leave2            = TB_Misc.redOrBlue(0.46, -26.57, -46.76);

        Position p_park                     = TB_Misc.redOrBlue(55, -20, -90);

        autoSpeed = 0.75;  //0.5;  // todo: remember to change this back to 1

//        smAutoTestGotoCenter = new StateMachine("ATCenter");
//        task = smAutoTestGotoCenter;
//        task.setGroups("autotest");  // will be killed by
//        task.setAutoRestart(false);
//        task.addRunOnce(TB_Intake::intakeOff);
//        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetAccurate(p_center)) );
//        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
//        task.addRunOnce(() -> parts.autoDrive.stop());

        smAutoFarGotoShoot180 = new StateMachine("ATFarShoot180");
        task = smAutoFarGotoShoot180;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        // Note: First shoot will require time for turret to get ready, so tighter tolerance OK?
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_farShoot180)) );
//        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLowHighPID(p_farShoot180)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
//        task.addRunOnce(() -> parts.autoDrive.stop() );  // so it doesn't try to "hold" back to position

        smAutoFarGotoShoot90 = new StateMachine("ATFarShoot90");
        task = smAutoFarGotoShoot90;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
//        task.addRunOnce(TB_Intake::intakeOff);
        // reminder: comment out new drive position if testing arc generator in spike2/gate
        //           or refactor and don't call this machine at all
        // Note: Overshooting here will be dangerous, so may require tighter tolerance
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_farShoot90)) );
        task.addDelayOf(500); // try to make sure the last ball is intaken
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
//        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLowHighPID(p_farShoot90)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
//        task.addRunOnce(() -> parts.autoDrive.stop() );  // so it doesn't try to "hold" back to position

        smAutoFarGotoSpike3 = new StateMachine("ATSpike3");
        task = smAutoFarGotoSpike3;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike3_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetLow(p_spike3_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_spike3_leave)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoFarHumanArea = new StateMachine("ATHumanArea");
        task = smAutoFarHumanArea;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
          // end the task when it has three balls or at least one and _ seconds have passed (todo: tune the time)
        task.setEndCriteria( () -> TB_Intake.probably3 || (TB_Intake.atLeast1 && (smAutoFarHumanArea.getRuntime() > 7500)) );
        task.setEndCriteriaRunnable( () -> parts.autoDrive.stop());  // stop navigating
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_wall)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_retry)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_wall)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_retry2)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_wall2)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_retry2)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_wall3)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_retry2)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_slide_start)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_human_slide_end)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);


        smAutoFarOrchestrator = new StateMachine("FarOrchestrator");
        task = smAutoFarOrchestrator;
        task.setGroups("orchestrator");  // will be killed by
        task.setAutoRestart(false);
        task.setEndCriteria( () -> (smAutoFarOrchestrator.getRuntime() > 38000) && !TB_Tasks.smLaunch.isRunning() );   // change to 28? seconds
        task.setEndCriteriaRunnable( () -> {
            StateMachine.stopGroups("autotest");
            parts.autoDrive.stop();  // stop navigating
            parts.autoDrive.addNavTargets(toTargetTransition(p_park));
        });

        task.addRunOnce(() -> {
            TB_Turret.armTurret(true);
            TB_Turret.armSpinner(true);
        });

//        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_start_move1)) );
        task.addRunOnce(smAutoFarGotoShoot180::restart);
        task.addWaitFor(smAutoFarGotoShoot180::isDone);
        task.addWaitFor(TB_Turret::isSpinnerInToleranceV2, 2000);  //added for the fist shot only?
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoFarGotoSpike3::restart);
        task.addWaitFor(smAutoFarGotoSpike3::isDone);
        task.addRunOnce(smAutoFarGotoShoot90::restart);
        task.addWaitFor(smAutoFarGotoShoot90::isDone);
        task.addWaitFor(TB_Turret::isSpinnerInToleranceV2, 2000);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        //add human player area stuff here
        task.addRunOnce(smAutoFarHumanArea::restart);
        task.addWaitFor(smAutoFarHumanArea::isDone);
        task.addRunOnce(smAutoFarGotoShoot90::restart);
        task.addWaitFor(smAutoFarGotoShoot90::isDone);
        task.addWaitFor(TB_Turret::isSpinnerInToleranceV2, 2000);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoFarHumanArea::restart);
        task.addWaitFor(smAutoFarHumanArea::isDone);
        task.addRunOnce(smAutoFarGotoShoot90::restart);
        task.addWaitFor(smAutoFarGotoShoot90::isDone);
        task.addWaitFor(TB_Turret::isSpinnerInToleranceV2, 2000);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoFarHumanArea::restart);
        task.addWaitFor(smAutoFarHumanArea::isDone);
        task.addRunOnce(smAutoFarGotoShoot90::restart);
        task.addWaitFor(smAutoFarGotoShoot90::isDone);
        task.addWaitFor(TB_Turret::isSpinnerInToleranceV2, 2000);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        // get off the line (park)
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_park)) );

    }

    static NavigationTarget toTargetTransition (Position pos) {
        return toTargetTransition (pos, autoSpeed, 1000, true);   //giving up on noslow for now 5000
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


}
