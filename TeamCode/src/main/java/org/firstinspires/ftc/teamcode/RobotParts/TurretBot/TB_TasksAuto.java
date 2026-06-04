package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.ArcPath;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;

public class TB_TasksAuto {

    public static Parts parts;

    public static StateMachine smAutoTest1;
    public static StateMachine smAutoTestGotoCenter;
    public static StateMachine smAutoTestGotoNearShoot;
    public static StateMachine smAutoTestGotoSpike1;
    public static StateMachine smAutoTestGotoSpike2;
    public static StateMachine smAutoTestGotoSpike3;
    public static StateMachine smAutoTestDriveToGate;
    public static StateMachine smAutoTestOperateGate;
    public static StateMachine smTieItAllTogether;
    public static StateMachine smAutoTestTransitions;

    public static double autoSpeed = 1.0;

    public static void setup(Parts p) {
        parts = p;
    }

    public static void initialize(){
        buildStateMachines();
    }

    static void buildStateMachines() {
        StateMachine task;

        // reminder: X is toward audience, Y is away from red team
        Position l_pos1          = new Position(-12, 8,150);
        Position l_pos2          = new Position(-24, 16,180);
        Position l_pos3          = new Position(-36, 8,-135);
        Position l_pos4          = new Position(-48, 0,-90); //here
        Position l_pos5          = new Position(-48, 16,180);
        Position l_pos6          = new Position(-36, 16, 180);
        Position l_pos7          = new Position(-24, 8, 150);
        Position l_pos8          = new Position(-12,0,180);
        Position l_pos9          = new Position(0,0,180);

        /******************************************************************************************/
        // This robot is fast so it might pass right through a transition point without registering
        // (and then reverse to go back to it)
        // The trick will be to find tolerance parameters and map the navigation points
        // in such a way as to avoid this problem.
        // Or don't use noSlow but make large tolerance values to minimize slowing.

        smAutoTest1 = new StateMachine("AutoTest1");
        task = smAutoTest1;
        task.setGroups("auto-master");  // will be killed by
        //task.setStopGroups("launcher", "green");    // groups to kill
        //task.setMemberGroups("all", "green");  // will be killed by
        task.setAutoRestart(false);
        //task.setStopRunnable( () -> {} );
        //task.setTimeLimit(5000);
        //task.setTimeoutRunnable( () -> {} );
        //task.setEndCriteria( () -> false );
        //task.setEndCriteriaRunnable( () -> {} );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos1)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos2)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos3)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetAccurate(l_pos4)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart());
        task.addDelayOf(2000);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos5)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos6)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos7)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(l_pos8)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetAccurate(l_pos9)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart());
        task.addWaitFor(2000);
        task.addRunOnce(() -> parts.autoDrive.stop());
        
        
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

        // These are positions copied from robot 1 blue side. Could do red instead.

//        Position p_targetGoal                = redOrBlue(-70.5, -60.5, 180);   // Y: -70.5; BlueGoal Position.
        Position p_fieldStart                = TB_Misc.redOrBlue(-39.0,-55,-180); // TODO: Confirm/Tune this position.
//        Position p_obeliskView               = redOrBlue(-39.0, -31, 160);  // GoalBlue: ObeliskView Position
        Position p_launchPosZero             = TB_Misc.redOrBlue(-28.0,-16,-135);    // GoalBlue Launching Position.
        Position p_launchPosOne              = TB_Misc.redOrBlue(-28.0,-16,-135);    // GoalBlue Launching Position.
        Position p_launchPosFinal            = TB_Misc.redOrBlue(-28.0,-16,-135);    // GoalBlue Launching Position for pinkServo. Z:???.

        Position p_pre_intakeArtifactRow2    = TB_Misc.redOrBlue(14, -22, 90);   // X:12; Blue: Ready to collect on Row2
        Position p_intakeArtifactRow2        = TB_Misc.redOrBlue(14, -60, 90);   // X:12; Blue: Intake Artifacts in Row2

        Position p_pre_intakeArtifactRow1    = TB_Misc.redOrBlue(-12, -22, 90);  // Blue: Ready to collect on Row1
        Position p_intakeArtifactRow1        = TB_Misc.redOrBlue(-12, -53, 90);  // Blue: Intake Artifacts in Row1

        //leave row 3 for far team?
//        Position p_pre_intakeArtifactRow3    = redOrBlue(35.5, -22, 90);   // Blue: Ready to collect in Row3
//        Position p_intakeArtifactRow3        = redOrBlue(35.5, -60, 90);   // Blue: Intake Artifacts in Row3

        Position p_pre_leverOpen             = TB_Misc.redOrBlue(0, -45, -180);    // Blue: Pre-Open Lever Position
        Position p_leverOpen                 = TB_Misc.redOrBlue(0, -55, -180);    // Blue: Open Lever Position
        Position p_parkAfterAuto             = TB_Misc.redOrBlue(-12,-28,-180);

        // suggestion:  Make different actions different tasks with an orchestrator task that
        // runs them all and can adjust for time remaining, etc.
        // Do not give the orchestrator task the same group name or it will be stopped by the subtasks.



        Position p_center                   = TB_Misc.redOrBlue(0,0,180);
//        Position p_nearStart                = redOrBlue(-41, -56, 180);
//        Position p_nearStart                = redOrBlue(-40, -56, 180);  // This isn't defined here; see partsTB
//        Position p_nearShoot                = redOrBlue(-27, -22, -134);
        Position p_start_move1              = TB_Misc.redOrBlue(-40,-48,180);   // move away from the wall first
        Position p_nearShoot                = TB_Misc.redOrBlue(-12, -17, -135);

        Position p_spike1_pre               = TB_Misc.redOrBlue(-13, -27, -90); //-15
        Position p_spike1_fin               = TB_Misc.redOrBlue(-13, -53, -90);

        Position p_spike2_pre               = TB_Misc.redOrBlue(12, -29, -90);  //13
        Position p_spike2_fin               = TB_Misc.redOrBlue(12, -60, -90);
        Position p_spike2_leave             = p_spike2_pre.clone();

        Position p_spike3_pre               = p_spike2_pre.withX(35.5); //36.5
        Position p_spike3_fin               = p_spike2_fin.withX(35.5);
        Position p_spike3_leave             = p_spike3_pre.clone();

        Position p_gate_pre1                = TB_Misc.redOrBlue(5, -29, -90);
        Position p_gate_pre2                = TB_Misc.redOrBlue(7, -46, -90);
        Position p_gate_tap                 = TB_Misc.redOrBlue(7, -54, -90);
//        Position p_gate_gather              = redOrBlue(12.5, -58.75, -120);
        Position p_gate_gather              = TB_Misc.redOrBlue(14, -59.75, -120);  // -58.75
        Position p_gate_leave1              = TB_Misc.redOrBlue(15, -54, -120);
        Position p_gate_leave2              = p_spike2_pre.clone();

        autoSpeed = 1.0;  //0.5;

        smAutoTestGotoCenter = new StateMachine("ATCenter");
        task = smAutoTestGotoCenter;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetAccurate(p_center)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.stop());

        smAutoTestGotoNearShoot = new StateMachine("ATNearShoot");
        task = smAutoTestGotoNearShoot;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_nearShoot)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoTestGotoSpike1 = new StateMachine("ATSpike1");
        task = smAutoTestGotoSpike1;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike1_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike1_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike1_pre)) ); //toTargetMedium
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );

        smAutoTestGotoSpike2 = new StateMachine("ATSpike2");
        task = smAutoTestGotoSpike2;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike2_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike2_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike2_leave)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoTestGotoSpike3 = new StateMachine("ATSpike3");
        task = smAutoTestGotoSpike3;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike3_pre)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike3_fin)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_spike3_leave)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoTestDriveToGate = new StateMachine("ATDriveToGate");
        task = smAutoTestDriveToGate;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_pre1)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_pre2)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);

        smAutoTestOperateGate = new StateMachine("ATOperateGate");
        task = smAutoTestOperateGate;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Tasks.smIntakeOn.restart() );
//        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_tap)) );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_gather)) );
        task.addWaitFor(() -> TB_Intake.definitely3, 3000);
        task.addRunOnce(() -> parts.autoDrive.stop());  // in case it's still trying to navigate
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_leave1)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
        task.addRunOnce(() -> TB_Tasks.smIntakeOff.restart() );
        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetMedium(p_gate_leave2)) );
        task.addWaitFor(() -> !parts.autoDrive.isNavigating);


        smTieItAllTogether = new StateMachine("TieItAllTogether");
        task = smTieItAllTogether;
        task.setGroups("orchestrator");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            TB_Turret.armTurret(true);
            TB_Turret.armSpinner(true);
        });

        task.addRunOnce(() -> parts.autoDrive.addNavTargets(toTargetTransition(p_start_move1)) );
//        task.addWaitFor(() -> !parts.autoDrive.isNavigating);
//        task.addRunOnce(() -> parts.autoDrive.stop());

        task.addRunOnce(smAutoTestGotoNearShoot::restart);
        task.addWaitFor(smAutoTestGotoNearShoot::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoTestGotoSpike2::restart);
        task.addWaitFor(smAutoTestGotoSpike2::isDone);
        task.addRunOnce(smAutoTestGotoNearShoot::restart);
        task.addWaitFor(smAutoTestGotoNearShoot::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoTestDriveToGate::restart);
        task.addWaitFor(smAutoTestDriveToGate::isDone);
        task.addRunOnce(smAutoTestOperateGate::restart);
        task.addWaitFor(smAutoTestOperateGate::isDone);
        task.addRunOnce(smAutoTestGotoNearShoot::restart);
        task.addWaitFor(smAutoTestGotoNearShoot::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoTestDriveToGate::restart);
        task.addWaitFor(smAutoTestDriveToGate::isDone);
        task.addRunOnce(smAutoTestOperateGate::restart);
        task.addWaitFor(smAutoTestOperateGate::isDone);
        task.addRunOnce(smAutoTestGotoNearShoot::restart);
        task.addWaitFor(smAutoTestGotoNearShoot::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoTestDriveToGate::restart);
        task.addWaitFor(smAutoTestDriveToGate::isDone);
        task.addRunOnce(smAutoTestOperateGate::restart);
        task.addWaitFor(smAutoTestOperateGate::isDone);
        task.addRunOnce(smAutoTestGotoNearShoot::restart);
        task.addWaitFor(smAutoTestGotoNearShoot::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);

        task.addRunOnce(smAutoTestGotoSpike1::restart);
        task.addWaitFor(smAutoTestGotoSpike1::isDone);
        task.addRunOnce(smAutoTestGotoNearShoot::restart);
        task.addWaitFor(smAutoTestGotoNearShoot::isDone);
        task.addRunOnce(TB_Tasks.smLaunch::restart);
        task.addWaitFor(TB_Tasks.smLaunch::isDone);


        Position posStart = new Position(-24,0,-90);
        Position posOpp   = new Position(24,0,90);
        Position posCenter = new Position(0,0,0);
        int timeLimit = 5000;
        double speed = 0.75;

        smAutoTestTransitions = new StateMachine("ATTransitions");
        task = smAutoTestTransitions;
        task.setGroups("autotest");  // will be killed by
        task.setAutoRestart(false);
        ////// NEED TO WRITE THIS
        ////// See: parts.dsAuto.testAutoMethod4();

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

    static NavigationTarget toTarget (Position pos, PositionTolerance tolerance, double maxSpeed, long timeLimit, boolean noSlow) {
        return new NavigationTarget(pos, tolerance, maxSpeed, timeLimit, noSlow);
    }

    // These could also be stored in TB_Misc (and in fact they are!)
    static PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
    static PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
    static PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
    static PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
    static PositionTolerance toleranceTransition = new PositionTolerance(6.0,90.0,0);


    // added for testing transition paths 20260603
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
