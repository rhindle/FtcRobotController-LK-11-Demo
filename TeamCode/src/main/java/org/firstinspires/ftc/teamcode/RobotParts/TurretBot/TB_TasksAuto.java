package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;

public class TB_TasksAuto {

    public static Parts parts;

    public static StateMachine smAutoTest1;


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
        Position p_fieldStart                = redOrBlue(-39.0,-55,-180); // TODO: Confirm/Tune this position.
//        Position p_obeliskView               = redOrBlue(-39.0, -31, 160);  // GoalBlue: ObeliskView Position
        Position p_launchPosZero             = redOrBlue(-28.0,-16,-135);    // GoalBlue Launching Position.
        Position p_launchPosOne              = redOrBlue(-28.0,-16,-135);    // GoalBlue Launching Position.
        Position p_launchPosFinal            = redOrBlue(-28.0,-16,-135);    // GoalBlue Launching Position for pinkServo. Z:???.

        Position p_pre_intakeArtifactRow2    = redOrBlue(14, -22, 90);   // X:12; Blue: Ready to collect on Row2
        Position p_intakeArtifactRow2        = redOrBlue(14, -60, 90);   // X:12; Blue: Intake Artifacts in Row2

        Position p_pre_intakeArtifactRow1    = redOrBlue(-12, -22, 90);  // Blue: Ready to collect on Row1
        Position p_intakeArtifactRow1        = redOrBlue(-12, -53, 90);  // Blue: Intake Artifacts in Row1

        //leave row 3 for far team?
//        Position p_pre_intakeArtifactRow3    = redOrBlue(35.5, -22, 90);   // Blue: Ready to collect in Row3
//        Position p_intakeArtifactRow3        = redOrBlue(35.5, -60, 90);   // Blue: Intake Artifacts in Row3

        Position p_pre_leverOpen             = redOrBlue(0, -45, -180);    // Blue: Pre-Open Lever Position
        Position p_leverOpen                 = redOrBlue(0, -55, -180);    // Blue: Open Lever Position
        Position p_parkAfterAuto             = redOrBlue(-12,-28,-180);

        // suggestion:  Make different actions different tasks with an orchestrator task that
        // runs them all and can adjust for time remaining, etc.
        // Do not give the orchestrator task the same group name or it will be stopped by the subtasks.



    }

    static Position redOrBlue (double X, double Y, double R) {
        if (TB_Misc.isAllianceRed()) {
            return new Position(X, -Y, -R);
        }
        else {
            return new Position(X, Y, R);
        }
    }
    
    static NavigationTarget toTargetTransition (Position pos) {
        return new NavigationTarget(pos, toleranceTransition, 0.5, 5000, true);
    }

    static NavigationTarget toTargetAccurate (Position pos) {
        return new NavigationTarget(pos, toleranceHigh, 0.5, 5000, false);
    }

    static NavigationTarget toTarget (Position pos, PositionTolerance tolerance, double maxSpeed, long timeLimit, boolean noSlow) {
        return new NavigationTarget(pos, tolerance, maxSpeed, timeLimit, noSlow);
    }

    // These could also be stored in TB_Misc (and in fact they are!)
    static PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
    static PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
    static PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
    static PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
    static PositionTolerance toleranceTransition = new PositionTolerance(4.0,90.0,0);
}
