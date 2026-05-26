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
