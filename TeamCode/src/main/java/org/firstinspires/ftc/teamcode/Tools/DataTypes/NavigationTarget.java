package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import androidx.annotation.NonNull;

public class NavigationTarget {

    public Position targetPos;
    public PositionTolerance tolerance;
    public double maxSpeed;
    public long timeLimit;
    public boolean noSlow;
    // add pid coefficients?

    public NavigationTarget(Position targetPos, PositionTolerance tolerance, double maxSpeed, long timeLimit, boolean noSlow) {
        if (tolerance==null) tolerance=new PositionTolerance(2, 2, 125);   //todo: determine if this is a good default
        if (maxSpeed==0) maxSpeed = 1;
        if (timeLimit==0) timeLimit = 10000;  //todo: inserted for debugging
        this.targetPos = targetPos;
        this.tolerance = tolerance;
        this.maxSpeed = maxSpeed;
        this.timeLimit = timeLimit;
        this.noSlow = noSlow;
    }

    //without noslow
    public NavigationTarget(Position targetPos) {
        this(targetPos, null);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance) {
        this(targetPos, tolerance, 0.0);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance, double maxSpeed) {
        this(targetPos, tolerance, maxSpeed, 0, false);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance, long timeLimit) {
        this(targetPos, tolerance, 0.0, timeLimit, false);
    }
    public NavigationTarget(Position targetPos, double maxSpeed) {
        this(targetPos, null, maxSpeed, 0, false);
    }
    public NavigationTarget(Position targetPos, double maxSpeed, long timeLimit) {
        this(targetPos, null, maxSpeed, timeLimit, false);
    }
    public NavigationTarget(Position targetPos, long timeLimit) {
        this(targetPos, null, 0.0, timeLimit, false);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance, double maxSpeed, long timeLimit) {
        this(targetPos, tolerance, maxSpeed, timeLimit, false);
    }

    //with noslow
    public NavigationTarget(Position targetPos, boolean noSlow) {
        this(targetPos, null, noSlow);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance, boolean noSlow) {
        this(targetPos, tolerance, 0.0, noSlow);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance, double maxSpeed, boolean noSlow) {
        this(targetPos, tolerance, maxSpeed, 0, noSlow);
    }
    public NavigationTarget(Position targetPos, PositionTolerance tolerance, long timeLimit, boolean noSlow) {
        this(targetPos, tolerance, 0.0, timeLimit, noSlow);
    }
    public NavigationTarget(Position targetPos, double maxSpeed, boolean noSlow) {
        this(targetPos, null, maxSpeed, noSlow);
    }
    public NavigationTarget(Position targetPos, double maxSpeed, long timeLimit, boolean noSlow) {
        this(targetPos, null, maxSpeed, timeLimit, noSlow);
    }
    public NavigationTarget(Position targetPos, long timeLimit, boolean noSlow) {
        this(targetPos, null, 0.0, timeLimit, noSlow);
    }

    @NonNull
    public NavigationTarget clone() {
        return new NavigationTarget(targetPos, tolerance, maxSpeed, timeLimit, noSlow);
    }

    public boolean inTolerance (Position currentPos) {
        return tolerance.inTolerance(targetPos, currentPos);
    }
    public boolean inToleranceByTime (Position currentPos) {
        return tolerance.inToleranceByTime(targetPos, currentPos);
    }

}
