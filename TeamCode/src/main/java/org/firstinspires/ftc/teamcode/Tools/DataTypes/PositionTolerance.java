package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import androidx.annotation.NonNull;

public class PositionTolerance {
    double X;
    double Y;
    double R;
    double dist;
    boolean xyPreferred;
    long toleranceTime = 0;
    long dwell = 0;

    public PositionTolerance(double X, double Y, double R, double dist, boolean xyPreferred, long dwell) {
        this.X = Math.abs(X);
        this.Y = Math.abs(Y);
        this.R = Math.abs(R);
        this.dist = Math.abs(dist);
        this.dwell = dwell;
        this.xyPreferred = xyPreferred;
    }

    public PositionTolerance() {
        this(0,0,0,0,false,0);
    }

    public PositionTolerance(double X, double Y, double R, long time) {
        this(X, Y, R, Math.sqrt(X*X + Y*Y), true, time);
    }
    public PositionTolerance(double X, double Y, double R) {
        this(X, Y, R,0);
    }

    public PositionTolerance(double dist, double R, long time) {
        this(dist, dist, R, dist, false, time);
    }
    public PositionTolerance(double dist, double R) {
        this(dist, dist, R, dist, false, 0);
    }

    public PositionTolerance(double X, double Y, double R, double dist, long time) {
        this(X, Y, R, dist, true, time);
    }
    public PositionTolerance(double X, double Y, double R, double dist) {
        this(X, Y, R, dist, true, 0);
    }

    public PositionTolerance(Position pos, long time) {
        this(pos.X, pos.Y, pos.R, Math.sqrt(pos.X*pos.X + pos.Y*pos.Y), true, time);
    }
    public PositionTolerance(Position pos) {
        this(pos,0);
    }

    public PositionTolerance(Vector2D vec, double R, long time) {
        this(vec.distance, vec.distance, R, vec.distance, false, time);
    }
    public PositionTolerance(Vector2D vec, double R) {
        this(vec.distance, vec.distance, R, vec.distance, false, 0);
    }

    public PositionTolerance(Position pos, Vector2D vec, long time) {
        this(pos.X, pos.Y, pos.R, vec.distance, true, time);
    }
    public PositionTolerance(Position pos, Vector2D vec) {
        this(pos.X, pos.Y, pos.R, vec.distance, true, 0);
    }

    public Position toPosition() {
        return new Position (X, Y, R);
    }

    @NonNull
    public PositionTolerance clone() {
        return new PositionTolerance(X, Y, R, dist, xyPreferred, dwell);
    }

    public boolean inTolerance (Position target, Position current) {
        Position posError = target.clone();
        posError.subtract(current);
        posError.normalize();
        double posErrorDist = Math.sqrt(posError.X*posError.X + posError.Y*posError.Y);
        posError.abs();
        if (xyPreferred) {
            if (posError.X <= X && posError.Y <= Y && posError.R <= R) return true;  // todo:should this be false otherwise?
        }
        if (posErrorDist <= dist && posError.R <= R) return true;
        toleranceTime = System.currentTimeMillis();
        return false;
    }

    public boolean inToleranceByTime (Position target, Position current, long duration) {
        if (inTolerance(target, current)) {
            if ((System.currentTimeMillis()-toleranceTime) > duration) return true;
        }
        return false;
    }
    public boolean inToleranceByTime (Position target, Position current) {
        return inToleranceByTime(target, current, dwell);
    }

    public boolean inToleranceExceptRotation (Position target, Position current) {
        Position posError = target.clone();
        posError.subtract(current);
        posError.normalize();
        double posErrorDist = Math.sqrt(posError.X*posError.X + posError.Y*posError.Y);
        posError.abs();
        if (xyPreferred) {
            if (posError.X <= X && posError.Y <= Y) return true;
        }
        if (posErrorDist <= dist) return true;
        return false;
    }
}
