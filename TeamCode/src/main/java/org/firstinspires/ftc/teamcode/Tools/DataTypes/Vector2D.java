package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import androidx.annotation.NonNull;

public class Vector2D {

    public double distance, angle;

    public Vector2D(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }

    public Vector2D() {
        distance = 0;
        angle = 0;
    }

    public Vector2D fromXY (double X, double Y) {
        return new Vector2D(Math.sqrt(X*X + Y*Y), Math.toDegrees(Math.atan2(Y, X)));
    }

    public Vector2D from2pos (Position pos1, Position pos2) {
        return fromXY(pos2.X-pos1.X, pos2.Y-pos1.Y);
    }

    public double X () {
        return distance * Math.cos(Math.toRadians(angle));
    }

    public double Y () {
        return distance * Math.sin(Math.toRadians(angle));
    }

    public Position XY () {
        return new Position(distance * Math.cos(Math.toRadians(angle)),distance * Math.sin(Math.toRadians(angle)));
    }

    @NonNull
    public Vector2D clone() { return new Vector2D(distance, angle);}
}
