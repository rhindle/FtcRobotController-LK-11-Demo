package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import org.firstinspires.ftc.teamcode.Tools.Functions;

// testing - brought this in from Om's old code
public class Position
{
    public double X, Y, R;

    public Position(double X, double Y, double R){
        this.X = X;
        this.Y = Y;
        this.R = R;
    }

    public Position(double X, double Y){
        this.X = X;
        this.Y = Y;
        this.R = 0;
    }

    public Position(){
        X = 0;
        Y = 0;
        R = 0;
    }

    public Position switchXY(){
        return new Position(Y, X, R);
    }

    public String toString(int decimals){
        Position pos = round(decimals);
        return "X: " + pos.X + ", Y: " + pos.Y + ", R: " + pos.R;
    }

    public Position clone(){ return new Position(X, Y, R);}

    public void normalize() {
        R = Functions.normalizeAngle(R);
    }

    public void add(Position pos2){
        X += pos2.X;
        Y += pos2.Y;
        R += pos2.R;
    }

    public void subtract(Position pos2){
        X -= pos2.X;
        Y -= pos2.Y;
        R -= pos2.R;
    }

    public void divide(double divisor){
        X /= divisor;
        Y /= divisor;
        R /= divisor;
    }

    public void abs(){
        X = Math.abs(X);
        Y = Math.abs(Y);
        R = Math.abs(R);
    }

    public Position round(int decimals){
        return new Position(
                Math.round(X * Math.pow(10, decimals))/ Math.pow(10, decimals),
                Math.round(Y * Math.pow(10, decimals))/ Math.pow(10, decimals),
                Math.round(R * Math.pow(10, decimals))/ Math.pow(10, decimals)
        );
    }

    public Position getAbsDiff(Position pos2){
        Position diff = this.clone();
        diff.subtract(pos2);
        diff.abs();
        return diff;
    }

    public boolean isEqualTo (Position pos2) {
        return ((X == pos2.X) && (Y == pos2.Y) && (R == pos2.R));
    }

    public Position withR (double newR) {
        return new Position(X, Y, newR);
    }

    public void addVector (Vector2D vec) {
        X += vec.X();
        Y += vec.Y();
    }

    public void addVectorRelative (Vector2D vec) {
        vec.angle += R;
        X += vec.X();
        Y += vec.Y();
    }

//    public Position transPose(Position pos1, Position pos2) {
//        return new Position(
//                (pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R)))),
//                (pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R)))),
//                (pos1.R + pos2.R)
//        );
//    }

    public Position transformPosition(Position pos2) {
        return new Position(
                (X + (pos2.X*Math.cos(Math.toRadians(R)) - pos2.Y*Math.sin(Math.toRadians(R)))),
                (Y + (pos2.X*Math.sin(Math.toRadians(R)) + pos2.Y*Math.cos(Math.toRadians(R)))),
                (R + pos2.R)
        );
    }

//    public void transPose(Position pos2) {
//        X = X + (pos2.X*Math.cos(Math.toRadians(R)) - pos2.Y*Math.sin(Math.toRadians(R)));
//        Y = Y + (pos2.X*Math.sin(Math.toRadians(R)) + pos2.Y*Math.cos(Math.toRadians(R)));
//        R = R + pos2.R;
//    }


    //todo: do these make sense? I can't remember why I'd want to compare to "this"
    public boolean inTolerance (Position target, PositionTolerance tolerance) {
        return tolerance.inTolerance(target,this);
    }
    public boolean inToleranceByTime (Position target, PositionTolerance tolerance, long duration) {
        return tolerance.inToleranceByTime(target, this, duration);
    }
    public boolean inToleranceByTime (Position target, PositionTolerance tolerance) {
        return tolerance.inToleranceByTime(target, this);
    }
}

