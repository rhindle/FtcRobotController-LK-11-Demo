package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import org.firstinspires.ftc.teamcode.Tools.Functions;

import androidx.annotation.NonNull;

public class DriveData {

    public double driveSpeed, driveAngle, rotate;

    public DriveData() {
        this.driveSpeed = 0;
        this.driveAngle = 0;
        this.rotate = 0;
    }

    public DriveData(double driveSpeed, double driveAngle, double rotate) {
        this.driveSpeed = driveSpeed;
        this.driveAngle = driveAngle;
        this.rotate = rotate;
    }

    // regular mecanum drive
    public DriveData (float leftStickX, float leftStickY, float rightStickX ) {
        float X = -leftStickY;
        float Y = -leftStickX;
        this.driveSpeed = Functions.mathHypotenuse(X, Y);
        this.driveAngle = Math.toDegrees(Math.atan2(Y, X));
        this.rotate = rightStickX;
    }

    // forza drive
    // robot x direction is triggers, robot y direction is right stick x, rotate is left stick x
    public DriveData (float leftTrigger, float rightTrigger, float leftStickX, float rightStickX ) {
        float X = rightTrigger - leftTrigger;
        float Y = -rightStickX;
        this.driveSpeed = Functions.mathHypotenuse(X, Y);
        this.driveAngle = Math.toDegrees(Math.atan2(Y, X));
        this.rotate = leftStickX;
    }

    @NonNull
    public DriveData clone() {
        return new DriveData(driveSpeed, driveAngle, rotate);
    }

    public String toString(int decimals){
        return      "Speed:" + String.format("%."+ decimals +"f", driveSpeed)
                + ", Angle:" + String.format("%."+ decimals +"f", driveAngle)
                + ", Rotate:" + String.format("%."+ decimals +"f", rotate);
    }
}
