package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Functions {

    private static final ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Hypotenuse function originally used in Blocky
    public static double mathHypotenuse(float arg0, float arg1) {
        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
    }
    public static double mathHypotenuse(double arg0, double arg1) {
        return Math.sqrt(Math.pow(arg0, 2) + Math.pow(arg1, 2));
    }

    public static double normalizeAngle(double angle) {
        // normalize angle A to -179 to +180 range
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public static double calculateLoopTime() {
        double timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }

    public static double interpolate(double x, double x1, double x2, double y1, double y2) {
        return y1 + (x-x1)*(y2-y1)/(x2-x1);
    }

    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static double averageAngle(double... anglesDeg) {
        double xCos = 0.0;
        double ySin = 0.0;
        for (double angle : anglesDeg) {
            xCos += Math.cos(Math.toRadians(normalizeAngle(angle)));
            ySin += Math.sin(Math.toRadians(normalizeAngle(angle)));
        }
        xCos /= anglesDeg.length;
        ySin /= anglesDeg.length;
        return normalizeAngle(Math.toDegrees(Math.atan2(ySin, xCos)));
    }

}