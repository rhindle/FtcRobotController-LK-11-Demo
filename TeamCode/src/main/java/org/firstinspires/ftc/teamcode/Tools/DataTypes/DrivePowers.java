package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class DrivePowers {

    public double v0, v1, v2, v3;

    public DrivePowers(double v0, double v1, double v2, double v3) {
        this.v0 = v0;
        this.v1 = v1;
        this.v2 = v2;
        this.v3 = v3;
    }

    public DrivePowers(double[] powers) {
        v0 = powers[0];
        v1 = powers[1];
        v2 = powers[2];
        v3 = powers[3];
    }

    public DrivePowers() {
        v0 = 0;
        v1 = 0;
        v2 = 0;
        v3 = 0;
    }

    public void scaleMax (double max) {
        double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), max));
        v0 /= highValue;
        v2 /= highValue;
        v1 /= highValue;
        v3 /= highValue;
    }

    public void scaleAverage (double max) {
        double averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3)));
        averageValue = averageValue / max;
        if (averageValue > 1) {
            v0 /= averageValue;
            v2 /= averageValue;
            v1 /= averageValue;
            v3 /= averageValue;
        }
    }

    public DrivePowers clone() {
        return new DrivePowers(v0, v1, v2, v3);
    }

    public String toString(int decimals){
        return      "v0:" + String.format("%."+ decimals +"f", v0)
                + ", v1:" + String.format("%."+ decimals +"f", v1)
                + ", v2:" + String.format("%."+ decimals +"f", v2)
                + ", v3:" + String.format("%."+ decimals +"f", v3);
    }

}
