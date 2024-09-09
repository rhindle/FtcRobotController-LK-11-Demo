package org.firstinspires.ftc.teamcode.RobotParts.Common;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

public class TelemetryMgr {

    static int debugLevel = 10;
    static boolean needsUpdate = false;
    static LinearOpMode opMode;
    public static boolean[] showCategory;

    public TelemetryMgr(LinearOpMode opMode) {
        construct(opMode);
    }

    void construct(LinearOpMode opMode){
        TelemetryMgr.opMode = opMode;

        showCategory = new boolean[Category.values().length];
        enableDefaultCategories();
    }

    /* message with debuglevel */
    public static void message(int lvl, String cap, Object val) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addData(addNumber(lvl, cap), val);
            needsUpdate = true;
        }
    }

    public static void message(int lvl, String cap, String fmt, Object... args) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addData(addNumber(lvl, cap), fmt, args);
            needsUpdate = true;
        }
    }

    public static void message(int lvl, String cap) {
        if (lvl <= debugLevel) {
            opMode.telemetry.addLine(addNumber(lvl, cap));
            needsUpdate = true;
        }
    }

    /* message with category */
    public static void message(Category category, String cap, Object val) {
        if (showCategory[category.ordinal()]) {
            opMode.telemetry.addData(addLabel(category, cap), val);
            needsUpdate = true;
        }
    }

    public static void message(Category category, String cap, String fmt, Object... args) {
        if (showCategory[category.ordinal()]) {
            opMode.telemetry.addData(addLabel(category, cap), fmt, args);
            needsUpdate = true;
        }
    }

    public static void message(Category category, String cap) {
        if (showCategory[category.ordinal()]) {
            opMode.telemetry.addLine(addLabel(category, cap));
            needsUpdate = true;
        }
    }

    public static void Update () {
        if (needsUpdate) opMode.telemetry.update();
        needsUpdate = false;
    }

    public static void Update (boolean forceUpdate) {
        if (forceUpdate) opMode.telemetry.update();
        needsUpdate = false;
    }

    public static void setDebugLevel (int lvl) {
        debugLevel = lvl;
    }

    public static void enableCategories(Category[] categories) {
        for (Category category : categories) {
            showCategory[category.ordinal()] = true;
        }
    }

    public static void disableCategories (Category[] categories) {
        for (Category category : categories) {
            showCategory[category.ordinal()] = false;
        }
    }

    public static void disableEXTCategories () {
        for (int i=0; i<showCategory.length; i++) {
            if (Category.values()[i].name().contains("_EXT")) showCategory[i]=false;
        }
    }

    public static void enableAllCategories() {
        Arrays.fill(showCategory, true);
    }

    public static void enableDefaultCategories() {
        Arrays.fill(showCategory, false);
        showCategory[Category.MANDATORY.ordinal()] = true;
        showCategory[Category.BASIC.ordinal()] = true;
    }

    @SuppressLint("DefaultLocale")
    static String addNumber(int lvl, String cap) {
       return String.format("%02d", lvl) + " " + cap;
    }

    static String addLabel(Category cat, String cap) {
        return cat.label + "| " + cap;
    }

    public enum Category {
        BASIC("-------"),
        MANDATORY(">>>"),
        SENSORS("SNS"),
        SENSORS_EXT("SNX"),
        IMU("IMU"),
        IMU_EXT("IMX"),
        SLAMRA("SLM"),
        SLAMRA_EXT("SLX"),
        ODOMETRY("ODO"),
        ODOMETRY_EXT("ODX"),
        AUTODRIVE("ADR"),
        AUTODRIVE_EXT("ADX"),
        USERDRIVE("UDR"),
        USERDRIVE_EXT("UDX"),
        POSITION("POS"),
        POSITION_EXT("POX"),
        DRIVETRAIN("DVT"),
        CONTROLS("CON"),
        APRILTAG("TAG"),
        APRILTAG_EXT("TAX"),
        SPEED("SPD"),
        DISCSHOOTER("DSS"),
        NAVIGATOR("NAV");   //navigator is deprecated

        public final String label;

        Category(String label) {
            this.label = label;
        }
   }
}