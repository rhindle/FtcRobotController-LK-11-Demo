package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake.IntakeActions;

public class smAutoIntake {

    private static int state = 0;
    private static boolean complete = false;
    private static long waitTimer;

    //----State Machine Start-----
    public static void stateMachine() {
        if (complete) state = 0;
        if (state < 1) return;  // not running

//        Intake sample (no automation)
//
//        Start Spinner
//        Start reading distance sensor
//        If distance sensor < 2...
//        Start reading color sensor instead
//        If color legal...
//        reverse spinner for ___ ms
//        stop spinner
//        RETRACT
//                (?TRANSFER)
//        If color not legal...
//        reverse spinner for ___ ms
//        (? START OVER? MOVE FIRST?)

        if (state == 1) {                 // start spintaking!
            state++;
            SB_Intake.action(IntakeActions.SPINTAKE_ALMOSTFLOOR);
            SB_Intake.action(IntakeActions.SPINNER_IN);
        }
        if (state == 2) {                 // wait until a block is ingested
            if (getDistance() < 1.5) {
                state++;
//                if (getSampleType()) state=10;  // jump ahead
            }
        }
        if (state == 3) {                 // then check the color/type
            int sample = getSampleType();
            if (sample > 0) {
                state++;
                if (sample==1 && SB_Intake.isRedLegal) state=10;
                if (sample==2 && SB_Intake.isYellowLegal) state=10;
                if (sample==3 && SB_Intake.isBlueLegal) state=10;
            }
        }
        if (state == 4) {                 // got bad sample, eject
            SB_Intake.action(IntakeActions.SPINNER_OUT);
            waitTimer = System.currentTimeMillis() + 1500;
            state++;
        }
        if (state == 5) {
            if (System.currentTimeMillis() > waitTimer) {
                state=1;  // start over
                SB_Intake.action(IntakeActions.SPINNER_OFF);
            }
        }

        if (state == 10) {                 // got good sample, eject possible second sample
            SB_Intake.action(IntakeActions.SPINNER_SLOWOUT);
            waitTimer = System.currentTimeMillis() + 500;
            state++;
        }
        if (state == 11) {
            if (System.currentTimeMillis() > waitTimer) {
                state++;
                SB_Intake.action(IntakeActions.SPINNER_OFF);
            }
        }
        if (state == 12) {                 // park
            complete = true;
            SB_Intake.action(IntakeActions.AUTO_TRANSFER);
        }
    }
    //----State Machine End-----

    static double getDistance() {
        return ((DistanceSensor) SB_Intake.sensorColor).getDistance(DistanceUnit.CM);
    }

    static int getSampleType() {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = SB_Intake.sensorColor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        int hue = (int) hsvValues[0];
        int type = -1;
        if (hue < 60) type = 1;                //red     // red sometimes triggers as 60. Better to wait for a better read?
        if (hue >= 65 && hue <= 160) type = 2; //yellow
        if (hue > 160) type = 3;               //blue
        if (hue == 0) type = 0;
//        if (type==1 && SB_Intake.isRedLegal) return true;
//        if (type==2 && SB_Intake.isYellowLegal) return true;
//        if (type==3 && SB_Intake.isBlueLegal) return true;
//        return false;
        SB_Intake.lastHue = hue;  // for debugging
        SB_Intake.lastType = type;
        return type;
    }

    public static void start() {
        SB_Intake.cancelStateMachines();
        complete = false;
        state = 1;
    }

    public static void stop() {
        // make it safer somehow?
        state = -1;
    }

    public static void mildStop() {
        state = -1;
    }

    public static boolean isRunning() {
        return (state>0);
    }

    public static boolean isComplete() {
        return complete;
    }

    public static int getState() {
        return state;
    }
}
