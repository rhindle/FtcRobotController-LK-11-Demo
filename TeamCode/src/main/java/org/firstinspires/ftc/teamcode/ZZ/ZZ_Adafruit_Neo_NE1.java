package org.firstinspires.ftc.teamcode.ZZ;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;

@TeleOp (name="AA_Adafruit_Neo_NE1", group="Test")
//@Disabled
public class ZZ_Adafruit_Neo_NE1 extends LinearOpMode {

    ZZ_Robot_2025 robot;
    ButtonMgr buttonMgr;
    NeoMatrix neo;

    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;
    int[][] textMatrix;
    int[][] textMatrix1;
    long lastAnimUpdate;
    int animUpdateInterval = 100;  //250

    //AdafruitNeoDriver neo = null;

    @Override
    public void runOpMode() {
        robot = new ZZ_Robot_2025(this);
        buttonMgr = new ButtonMgr(this);
        neo = new NeoMatrix(this, "neo");

        robot.motorNames = new String[] { };
        robot.servoNames = new String[] { };
        robot.digitalNames = new String[] { };
        robot.analogNames = new String[] { };



        robot.init();
        neo.initialize();
        neo.setUpdateLimit(0);



        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();
        lastAnimUpdate = System.currentTimeMillis();

//        neo.drawRectangle(0,7,0,7, Color.rgb(5,0,0), true, Color.rgb(0,1,0));
//        neo.drawRectangle(0,3,0,3, Color.rgb(0,4,0), true, Color.rgb(0,4,0));
//        neo.drawRectangle(0,3,4,7, Color.rgb(3,3,0), true, Color.rgb(3,3,0));
//        neo.drawRectangle(28,31,0,3, Color.rgb(2,2,2), true, Color.rgb(2,2,2));
//        neo.drawRectangle(28,31,4,7, Color.rgb(0,0,4), true, Color.rgb(0,0,4));
//        textMatrix1 = neo.buildPixelMapFromString("     f ", neo.specialChar, Color.rgb(10,5,0), 0);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("   27050 ", neo.littleLetters, Color.rgb(10,10,10), 0);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,2, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("SENSE", neo.littleLetters, Color.rgb(10,0,0), 0);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("+", neo.littleLetters, Color.rgb(5,5,5), 0);
        //textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("SOUND ", neo.littleLetters, Color.rgb(0,10,0), 0);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,3, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        textMatrix1 = neo.buildPixelMapFromString("ROBOTICS", neo.littleLetters, Color.rgb(0,0,10), 0);
        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,2, true);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
        neo.applyPixelMapToBuffer(textMatrix,4,27, 0, true);
//        neo.applyPixelMapToBuffer(textMatrix,7,31, 0, true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            neo.runLoop();

            if (System.currentTimeMillis() >= lastAnimUpdate + animUpdateInterval) {
                lastAnimUpdate = System.currentTimeMillis();
                //neo.scrollRegion(4, 27, 0, 7, -1, 0, true);  // 19,16
                textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);
//                textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);
//                neo.applyPixelMapToBuffer(textMatrix,4,27, 0, true);
                neo.applyPixelMapToBuffer(textMatrix,0,31, 0, true);
            }

            sleep(8);

            loopyTime[loopyTimeCounter]=loopElapsedTime.milliseconds();
            loopyTimeCounter++;
            if (loopyTimeCounter >= loopySample) loopyTimeCounter = 0;
            double loopyTimeAverage = 0;
            for(int i=0; i<loopySample; i++) loopyTimeAverage+=loopyTime[i];
            loopyTimeAverage /= loopySample;

//            telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopTimeAvg10(ms)","%.1f",loopyTimeAverage);
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();

        }
    }
}