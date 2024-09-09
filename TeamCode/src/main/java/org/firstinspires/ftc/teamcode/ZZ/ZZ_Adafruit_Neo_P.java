package org.firstinspires.ftc.teamcode.ZZ;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.LegacyBots.Robot;

@TeleOp (name="AA_Adafruit_Neo_P", group="Test")
//@Disabled
public class ZZ_Adafruit_Neo_P extends LinearOpMode {

    Robot robot;
    ButtonMgr buttonMgr;
    NeoMatrix neo;

    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;
    int[][] textMatrix;
    int[][] textMatrix1;
    int[][][] pacman = new int[6][8][8];
    int[][][][] ghosts = new int[5][2][7][8];
    long lastAnimUpdate;
    int updateCount = 0;
    int animUpdateInterval = 200;  //250
    int pacCount = 0;
    int ghostCount = 0;

    //AdafruitNeoDriver neo = null;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        buttonMgr = new ButtonMgr(this);
        neo = new NeoMatrix(this, "neo");

        robot.init();
        neo.initialize();
        neo.flipVert=true;
        //neo.flipHoriz=true;
        neo.setUpdateLimit(0); //1//4
        neo.setPreventTearing(true);
        neo.setDimmingValue(255);

        //sleep(250);

//        neo.drawLine(0,0,31,0, Color.rgb(10,10,10));
//        neo.drawLine(0,1,31,1, Color.rgb(20,0,0));
//        neo.drawLine(0,2,31,2, Color.rgb(15,5,0));
//        neo.drawLine(0,3,31,3, Color.rgb(10,10,0));
//        neo.drawLine(0,4,31,4, Color.rgb(0,20,0));
//        neo.drawLine(0,5,31,5, Color.rgb(0,0,20));
//        neo.drawLine(0,6,31,6, Color.rgb(10,0,10));
//        neo.drawLine(0,7,31,7, Color.rgb(10,10,10));
        neo.drawRectangle(0,31,0,7,Color.rgb(0,0,0),true,0);
        neo.forceUpdateMatrix();

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();
        lastAnimUpdate = System.currentTimeMillis();

        neo.clearCols(0,31);
//        neo.drawRectangle(0,31,0,7,Color.rgb(10,0,0));
        pacman[0] = neo.buildPixelMapFromString("a", neo.pacFont, Color.rgb(10,10,0));
        pacman[1] = neo.buildPixelMapFromString("b", neo.pacFont, Color.rgb(10,10,0));
        pacman[2] = neo.buildPixelMapFromString("c", neo.pacFont, Color.rgb(10,10,0));
        pacman[3] = neo.buildPixelMapFromString("d", neo.pacFont, Color.rgb(10,10,0));
        pacman[4] = pacman[2];
        pacman[5] = pacman[1];
        ghosts[0][0] = neo.buildPixelMapFromString("G", neo.pacFont, Color.rgb(15,0,0));
        ghosts[0][1] = neo.buildPixelMapFromString("g", neo.pacFont, Color.rgb(15,0,0));
        ghosts[1][0] = neo.buildPixelMapFromString("G", neo.pacFont, Color.rgb(10,5,5));
        ghosts[1][1] = neo.buildPixelMapFromString("g", neo.pacFont, Color.rgb(10,5,5));
        ghosts[2][0] = neo.buildPixelMapFromString("G", neo.pacFont, Color.rgb(0,10,10));
        ghosts[2][1] = neo.buildPixelMapFromString("g", neo.pacFont, Color.rgb(0,10,10));
        ghosts[3][0] = neo.buildPixelMapFromString("G", neo.pacFont, Color.rgb(10,5,0));
        ghosts[3][1] = neo.buildPixelMapFromString("g", neo.pacFont, Color.rgb(10,5,0));

        textMatrix = neo.buildPixelMapFromString("E", neo.pacFont, Color.rgb(10,10,10));
        textMatrix1 = neo.buildPixelMapFromString("e", neo.pacFont, Color.rgb(0,0,5));

        for (int i=0; i<4; i++) {
            for (int j=0; j<2; j++) {
                ghosts[i][j] = neo.overlayPixelMap(textMatrix, ghosts[i][j],0);
                ghosts[i][j] = neo.overlayPixelMap(textMatrix1, ghosts[i][j],0);
            }
        }

        ghosts[4][0] = neo.buildPixelMapFromString("H", neo.pacFont, Color.rgb(20,0,0));
        ghosts[4][1] = neo.buildPixelMapFromString("h", neo.pacFont, Color.rgb(20,0,0));
        textMatrix = neo.buildPixelMapFromString("I", neo.pacFont, Color.rgb(5,5,5));
        textMatrix1 = neo.buildPixelMapFromString("i", neo.pacFont, Color.rgb(5,5,5));
        ghosts[4][0] = neo.overlayPixelMap(textMatrix, ghosts[4][0],0);
        ghosts[4][1] = neo.overlayPixelMap(textMatrix1, ghosts[4][1],0);

//        textMatrix = neo.buildPixelMapFromString("  FTC Legal I2C LED Controller: Adafruit NeoDriver 5766      ", neo.compactFull, Color.rgb(20,20,20), Color.rgb(0,0,0));
//        textMatrix1 = neo.buildPixelMapFromString("  FTC Legal", neo.compactFull, Color.rgb(0,5,0), Color.rgb(0,0,0));
//        textMatrix = neo.underlayPixelMap(textMatrix1,textMatrix,1);

//        textMatrix = neo.shiftPixelMap(textMatrix,0,1, true);
//        textMatrix1 = neo.buildPixelMapFromString("SENSE", neo.littleLettersSq, Color.rgb(20,0,0), 0);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
//        textMatrix1 = neo.buildPixelMapFromString("+", neo.littleLettersSq, Color.rgb(10,10,10), 0);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,1, true);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
//        textMatrix1 = neo.buildPixelMapFromString("SOUND ", neo.littleLettersSq, Color.rgb(0,20,0), 0);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,3, true);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);
//        textMatrix1 = neo.buildPixelMapFromString("ROBOTICS", neo.littleLettersSq, Color.rgb(0,0,20), 0);
//        textMatrix1 = neo.shiftPixelMap(textMatrix1,0,2, true);
//        textMatrix = neo.appendPixelMap(textMatrix, textMatrix1);

//        neo.drawRectangle(0,31,0,7,Color.rgb(10,0,0),true,Color.rgb(0,0,0));
//        neo.applyPixelMapToBuffer(textMatrix,0,31, 0, false);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.loop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            neo.runLoop();

            if (System.currentTimeMillis() >= lastAnimUpdate + animUpdateInterval) {
                lastAnimUpdate = System.currentTimeMillis();
                textMatrix = neo.buildPixelMapFromString("`", neo.pacFont, Color.rgb(0,0,0));
                textMatrix1 = neo.cloneArray(textMatrix);
                for (int i=0; i<4; i++) {
                    textMatrix1 = neo.appendPixelMap(textMatrix1, ghosts[3-i][ghostCount]);
                    textMatrix1 = neo.appendPixelMap(textMatrix1, textMatrix);
                }
                for (int i=0; i<4; i++) {
                    textMatrix1 = neo.appendPixelMap(textMatrix1, textMatrix);
                }
                textMatrix1 = neo.appendPixelMap(textMatrix1, pacman[pacCount]);
                int pLen = textMatrix1.length;
                textMatrix = neo.buildPixelMapFromString("````````````````````````````````", neo.pacFont, Color.rgb(0,0,0));
                textMatrix1 = neo.appendPixelMap(textMatrix1, textMatrix);
                int mLen = textMatrix1.length;

                //neo.scrollRegion(4, 27, 0, 7, -1, 0, true);  // 19,16
                textMatrix1 = neo.shiftPixelMap(textMatrix1,updateCount-pLen,0,true);
                //textMatrix = neo.shiftPixelMap(textMatrix,-1,0,true);

//                neo.drawRectangle(0,31,0,7,Color.rgb(10,0,0),true,Color.rgb(0,0,0));
                neo.applyPixelMapToBuffer(textMatrix1,0,31, 0, true);

                if (++updateCount >= mLen) updateCount = 0;
                if (++pacCount > 5) pacCount = 0;
                if (++ghostCount > 1) ghostCount = 0;

            }

            sleep(8);

            loopyTime[loopyTimeCounter]=loopElapsedTime.milliseconds();
            loopyTimeCounter++;
            if (loopyTimeCounter >= loopySample) loopyTimeCounter = 0;
            double loopyTimeAverage = 0;
            for(int i=0; i<loopySample; i++) loopyTimeAverage+=loopyTime[i];
            loopyTimeAverage /= loopySample;

            telemetry.addData("Heading", "%.1f", robot.returnImuHeading());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopTimeAvg10(ms)","%.1f",loopyTimeAverage);
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();

        }
    }
}