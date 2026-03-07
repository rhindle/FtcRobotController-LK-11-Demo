package org.firstinspires.ftc.teamcode.ZZ;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Tools.i2c.QwiicLEDStickLK;


@TeleOp (name="ZZ_TestBot_LEDStick3", group="Test")
//@Disabled
public class ZZ_TestBot_LEDStick3 extends LinearOpMode {

    ZZ_Robot_2025 robot;
    ZZ_ButtonMgr buttonMgr;
    QwiicLEDStickLK qled = null;

    int[] bufferTest = {0,0,0};
    int[] bufferDesired = {0,0,0};
    int[] bufferActual = {0,0,0};
    int[] bufferColors = {0,0,0};
    int pointer = 0;
    boolean updateLED = false;

    int loopySample = 50;
    double[] loopyTime = new double[loopySample];
    int loopyTimeCounter = 0;

    @Override
    public void runOpMode() {
        robot = new ZZ_Robot_2025(this);
        buttonMgr = new ZZ_ButtonMgr(this);
        robot.init();

        qled = hardwareMap.get(QwiicLEDStickLK.class, "ledstick");

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //angles = robot.sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", robot.sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            sleep(100);
        }

        ElapsedTime loopElapsedTime = new ElapsedTime();

        //qled.changeLength(20);
        qled.setBrightness(1);
        //qled.setDefaultBrightness(1);
        qled.turnAllOff();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();


            if (buttonMgr.wasPressed(1, ZZ_ButtonMgr.Buttons.a)) {
                updateLED = false;
                bufferTest = new int[] {0,0,0};
                bufferActual = new int[] {0,0,0};
                bufferDesired = new int[] {0,0,0};
                qled.turnAllOff();
            }

            if (buttonMgr.wasPressed(1, ZZ_ButtonMgr.Buttons.x)) {
                updateLED = true;
                if (++bufferTest[0] > 3) bufferTest[0] = 0;
                sendLedInfo(bufferTest);
            }

            if (buttonMgr.wasPressed(1, ZZ_ButtonMgr.Buttons.y)) {
                updateLED = true;
                if (++bufferTest[1] > 3) bufferTest[1] = 0;
                sendLedInfo(bufferTest);
            }

            if (buttonMgr.wasPressed(1, ZZ_ButtonMgr.Buttons.b)) {
                updateLED = true;
                if (++bufferTest[2] > 3) bufferTest[2] = 0;
                sendLedInfo(bufferTest);
            }

            updateLed();


            telemetry.addData("Heading", "%.1f", robot.sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

            loopyTime[loopyTimeCounter]=loopElapsedTime.milliseconds();
            loopyTimeCounter++;
            if (loopyTimeCounter >= loopySample) loopyTimeCounter = 0;
            double loopyTimeAverage = 0;
            for(int i=0; i<loopySample; i++) loopyTimeAverage+=loopyTime[i];
            loopyTimeAverage /= loopySample;

            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopTimeAvg10(ms)","%.1f",loopyTimeAverage);
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();
        }
        qled.turnAllOff();
    }

    public void sendLedInfo (int a, int b, int c) {
        bufferDesired = new int[] {a,b,c};
    }

    public void sendLedInfo (int[] a) {
        bufferDesired = new int[] {a[0], a[1], a[2]};
    }

    public void updateLed () {

        // just exit if not needed
        if (!updateLED) return;

        // this is the round robin pointer...  loops around for updating two led patches
        if (++pointer > 2) pointer = 0;

        // if the current round robin candidate is already set properly, just exit
        if (bufferDesired[pointer] == bufferActual[pointer]) return;

        // update different patches depending on the pointer
        switch (pointer) {
            case 0:
                bufferActual[0] = bufferDesired[0];
                bufferActual[1] = bufferDesired[1];
                updateColorValues();
                qled.setColorGroupX2(0,2,bufferColors[0],4,2,bufferColors[1]);
                break;
            case 1:
                bufferActual[1] = bufferDesired[1];
                bufferActual[2] = bufferDesired[2];
                updateColorValues();
                qled.setColorGroupX2(4,2,bufferColors[1],8,2,bufferColors[2]);
                break;
            case 2:
                bufferActual[0] = bufferDesired[0];
                bufferActual[2] = bufferDesired[2];
                updateColorValues();
                qled.setColorGroupX2(0,2,bufferColors[0],8,2,bufferColors[2]);
                break;
            default:
                break;
        }
    }

    public void updateColorValues () {
        for (int i=0; i<3; i++) {
            switch (bufferActual[i]) {
                case 0:
                    bufferColors[i] = Color.rgb(0, 0, 0);
                    break;
                case 1:
                    bufferColors[i] = Color.rgb(0, 255, 0);
                    break;
                case 2:
                    bufferColors[i] = Color.rgb(127, 0, 127);
                    break;
                case 3:
                    bufferColors[i] = Color.rgb(32, 0, 0);
                    break;
                default:
                    break;
            }
        }
    }

}