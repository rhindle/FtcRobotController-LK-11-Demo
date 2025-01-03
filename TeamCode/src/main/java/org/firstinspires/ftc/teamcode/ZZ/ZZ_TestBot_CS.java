package org.firstinspires.ftc.teamcode.ZZ;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.i2c.AdafruitNeoDriver;
import org.firstinspires.ftc.teamcode.Tools.i2c.QwiicLEDStickLK;

@TeleOp (name="AA_TestBot_CS", group="Test")
//@Disabled
public class ZZ_TestBot_CS extends LinearOpMode {


    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        float[] hsvValues = new float[3];
        final double gain = 1500; //255;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double distance = 0;
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red * gain)
                    .addData("Green", "%.3f", colors.green * gain)
                    .addData("Blue", "%.3f", colors.blue * gain);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
            telemetry.addData("Distance (cm)", "%.3f", distance);

            if (distance > 5) {
                telemetry.addData("Sample", "NONE");
            }
            else if (distance > 1.5) { //no pixel
                telemetry.addData("Sample", "YES (too far)");
            }
            else {
                int hue = (int) hsvValues[0];
                int pixel = 0;
                if (hue < 60) pixel = 1;                //red
                if (hue >= 60 && hue <= 160) pixel = 2; //yellow
                if (hue > 160) pixel = 3;               //blue
                switch (pixel) {
                    case 1:  //red
                        telemetry.addData("Sample", "RED");
                        break;
                    case 2:  //yellow
                        telemetry.addData("Sample", "YELLOW");
                        break;
                    case 3:  //blue
                        telemetry.addData("Sample", "BLUE");
                        break;
                    default:
                        //something went wrong
                        break;
                }
            }

            telemetry.update();
            sleep(100);
        }
    }
}