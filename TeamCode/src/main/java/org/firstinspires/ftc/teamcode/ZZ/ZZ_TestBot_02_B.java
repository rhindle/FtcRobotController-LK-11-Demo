package org.firstinspires.ftc.teamcode.ZZ;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import android.graphics.Color;


// import com.qualcomm.robotcore.util.Range;
// import java.util.Locale;

@TeleOp (name="ZZ_TestBot_02B(J)", group="Test")
//@Disabled
public class ZZ_TestBot_02_B extends LinearOpMode {

    ZZ_Hardware_TestBot_B robot   = new ZZ_Hardware_TestBot_B();
    Orientation angles;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor0b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor2b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor3b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
//        while (!isStopRequested() && !robot.sensorIMU.isGyroCalibrated())  {
//            sleep(50);
//            idle();
//        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor0b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor1b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor2b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor3b.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor0b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor1b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor2b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motor3b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
//            angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData(">", "Robot Heading = %.1f", angles.firstAngle);
            telemetry.update();
            sleep(100);
        }

        double tgtPower;
        double tgtPosition = 0.5;

        //double counter = 0;

        ElapsedTime loopElapsedTime = new ElapsedTime();

        int tgtMotor = 0;
        int tgtServo = 0;

        boolean toggleLB = false;
        boolean toggleRB = false;
        boolean toggleA = false;
        boolean tgtServoLive = false;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //counter++;

            /* Color Sensor Section */
            //   Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
            //         (int) (robot.sensorColor.green() * SCALE_FACTOR),
            //       (int) (robot.sensorColor.blue() * SCALE_FACTOR),
            //     hsvValues);

            /* IMU */
//            angles = robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angles = robot.sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            /* Check for button presses to switch test motor & servo */
            // Left bumper switches motors
            if (gamepad1.left_bumper && !toggleLB) {
                stopAllMotors();
                tgtMotor++;
                if (tgtMotor>7) tgtMotor=0;
                toggleLB=true;
            }
            // Right bumper switches servos
            if (gamepad1.right_bumper && !toggleRB) {
                tgtServo++;
                if (tgtServo>11) tgtServo=0;
                toggleRB=true;
                /* Get the position of the target servo.
                 *  (This would be a lot nicer in an array) */
                switch (tgtServo) {
                    case 0: tgtPosition=robot.servo0.getPosition(); break;
                    case 1: tgtPosition=robot.servo1.getPosition(); break;
                    case 2: tgtPosition=robot.servo2.getPosition(); break;
                    case 3: tgtPosition=robot.servo3.getPosition(); break;
                    case 4: tgtPosition=robot.servo4.getPosition(); break;
                    case 5: tgtPosition=robot.servo5.getPosition(); break;
                    case 6: tgtPosition=robot.servo0b.getPosition(); break;
                    case 7: tgtPosition=robot.servo1b.getPosition(); break;
                    case 8: tgtPosition=robot.servo2b.getPosition(); break;
                    case 9: tgtPosition=robot.servo3b.getPosition(); break;
                    case 10: tgtPosition=robot.servo4b.getPosition(); break;
                    case 11: tgtPosition=robot.servo5b.getPosition(); break;
                    default: break;
                }
            }
            // Y button resets the encoder for the current target motor
            if (gamepad1.y) {
                switch (tgtMotor) {
                    case 0: robot.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 1: robot.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 2: robot.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 3: robot.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 4: robot.motor0b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 5: robot.motor1b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 6: robot.motor2b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    case 7: robot.motor3b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); robot.motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); break;
                    default: break;
                }
            }
            // A button toggles whether the servo position will be applied "Live"
            if (gamepad1.a & !toggleA) {
                tgtServoLive=!tgtServoLive;
                toggleA = true;
            }
            // Toggles for certain buttons to avoid repeating actions
            if (!gamepad1.left_bumper) toggleLB=false;
            if (!gamepad1.right_bumper) toggleRB=false;
            if (!gamepad1.a) toggleA=false;

            /* Check controls for changes to target motor power and target servo position */
            // Left Stick controls motor power
            // X button will make the motor full speed, otherwise only 25%
            // Right Stick adjusts target servo position
            // B button panic stops all motors (should no longer be necessary)
            // Start button sets servo to center position (to stop if in continuous mode)
            // !!! Can't seem to back the "back" button work ???
            tgtPower = -gamepad1.left_stick_y * (gamepad1.x ? 1 : 0.25);
            if (!gamepad1.start) {
                tgtPosition = Math.max(0, Math.min(1, tgtPosition - gamepad1.right_stick_y * .025));
            } else {
                // added "stop" function in case the servo is in continuous mode
                tgtPosition = 0.5;
            }
            if (gamepad1.b) {
                stopAllMotors();
                tgtPower=0;
            }

            /* Update the servo position */
            if (tgtServoLive) {
                switch (tgtServo) {
                    case 0: robot.servo0.setPosition(tgtPosition); break;
                    case 1: robot.servo1.setPosition(tgtPosition); break;
                    case 2: robot.servo2.setPosition(tgtPosition); break;
                    case 3: robot.servo3.setPosition(tgtPosition); break;
                    case 4: robot.servo4.setPosition(tgtPosition); break;
                    case 5: robot.servo5.setPosition(tgtPosition); break;
                    case 6: robot.servo0b.setPosition(tgtPosition); break;
                    case 7: robot.servo1b.setPosition(tgtPosition); break;
                    case 8: robot.servo2b.setPosition(tgtPosition); break;
                    case 9: robot.servo3b.setPosition(tgtPosition); break;
                    case 10: robot.servo4b.setPosition(tgtPosition); break;
                    case 11: robot.servo5b.setPosition(tgtPosition); break;
                    default: break;
                }
            }

            /* Set the motor power */
            switch (tgtMotor) {
                case 0: robot.motor0.setPower(tgtPower); break;
                case 1: robot.motor1.setPower(tgtPower); break;
                case 2: robot.motor2.setPower(tgtPower); break;
                case 3: robot.motor3.setPower(tgtPower); break;
                case 4: robot.motor0b.setPower(tgtPower); break;
                case 5: robot.motor1b.setPower(tgtPower); break;
                case 6: robot.motor2b.setPower(tgtPower); break;
                case 7: robot.motor3b.setPower(tgtPower); break;
                default: break;
            }

            // If needed, could disable the servo signal with Servo.getController().pwmDisable()

            /* Add extensive telemetry for debugging */
            telemetry.addLine()
                    .addData("Mtr",tgtMotor)
                    .addData("Pwr", "%.2f", tgtPower)
                    .addData("Srv", tgtServo)
                    .addData("Pos","%.3f", tgtPosition)
                    .addData("Live?", tgtServoLive);
            telemetry.addLine()
                    .addData("Encoder 0", robot.motor0.getCurrentPosition())
                    .addData("1", robot.motor1.getCurrentPosition())
                    .addData("2", robot.motor2.getCurrentPosition())
                    .addData("3", robot.motor3.getCurrentPosition())
                    .addData("4", robot.motor0b.getCurrentPosition())
                    .addData("5", robot.motor1b.getCurrentPosition())
                    .addData("6", robot.motor2b.getCurrentPosition())
                    .addData("7", robot.motor3b.getCurrentPosition());
            telemetry.addLine()
                    .addData("Motor 0", "%.2f", robot.motor0.getPower())
                    .addData("1", "%.2f", robot.motor1.getPower())
                    .addData("2", "%.2f", robot.motor2.getPower())
                    .addData("3", "%.2f", robot.motor3.getPower())
                    .addData("4", "%.2f", robot.motor0b.getPower())
                    .addData("5", "%.2f", robot.motor1b.getPower())
                    .addData("6", "%.2f", robot.motor2b.getPower())
                    .addData("7", "%.2f", robot.motor3b.getPower());
            telemetry.addLine()
                    .addData("S 0", "%.2f", robot.servo0.getPosition())
                    .addData("1","%.2f", robot.servo1.getPosition())
                    .addData("2","%.2f", robot.servo2.getPosition())
                    .addData("3","%.2f", robot.servo3.getPosition())
                    .addData("4","%.2f", robot.servo4.getPosition())
                    .addData("5","%.2f", robot.servo5.getPosition())
                    .addData("6","%.2f", robot.servo0b.getPosition())
                    .addData("7","%.2f", robot.servo1b.getPosition())
                    .addData("8","%.2f", robot.servo2b.getPosition())
                    .addData("9","%.2f", robot.servo3b.getPosition())
                    .addData("10","%.2f", robot.servo4b.getPosition())
                    .addData("11","%.2f", robot.servo5b.getPosition());
//            telemetry.addLine()
//                .addData("Color A", robot.sensorColor.alpha())
//                .addData("R", robot.sensorColor.red())
//                .addData("G", robot.sensorColor.green())
//                .addData("B", robot.sensorColor.blue());
//            telemetry.addLine()
//                .addData("Hue", "%.1f", hsvValues[0])
//                .addData("Distance (cm)", "%.01f", robot.sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.addLine()
                    .addData("Color", "%.1f", hsvValues[0]);
            telemetry.addLine()
                    .addData("D 0", (robot.digital0.getState() ? "T" : "F") +
                            " | 1 : " + (robot.digital1.getState() ? "T" : "F") +
                            " | 2 : " + (robot.digital2.getState() ? "T" : "F")  +
                            " | 3 : " + (robot.digital3.getState() ? "T" : "F")+
                            " | 4 : " + (robot.digital4.getState() ? "T" : "F") +
                            " | 5 : " + (robot.digital5.getState() ? "T" : "F") +
                            " | 6 : " + (robot.digital6.getState() ? "T" : "F") +
                            " | 7 : " + (robot.digital7.getState() ? "T" : "F"));
            telemetry.addLine()
                    .addData("D 0B", (robot.digital0b.getState() ? "T" : "F") +
                            " | 1B : " + (robot.digital1b.getState() ? "T" : "F") +
                            " | 2B : " + (robot.digital2b.getState() ? "T" : "F")  +
                            " | 3B : " + (robot.digital3b.getState() ? "T" : "F")+
                            " | 4B : " + (robot.digital4b.getState() ? "T" : "F") +
                            " | 5B : " + (robot.digital5b.getState() ? "T" : "F") +
                            " | 6B : " + (robot.digital6b.getState() ? "T" : "F") +
                            " | 7B : " + (robot.digital7b.getState() ? "T" : "F"));
            telemetry.addData("Heading", "%.1f", angles.firstAngle);
            //telemetry.addData("Counter", counter);
            //telemetry.addData("LoopSpeed","%.1f",calcLoopSpeed());
            telemetry.addData("LoopTime(ms)","%.1f",loopElapsedTime.milliseconds());
            telemetry.addData("LoopSpeed(lps)","%.1f",1/(loopElapsedTime.milliseconds()/1000));
            loopElapsedTime.reset();
            telemetry.update();
        }
    }

    public void stopAllMotors () {
        robot.motor0.setPower(0);
        robot.motor1.setPower(0);
        robot.motor2.setPower(0);
        robot.motor3.setPower(0);
        robot.motor0b.setPower(0);
        robot.motor1b.setPower(0);
        robot.motor2b.setPower(0);
        robot.motor3b.setPower(0);
    }

//    static int loopCounter = 0;
//    static double loopSpeed = 0;
//    static ElapsedTime loopElapsedTime = new ElapsedTime();
//
//    private double calcLoopSpeed() {
//
//        loopCounter += 1;
//        if (loopElapsedTime.milliseconds() > 1000) {
//            //loopSpeedText = JavaUtil.formatNumber(loopCounter / (elapsedTime.milliseconds() / 1000), 1);
//            loopSpeed = loopCounter / (loopElapsedTime.milliseconds() / 1000);
//            loopCounter = 0;
//            loopElapsedTime.reset();
//        }
//        return loopSpeed;
//    }

}
