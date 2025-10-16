package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.RobotV2;

@TeleOp (name="ZZ_TestBot_2025", group="Test")
//@Disabled
public class ZZ_TestBot_2025 extends LinearOpMode {

    RobotV2 robot;
    ButtonMgr buttonMgr;

    boolean[] srvReverse;
    boolean[] srvLive;
    boolean[] srvEnabled;
    double[] srvNewPos;
    static double[] srvOldPos;
    int numServos;
    int tgtServo;
    final double servoChange = .025;  //.0001;
//    final double largeChange = .005;

    boolean[] mtrReverse;
    boolean[] mtrEncoder;
    boolean[] mtrBrake;
    boolean[] mtrFullSpeed;
    double[] mtrNewPow;
    double[] mtrOldPow;
    int numMotors;
    int tgtMotor;

    String[] motorNicks;
    String[] encoderNicks;
    String[] servoNicks;
    String[] digitalNicks;
    String[] analogNicks;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new RobotV2(this);
        buttonMgr = new ButtonMgr(this);
        boolean dualHub = true;

        // Wait for the opMode to be "started" and allow configuration changes
        while (!isStarted()) {
            buttonMgr.updateAll();
            telemetry.addLine("===========  ZZ TestBot 2025 (init) ============");
            telemetry.addLine();
            telemetry.addLine("Press X for Single Hub, Y for Dual Hubs");
            telemetry.addLine();
            telemetry.addLine("Current Selection: " + (dualHub ? "Dual Hubs" : "Single Hub"));
            telemetry.update();
            if (buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasPressed)) {
                dualHub = false;
            }
            if (buttonMgr.getState(1, ButtonMgr.Buttons.y, ButtonMgr.State.wasPressed)) {
                dualHub = true;
            }
            sleep(10);
        }

        // Set up the robot and related variables; this is done after init so changes can be made.
        if (dualHub) {
            robot.motorNames = new String []  {
                    "motor0", "motor1", "motor2", "motor3",
                    "motor0B", "motor1B", "motor2B", "motor3B"
            };
            motorNicks = new String [] {"MA0", "MA1", "MA2", "MA3", "MB0", "MB1", "MB2", "MB3"};
            encoderNicks = new String [] {"EA0", "EA1", "EA2", "EA3", "EB0", "EB1", "EB2", "EB3"};
            robot.servoNames = new String[] {
                    "servo0", "servo1", "servo2", "servo3", "servo4", "servo5",
                    "servo0B", "servo1B", "servo2B", "servo3B", "servo4B", "servo5B"
            };
            servoNicks = new String [] {"SA0", "SA1", "SA2", "SA3", "SA4", "SA5", "SB0", "SB1", "SB2", "SB3", "SB4", "SB5"};
            robot.digitalNames = new String[] {
                    "digital0", "digital1", "digital2", "digital3", "digital4", "digital5", "digital6", "digital7",
                    "digital0B", "digital1B", "digital2B", "digital3B", "digital4B", "digital5B", "digital6B", "digital7B"
            };
            digitalNicks = new String [] {"DA0", "DA1", "DA2", "DA3", "DA4", "DA5", "DA6", "DA7", "DB0", "DB1", "DB2", "DB3", "DB4", "DB5", "DB6", "DB7"};
            robot.analogNames = new String[]{
                    "analog0", "analog1", "analog2", "analog3",
                    "analog0B", "analog1B", "analog2B", "analog3B"
            };
            analogNicks = new String [] {"AA0", "AA1", "AA2", "AA3", "AB0", "AB1", "AB2", "AB3"};
        } else {
            robot.motorNames = new String []  {
                    "motor0", "motor1", "motor2", "motor3"
            };
            motorNicks = new String [] {"MA0", "MA1", "MA2", "MA3"};
            encoderNicks = new String [] {"EA0", "EA1", "EA2", "EA3"};
            robot.servoNames = new String[] {
                    "servo0", "servo1", "servo2", "servo3", "servo4", "servo5"
            };
            servoNicks = new String [] {"SA0", "SA1", "SA2", "SA3", "SA4", "SA5"};
            robot.digitalNames = new String[] {
                    "digital0", "digital1", "digital2", "digital3", "digital4", "digital5", "digital6", "digital7"
            };
            digitalNicks = new String [] {"DA0", "DA1", "DA2", "DA3", "DA4", "DA5", "DA6", "DA7"};
            robot.analogNames = new String[]{
                    "analog0", "analog1", "analog2", "analog3"
            };
            analogNicks = new String [] {"AA0", "AA1", "AA2", "AA3"};
        }

        robot.initialize();

        numServos = robot.servoNames.length;
        srvReverse = new boolean[numServos];
        srvLive = new boolean[numServos];
        srvEnabled = new boolean[numServos];
        srvNewPos = new double[numServos];
        if (srvOldPos == null || srvOldPos.length != numServos) {
            srvOldPos = new double[numServos];
            for (int i = 0; i < numServos; i++) {
                srvOldPos[i] = 0.5;
            }
        }
        for (int i = 0; i < numServos; i++) {
            srvReverse[i] = false;
            srvLive[i] = false;
            srvEnabled[i] = false;
            srvNewPos[i] = srvOldPos[i];
        }

        numMotors = robot.motorNames.length;
        mtrReverse = new boolean[numMotors];
        mtrEncoder = new boolean[numMotors];
        mtrBrake = new boolean[numMotors];
        mtrFullSpeed = new boolean[numMotors];
        mtrNewPow = new double[numMotors];
        mtrOldPow = new double[numMotors];
        for (int i = 0; i < numMotors; i++) {
            mtrReverse[i] = false;
            mtrEncoder[i] = true;
            mtrBrake[i] = true;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            // *** Part 1 - Servos *** //

            // increment the tgtServo selection
            if (buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper, ButtonMgr.State.isRepeating)) {
                tgtServo++;
                if (tgtServo > numServos - 1) tgtServo = 0;
            }

            // set selected servo forward/srvReverse (X)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasPressed)) {
                srvReverse[tgtServo] = !srvReverse[tgtServo];
                robot.servoArray[tgtServo].setDirection(srvReverse[tgtServo] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            }

            // set selected servo to Live or not (A)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.a, ButtonMgr.State.wasPressed)) {
                srvLive[tgtServo] = !srvLive[tgtServo];
                if (srvLive[tgtServo]) srvOldPos[tgtServo] += 0.000001;    // for the initial go srvLive
            }

            // disable the selected servo (B)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.b, ButtonMgr.State.wasPressed)) {
                srvEnabled[tgtServo] = false;
                ((ServoImplEx)robot.servoArray[tgtServo]).setPwmDisable();
            }

            // neutral-ize the selected servo (Y)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.y, ButtonMgr.State.wasPressed)) {
                srvNewPos[tgtServo] = 0.5;
            }

            // modify the selected servo
            srvNewPos[tgtServo] += gamepad1.right_stick_y * -servoChange;
            srvNewPos[tgtServo] = Math.max(0, Math.min(1, srvNewPos[tgtServo]));

            // update the servo position if Live and the position has changed
            for (int i = 0; i < numServos; i++) {
                if (srvLive[i] && srvNewPos[i] != srvOldPos[i]) {
                    robot.servoArray[i].setPosition(srvNewPos[i]);
                    srvOldPos[i] = srvNewPos[i];
                    srvEnabled[i] = true;
                }
            }

            // *** Part 2 - Motors *** //

            // increment the tgtMotor selection
            if (buttonMgr.getState(1, ButtonMgr.Buttons.left_bumper, ButtonMgr.State.isRepeating)) {
                stopAllMotors();
                tgtMotor++;
                if (tgtMotor > numMotors - 1) tgtMotor = 0;
            }

            // set selected motor forward/reverse (left)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_left, ButtonMgr.State.wasPressed)) {
                mtrReverse[tgtMotor] = !mtrReverse[tgtMotor];
                robot.motorArray[tgtMotor].setDirection(mtrReverse[tgtMotor] ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
            }

            // set selected motor to brake or not (right)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_right, ButtonMgr.State.wasPressed)) {
                mtrBrake[tgtMotor] = !mtrBrake[tgtMotor];
                if (mtrBrake[tgtMotor]) {
                    robot.motorArray[tgtMotor].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                } else {
                    robot.motorArray[tgtMotor].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                }
            }

            // set selected motor to use encoder or not (down)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasPressed)) {
                mtrEncoder[tgtMotor] = !mtrEncoder[tgtMotor];
                if (mtrEncoder[tgtMotor]) {
                    robot.motorArray[tgtMotor].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    robot.motorArray[tgtMotor].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            // reset the encoder for the selected motor (up)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasPressed)) {
                robot.motorArray[tgtMotor].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                if (mtrEncoder[tgtMotor]) {
                    robot.motorArray[tgtMotor].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    robot.motorArray[tgtMotor].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            // set the motor to full power or not (trigger)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.left_trigger, ButtonMgr.State.wasPressed)) {
                mtrFullSpeed[tgtMotor] = !mtrFullSpeed[tgtMotor];
            }

            // modify the selected motor if none of the bound buttons were pressed
            mtrNewPow[tgtMotor] = -gamepad1.left_stick_y;
            mtrNewPow[tgtMotor] = Math.max(-1, Math.min(1, mtrNewPow[tgtMotor]));
            mtrNewPow[tgtMotor] *= mtrFullSpeed[tgtMotor] ? 1 : 0.25;

            // update the motor power if the power has changed
            for (int i = 0; i < numMotors; i++) {
                if (mtrNewPow[i] != mtrOldPow[i]) {
                    robot.motorArray[i].setPower(mtrNewPow[i]);
                    mtrOldPow[i] = mtrNewPow[i];
                }
            }

            // *** Part 3 - Other *** //

            if (buttonMgr.getState(1, ButtonMgr.Buttons.back, ButtonMgr.State.wasPressed)) {
                stopEverything();
            }

            // *** Part 4 - Telemetry *** //

            telemetry.addLine("==============  ZZ TestBot 2025  ==============");
            telemetry.addLine();
            telemetry.addLine("up/down to select servo   |  doubletap a/b/x/y to bind");
            telemetry.addLine("left for forward/srvReverse    |  right for srvLive/not");
            telemetry.addLine("back for disable                 |  start for neutral (0.5)");
            telemetry.addLine();
            telemetry.addLine("hold a/b/x/y to change position for bound servos");
            telemetry.addLine("otherwise tgtServo servo only will change");
            telemetry.addLine("  left stick for large changes");
            telemetry.addLine("  right stick for small changes");
            telemetry.addLine();
            telemetry.addLine("[sel] [name] [bind] [srvLive][srvEnabled][dir] [current] [new]");
            telemetry.addLine();

            // Build telemetry strings
            for (int i = 0; i < numServos; i++) {
                String telString;
                telString = (i == tgtServo) ? "=>  " : "      ";
                telString += (robot.servoNames[i] + "               ").substring(0, 10);
//                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + "    ";
                telString += (srvLive[i] ? "L" : "_"); // + " ";
                telString += (srvEnabled[i] ? "E" : "_"); // + " ";
                telString += (srvReverse[i] ? "R" : "F") + "    ";
                telString += String.format("%.3f", srvOldPos[i]) + "    ";
                telString += String.format("%.3f", srvNewPos[i]);
                telString += (i == tgtServo) ? "  <=" : "      ";
                telemetry.addLine(telString);
            }

            telemetry.update();

        }

    }

    public void stopEverything() {
        stopAllMotors();
        stopAllServos();
    }

    public void stopAllMotors () {
        for (int i = 0; i < numMotors; i++) {
            robot.motorArray[i].setPower(0);
            mtrOldPow[i] = 0;
            mtrNewPow[i] = 0;
        }
    }

    public void stopAllServos () {
        for (int i = 0; i < numServos; i++) {
            srvEnabled[i] = false;
            ((ServoImplEx)robot.servoArray[i]).setPwmDisable();
        }
    }
}