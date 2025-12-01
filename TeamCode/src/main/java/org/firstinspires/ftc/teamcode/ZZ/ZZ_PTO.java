package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp (name="ZZ_PTO", group="Test")
//@Disabled
public class ZZ_PTO extends LinearOpMode {

    ZZ_Robot_2025 robot;
    ZZ_ButtonMgr buttonMgr;

    char[] binding;
    char[] binderKeys;
    boolean[] reverse;
    boolean[] live;
    boolean[] encoder;
    boolean[] brake;
    double[] newPow;
    double[] oldPow;
    int numMotors;
    int numServos;

    int selected;

    long[] motorTimeout;
    final long timeout = 30000; //30000
    boolean[] paused;

    final double servoDisengage = 0.5;
    final double servoEngage = 0.410;
    boolean servoEnabled = false;
    double servoPos = -1;

    static int motor1 = 0;
    static int motor2 = 1;
    static int servo1 = 0;
    static boolean engaged = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new ZZ_Robot_2025(this);
        buttonMgr = new ZZ_ButtonMgr(this);

        // Wait for the opMode to be "started" and allow configuration changes
        while (!isStarted()) {
            buttonMgr.updateAll();
            telemetry.addLine("===========  ZZ PTO Tester (init) ============");
            telemetry.addLine();
            telemetry.addLine("Press X for Single Hub, Y for Dual Hubs");
            telemetry.addLine();
            telemetry.addLine("Current Selection: " + (robot.zz_dualHub ? "Dual Hubs" : "Single Hub"));
            telemetry.update();
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_dualHub = false;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_dualHub = true;
            }
            sleep(10);
        }

        // Set up the robot and related variables; this is done after init so changes can be made.
        if (robot.zz_dualHub) {
            robot.motorNames = new String[]{
                    "motor0", "motor1", "motor2", "motor3",
                    "motor0B", "motor1B", "motor2B", "motor3B"
            };
            robot.servoNames = new String[] {
                    "servo0", "servo1", "servo2", "servo3", "servo4", "servo5",
                    "servo0B", "servo1B", "servo2B", "servo3B", "servo4B", "servo5B"
            };
        } else {
            robot.motorNames = new String[]{
                    "motor0", "motor1", "motor2", "motor3"
            };
            robot.servoNames = new String[] {
                    "servo0", "servo1", "servo2", "servo3", "servo4", "servo5"
            };
        }
        robot.digitalNames = new String[]{};
        robot.analogNames = new String[]{};

        robot.initialize();

        numMotors = robot.motorNames.length;
        binding = new char[numMotors];
        reverse = new boolean[numMotors];
        live = new boolean[numMotors];
        encoder = new boolean[numMotors];
        brake = new boolean[numMotors];
        newPow = new double[numMotors];
        oldPow = new double[numMotors];
        motorTimeout = new long[numMotors];
        paused = new boolean[numMotors];

        numServos = robot.servoNames.length;

        for (int i = 0; i < numMotors; i++) {
            binding[i] = 0;
            reverse[i] = false;
            live[i] = false;
            encoder[i] = true;
            brake[i] = true;
            newPow[i] = 0;
            oldPow[i] = 0;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorTimeout[i] = System.currentTimeMillis() + timeout;
        }

        binderKeys = new char[]{'a', 'b', 'x', 'y'};

        // Second level setup
        while (opModeIsActive()) {
            buttonMgr.updateAll();
            telemetry.addLine("===========  ZZ PTO Tester (setup) ============");
            telemetry.addLine();
            telemetry.addLine("Press X to pick motor 1");
            telemetry.addLine("Press Y to pick motor 2");
            telemetry.addLine("Press B to pick servo");
            telemetry.addLine();
            telemetry.addLine("Current Selections: " + (robot.zz_dualHub ? "Dual Hubs" : "Single Hub"));
            telemetry.addLine("  Motor 1 = " + robot.motorNames[motor1]);
            telemetry.addLine("  Motor 2 = " + robot.motorNames[motor2]);
            telemetry.addLine("  Servo    = " + robot.servoNames[servo1]);
            telemetry.addLine();
            telemetry.addLine("Press Start to accept and continue");
            telemetry.update();
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.wasPressed)) {
                motor1++;
                if (motor1==motor2) motor1++;
                if (motor1 > numMotors - 1) motor1 = 0;
                if (motor1==motor2) motor1++;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                motor2++;
                if (motor2==motor1) motor2++;
                if (motor2 > numMotors - 1) motor2 = 0;
                if (motor2==motor1) motor2++;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.b, ZZ_ButtonMgr.State.wasPressed)) {
                servo1++;
                if (servo1 > numServos - 1) servo1 = 0;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
                break;
            }
            sleep(10);
        }

        live[motor1] = true;
        live[motor2] = true;

        int selections = 3;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            // engage or disengage the servo
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.a, ZZ_ButtonMgr.State.wasPressed)) {
                engaged = true;
                robot.servoArray[servo1].setPosition(servoEngage);
                servoPos = servoEngage;
                servoEnabled = true;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.b, ZZ_ButtonMgr.State.wasPressed)) {
                engaged = false;
                robot.servoArray[servo1].setPosition(servoDisengage);
                servoPos = servoDisengage;
                servoEnabled = true;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                // engaged will be left as is
                servoEnabled = false;
                ((ServoImplEx) robot.servoArray[servo1]).setPwmDisable();
            }

            // move the active selection up and down
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.isRepeating)) {
                selected--;
                if (selected < 0) selected = selections - 1;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_down, ZZ_ButtonMgr.State.isRepeating)) {
                selected++;
                if (selected > selections - 1) selected = 0;
            }

            int selMotor = 0;
            if (selected == 0) selMotor = motor1;
            if (selected == 1) selMotor = motor2;

            if (selected == 0 || selected == 1) {  // a motor is selected

                // set selected motor forward/reverse (left)
                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_left, ZZ_ButtonMgr.State.wasPressed)) {
                    updateTimeout(selMotor);
                    reverse[selMotor] = !reverse[selMotor];
                    robot.motorArray[selMotor].setDirection(reverse[selMotor] ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
                    if (live[selMotor])
                        oldPow[selMotor] += 0.000001;    // to force a direction change if running
                }

                // set selected motor to live or not (right)
                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_right, ZZ_ButtonMgr.State.wasPressed)) {
                    if (paused[selMotor]) {
                        updateTimeout(selMotor);
                    } else {
                        updateTimeout(selMotor);
                        live[selMotor] = !live[selMotor];
                        if (live[selMotor])
                            oldPow[selMotor] += 0.000001;    // for the initial go live
                    }
                }

                // set selected motor to brake or not (left bumper)
                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.left_bumper, ZZ_ButtonMgr.State.wasPressed)) {
                    updateTimeout(selMotor);
                    brake[selMotor] = !brake[selMotor];
                    if (brake[selMotor]) {
                        robot.motorArray[selMotor].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                    } else {
                        robot.motorArray[selMotor].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                    }
                }

                // set selected motor to use encoder or not (right bumper)
                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.right_bumper, ZZ_ButtonMgr.State.wasPressed)) {
                    updateTimeout(selMotor);
                    encoder[selMotor] = !encoder[selMotor];
                    if (encoder[selMotor]) {
                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    } else {
                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }

                // reset the encoder for the selected motor (start)
                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
                    updateTimeout(selMotor);
                    robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    if (encoder[selMotor]) {
                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    } else {
                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
            }

            // stop all motors (back)
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.back, ZZ_ButtonMgr.State.wasPressed)) {
                for (int i = 0; i < numMotors; i++) {
                    robot.motorArray[i].setPower(0);
                    oldPow[i] = 0;
                    newPow[i] = 0;
                }
            }

            // decide if power is driven by left or right stick
            if (engaged) {
                newPow[motor1] = -gamepad1.left_stick_y;
                newPow[motor2] = -gamepad1.left_stick_y;
            }
            else {
                newPow[motor1] = -gamepad1.left_stick_y;
                newPow[motor2] = -gamepad1.right_stick_y;
            }

            // update the motor power if live and the position has changed
            for (int i = 0; i < numMotors; i++) {
                if (live[i] && newPow[i] != oldPow[i]) {
                    updateTimeout(i);
                    robot.motorArray[i].setPower(newPow[i]);
                    oldPow[i] = newPow[i];
                }
            }

            // timeout the motor if not interacted with (trying to avoid lightly stalled motors)
            for (int i = 0; i < numMotors; i++) {
                if (motorTimeout[i] < System.currentTimeMillis()) {
                    if (robot.motorArray[i].getPower() != 0) {
                        paused[i] = true;
                        robot.motorArray[i].setPower(0);
                        // leaving oldPow[i] alone intentionally
                    }
                }
            }

            telemetry.addLine("==============  ZZ PTO Tester  ==============");
            telemetry.addLine();
            telemetry.addLine("up/down to select actuator");
            telemetry.addLine();
            telemetry.addLine("Motors:");
            telemetry.addLine("  left for forward/reverse |  right for live/not");
            telemetry.addLine("  l_bumper for brake/not  |  r_bumper for encoder/not");
            telemetry.addLine("  back for stop all              |  start for reset encoder");
            telemetry.addLine();
            telemetry.addLine("Servo:");
            telemetry.addLine("  a for engage  |  b for disengage  |  y for disable");
            telemetry.addLine();
            telemetry.addLine("When PTO is disengaged:");
            telemetry.addLine("  LEFT STICK for motor1   | RIGHT STICK for motor2");
            telemetry.addLine("When PTO is engaged:");
            telemetry.addLine("  LEFT STICK for both motors");
            telemetry.addLine();
            telemetry.addLine("[sel] [name] [live][encoder][brake][dir] [current] [new] [encoder] [vel]");
            telemetry.addLine();

            // Build telemetry strings
            //motors
            for (int j = 0; j < 2; j++) {
                int i = (j==0) ? motor1 : motor2;
                String telString;
                telString = (j == selected) ? "=> " : "     ";
                telString += (robot.motorNames[i] + "               ").substring(0, 10);
                telString += (paused[i] ? "P" : (live[i] ? "L" : "_")); // + " ";
                telString += (encoder[i] ? "E" : "_"); // + " ";
                telString += (brake[i] ? "B" : "_"); // + " ";
                telString += (reverse[i] ? "R" : "F") + " ";
                telString += String.format("%.2f", oldPow[i]) + " ";
                telString += String.format("%.2f", newPow[i]) + " ";
                telString += (String.format("%07d", robot.motorArray[i].getCurrentPosition()) + "     ").substring(0, 8);
                telString += String.format("%.2f", robot.motorArray[i].getVelocity());
                telString += (i == selected) ? " <=" : "     ";
                telemetry.addLine(telString);
            }
            //servo
            int i = servo1;
            String telString;
            telString = (i == -1) ? "=>  " : "      ";
            telString += (robot.servoNames[i] + "               ").substring(0, 10);
            telString += (servoEnabled ? "E" : "_") + "    ";
            telString += String.format("%.3f", servoPos);
            telString += (engaged ? "  engaged" : "  disengaged");
            telemetry.addLine(telString);

            telemetry.update();
        }
    }

    void updateTimeout(int motor) {
        motorTimeout[motor] = System.currentTimeMillis() + timeout;
        if (paused[motor]) {
            paused[motor] = false;
            robot.motorArray[motor].setPower(oldPow[motor]);
        }
    }
}