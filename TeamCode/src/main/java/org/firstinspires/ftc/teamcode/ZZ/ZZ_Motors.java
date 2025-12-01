package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name="ZZ_Motors", group="Test")
//@Disabled
public class ZZ_Motors extends LinearOpMode {

    ZZ_Robot_2025 robot;
    ZZ_ButtonMgr buttonMgr;

    char[] binding;
    char[] binderKeys;
    boolean[] reverse;
    boolean[] live;
    boolean[] encoder;
    boolean[] brake;
    boolean absolute = false;
    double[] newPow;
    //    static double[] oldPow;
    double[] oldPow;
    int numMotors;

    int active;

    long[] motorTimeout;
    final long timeout = 30000; //30000
    boolean[] paused;

    final double smallChange = .0001;
    final double largeChange = .005;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new ZZ_Robot_2025(this);
        buttonMgr = new ZZ_ButtonMgr(this);

        // Wait for the opMode to be "started" and allow configuration changes
        while (!isStarted()) {
            buttonMgr.updateAll();
            telemetry.addLine("===========  ZZ Motor Tester (init) ============");
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
        } else {
            robot.motorNames = new String[]{
                    "motor0", "motor1", "motor2", "motor3"
            };
        }
        robot.servoNames = new String[]{};
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


//        if (oldPow == null || oldPow.length != numMotors) {
//            oldPow = new double[numMotors];
//            for (int i = 0; i < numMotors; i++) {
//                oldPow[i] = 0;
//            }
//        }
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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            // move the active selection up and down
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.isRepeating)) {
                active--;
                if (active < 0) active = numMotors - 1;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_down, ZZ_ButtonMgr.State.isRepeating)) {
                active++;
                if (active > numMotors - 1) active = 0;
            }

            // set selected motor forward/reverse (left)
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_left, ZZ_ButtonMgr.State.wasPressed)) {
                updateTimeout(active);
                reverse[active] = !reverse[active];
                robot.motorArray[active].setDirection(reverse[active] ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
                if (live[active])
                    oldPow[active] += 0.000001;    // to force a direction change if running
            }

            // set selected motor to live or not (right)
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_right, ZZ_ButtonMgr.State.wasPressed)) {
                if (paused[active]) {
                    updateTimeout(active);
                }
                else {
                    updateTimeout(active);
                    live[active] = !live[active];
                    if (live[active]) oldPow[active] += 0.000001;    // for the initial go live
                }
            }

            // set selected motor to brake or not (left bumper)
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.left_bumper, ZZ_ButtonMgr.State.wasPressed)) {
                updateTimeout(active);
                brake[active] = !brake[active];
                if (brake[active]) {
                    robot.motorArray[active].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                } else {
                    robot.motorArray[active].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                }
            }

            // set selected motor to use encoder or not (right bumper)
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.right_bumper, ZZ_ButtonMgr.State.wasPressed)) {
                updateTimeout(active);
                encoder[active] = !encoder[active];
                if (encoder[active]) {
                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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

            // reset the encoder for the selected motor (start)
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
                updateTimeout(active);
                robot.motorArray[active].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                if (encoder[active]) {
                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                } else {
                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            // add, change, or remove key bindings
            for (char binderKey : binderKeys) {
                if (buttonMgr.getState(1, String.valueOf(binderKey), ZZ_ButtonMgr.State.wasDoubleTapped)) {
                    if (binding[active] != binderKey) binding[active] = binderKey;
                    else binding[active] = 0;
                }
            }

            // decide if power is driven by left or right stick
            if (gamepad1.right_stick_y == 0 && gamepad1.left_stick_y != 0) {
                absolute = true;
            }
            if (gamepad1.right_stick_y != 0) {
                absolute = false;
            }

            // modify the new position by left and right stick for all bound motors
            boolean anyChange = false;
            for (int i = 0; i < numMotors; i++) {
                if (binding[i] != 0 && buttonMgr.getState(1, String.valueOf(binding[i]), ZZ_ButtonMgr.State.isPressed)) {
                    if (absolute) {
                        newPow[i] = -gamepad1.left_stick_y;
                    } else {
                        newPow[i] += gamepad1.right_stick_y * -largeChange;
                    }
                    newPow[i] = Math.max(-1, Math.min(1, newPow[i]));
                    anyChange = true;
                }
            }

            // modify the selected motor if none of the bound buttons were pressed
            if (!anyChange) {
                if (absolute) {
                    newPow[active] = -gamepad1.left_stick_y;
                } else {
                    newPow[active] += gamepad1.right_stick_y * -largeChange;
                }
                newPow[active] = Math.max(-1, Math.min(1, newPow[active]));
            }

            // update the motor power if live and the position has changed
            for (int i = 0; i < numMotors; i++) {
                if (live[i] && newPow[i] != oldPow[i]) {
                    //updateTimeout(active);  I think this is wrong 20251130?
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


            telemetry.addLine("==============  ZZ Motor Tester  ==============");
            telemetry.addLine();
            telemetry.addLine("up/down to select motor |  doubletap a/b/x/y to bind");
            telemetry.addLine("left for forward/reverse   |  right for live/not");
            telemetry.addLine("l_bumper for brake/not    |  r_bumper for encoder/not");
            telemetry.addLine("back for stop all                |  start for reset encoder");
            telemetry.addLine();
            telemetry.addLine("hold a/b/x/y to change power for bound motors");
            telemetry.addLine("otherwise active motor only will change");
            telemetry.addLine("  left stick for live absolute changes");
            telemetry.addLine("  right stick for relative changes");
            telemetry.addLine();
            telemetry.addLine("[sel] [name] [bind] [live][encoder][brake][dir] [current] [new] [encoder] [vel]");
            telemetry.addLine();

            // Build telemetry strings
            for (int i = 0; i < numMotors; i++) {
                String telString;
                telString = (i == active) ? "=> " : "     ";
                telString += (robot.motorNames[i] + "               ").substring(0, 10);
                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + " ";
                telString += (paused[i] ? "P" : (live[i] ? "L" : "_")); // + " ";
                //telString += (live[i] ? "L" : "_"); // + " ";
                telString += (encoder[i] ? "E" : "_"); // + " ";
                telString += (brake[i] ? "B" : "_"); // + " ";
                telString += (reverse[i] ? "R" : "F") + " ";
                telString += String.format("%.2f", oldPow[i]) + " ";
                telString += String.format("%.2f", newPow[i]) + " ";
                telString += (String.format("%07d", robot.motorArray[i].getCurrentPosition()) + "     ").substring(0, 8);
                telString += String.format("%.2f", robot.motorArray[i].getVelocity());
                telString += (i == active) ? " <=" : "     ";
                telemetry.addLine(telString);
            }

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