package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.RobotV2;

@TeleOp (name="ZZ_MotorSpin", group="Test")
//@Disabled
public class ZZ_MotorSpin extends LinearOpMode {

    RobotV2 robot;
    ButtonMgr buttonMgr;

    char[] binding;
    char[] binderKeys;
    boolean[] reverse;
    boolean[] live;
//    boolean[] encoder;
    boolean[] brake;
    boolean absolute = false;
    double[] newVel;
//    static double[] oldPow;
    double[] oldVel;
    double[] ticksPerRev;
    double[] spinMultiplier;
    double[] maxRPM;
    double[] newRPM;
    int numMotors;

    int active;

    final double smallChange = .0001;
    final double largeChange = .005;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new RobotV2(this);
        buttonMgr = new ButtonMgr(this);
        boolean dualHub = true;

        // Wait for the opMode to be "started" and allow configuration changes
        while (!isStarted()) {
            buttonMgr.updateAll();
            telemetry.addLine("===========  ZZ Motor Spinner (init) ============");
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
            ticksPerRev = new double[] {
                    28, 28, 28, 28,
                    28, 28, 28, 28
            };
        } else {
            robot.motorNames = new String []  {
                    "motor0", "motor1", "motor2", "motor3"
            };
            ticksPerRev = new double[] {
                    28, 28, 28, 28
            };
        }
        robot.servoNames = new String[] { };
        robot.digitalNames = new String[] { };
        robot.analogNames = new String[] { };

        robot.initialize();

        numMotors = robot.motorNames.length;
        binding = new char[numMotors];
        reverse = new boolean[numMotors];
        live = new boolean[numMotors];
//        encoder = new boolean[numMotors];
        brake = new boolean[numMotors];
        newVel = new double[numMotors];
        oldVel = new double[numMotors];
        newRPM = new double[numMotors];
        spinMultiplier = new double[numMotors];
        maxRPM = new double[numMotors];

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
//            encoder[i] = true;
            brake[i] = false;
            newVel[i] = 0;
            oldVel[i] = 0;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            spinMultiplier[i] = 60.0 / ticksPerRev[i];
            maxRPM[i] = 170000.0 / ticksPerRev[i];
        }

        binderKeys = new char[] {'a', 'b', 'x', 'y'};

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            // move the active selection up and down
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.isRepeating)) {
                active--;
                if (active < 0) active = numMotors - 1;
            }
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.isRepeating)) {
                active++;
                if (active > numMotors - 1) active = 0;
            }

            // set selected motor forward/reverse (left)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_left, ButtonMgr.State.wasPressed)) {
                reverse[active] = !reverse[active];
                robot.motorArray[active].setDirection(reverse[active] ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
                if (live[active]) oldVel[active] += 0.000001;    // to force a direction change if running
            }

            // set selected motor to live or not (right)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_right, ButtonMgr.State.wasPressed)) {
                live[active] = !live[active];
                if (live[active]) oldVel[active] += 0.000001;    // for the initial go live
            }

            // set selected motor to brake or not (left bumper)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.left_bumper, ButtonMgr.State.wasPressed)) {
                brake[active] = !brake[active];
                if (brake[active]) {
                    robot.motorArray[active].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                } else {
                    robot.motorArray[active].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                }
            }

//            // set selected motor to use encoder or not (right bumper)
//            if (buttonMgr.getState(1, ButtonMgr.Buttons.right_bumper, ButtonMgr.State.wasPressed)) {
//                encoder[active] = !encoder[active];
//                if (encoder[active]) {
//                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                } else {
//                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                }
//            }

            // stop all motors (back)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.back, ButtonMgr.State.wasPressed)) {
                for (int i = 0; i < numMotors; i++) {
                    robot.motorArray[i].setPower(0);
                    oldVel[i] = 0;
                    newVel[i] = 0;
                    newRPM[i] = 0;
                }
            }

//            // reset the encoder for the selected motor (start)
//            if (buttonMgr.getState(1, ButtonMgr.Buttons.start, ButtonMgr.State.wasPressed)) {
//                robot.motorArray[active].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                if (encoder[active]) {
//                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                } else {
//                    robot.motorArray[active].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                }
//            }

            // add, change, or remove key bindings
            for (char binderKey : binderKeys) {
                if (buttonMgr.getState(1, String.valueOf(binderKey), ButtonMgr.State.wasDoubleTapped)) {
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
                if (binding[i] != 0 && buttonMgr.getState(1, String.valueOf(binding[i]), ButtonMgr.State.isPressed)) {
                    newRPM[i] += gamepad1.left_stick_y * -largeChange * maxRPM[i];
                    newRPM[i] += gamepad1.right_stick_y * -smallChange * maxRPM[i];
                    newRPM[i] = Math.max(-maxRPM[i], Math.min(maxRPM[i], newRPM[i]));
                    newVel[i] = newRPM[i] / spinMultiplier[i];
                    anyChange = true;
                }
            }

            // modify the selected motor if none of the bound buttons were pressed
            if (!anyChange) {
                newRPM[active] += gamepad1.left_stick_y * -largeChange * maxRPM[active];
                newRPM[active] += gamepad1.right_stick_y * -smallChange * maxRPM[active];
                newRPM[active] = Math.max(-maxRPM[active], Math.min(maxRPM[active], newRPM[active]));
                newVel[active] = newRPM[active] / spinMultiplier[active];
            }

            // update the motor velocity if live and the position has changed
            for (int i = 0; i < numMotors; i++) {
                if (live[i] && newVel[i] != oldVel[i]) {
                    robot.motorArray[i].setVelocity(newVel[i]);
                    oldVel[i] = newVel[i];
                }
            }

            telemetry.addLine("==============  ZZ Motor Spinner  ==============");
            telemetry.addLine();
            telemetry.addLine("up/down to select motor |  doubletap a/b/x/y to bind");
            telemetry.addLine("left for forward/reverse   |  right for live/not");
            telemetry.addLine("l_bumper for brake/not    |  ");
            telemetry.addLine("back for stop all                |  ");
            telemetry.addLine();
            telemetry.addLine("hold a/b/x/y to change power for bound motors");
            telemetry.addLine("otherwise active motor only will change");
            telemetry.addLine("  left stick for large changes");
            telemetry.addLine("  right stick for small changes");
            telemetry.addLine();
            telemetry.addLine("[sel] [name] [bind] [live][brake][dir] [setrpm] [rpm]");
            telemetry.addLine();

            // Build telemetry strings
            for (int i = 0; i < numMotors; i++) {
                String telString;
                telString = (i == active) ? "=> " : "     ";
                telString += (robot.motorNames[i] + "               ").substring(0, 10);
                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + " ";
                telString += (live[i] ? "L" : "_"); // + " ";
//                telString += (encoder[i] ? "E" : "_"); // + " ";
                telString += (brake[i] ? "B" : "_"); // + " ";
                telString += (reverse[i] ? "R" : "F") + "   ";
//                telString += String.format("%.2f", oldVel[i]) + " ";
//                telString += String.format("%.2f", newVel[i]) + " ";
//                telString += (String.format("%07d", robot.motorArray[i].getCurrentPosition()) + "     ").substring(0, 8);
                telString += String.format("%05d", (int)newRPM[i]) + "   ";
                telString += String.format("%05d", (int)(robot.motorArray[i].getVelocity() * spinMultiplier[i]));
                //telString += String.format("%.2f", robot.motorArray[i].getVelocity() * spinMultiplier[i]);
                telString += (i == active) ? " <=" : "     ";
                telemetry.addLine(telString);
            }

            telemetry.update();

        }
    }
}