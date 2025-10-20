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
    boolean[] brake;
    boolean absolute = false;
    double[] newVel;
    double[] oldVel;
    double[] spinMultiplier;
    double[] maxRPM;
    double[] newRPM;
    int numMotors;
    static double[] tickOptions = {28.0, 103.8, 145.1, 384.5, 537.7, 751.8, 1425.1, 1993.6, 2786.2, 3895.9, 5281.1};
    static int[] rpmOptions = {6000, 1620, 1150, 435, 312, 223, 117, 84, 60, 43, 30};
    static double[] ticksPerRev;
    static double defaultTicks = tickOptions[0];
    int tickChoice = 0;
    boolean changeTicks = false;

    int active;

    final double smallChange = .0001;
    final double largeChange = .005;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new RobotV2(this);
        buttonMgr = new ButtonMgr(this);
        boolean dualHub = true;

        tickChoice = findTickIndex(defaultTicks);

        // Wait for the opMode to be "started" and allow configuration changes
        while (!isStarted()) {
            buttonMgr.updateAll();
            telemetry.addLine("===========  ZZ Motor Spinner (init) ============");
            telemetry.addLine();
            telemetry.addLine("Press X for Single Hub, Y for Dual Hubs");
            telemetry.addLine("Press triggers to cycle motor types");
            telemetry.addLine();
            telemetry.addLine("Current Selection: " + (dualHub ? "Dual Hubs" : "Single Hub"));
            telemetry.addLine("Default Ticks: " + defaultTicks + " (" +
                    rpmOptions[tickChoice] + " RPM)" + (changeTicks ? "" : " (no change)"));
            telemetry.update();
            if (buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasPressed)) {
                dualHub = false;
            }
            if (buttonMgr.getState(1, ButtonMgr.Buttons.y, ButtonMgr.State.wasPressed)) {
                dualHub = true;
            }
            if (buttonMgr.getState(1, ButtonMgr.Buttons.right_trigger, ButtonMgr.State.isRepeating)) {
                changeTicks = true;
                tickChoice++;
                if (tickChoice >= tickOptions.length) tickChoice = 0;
                defaultTicks = tickOptions[tickChoice];
            }
            if (buttonMgr.getState(1, ButtonMgr.Buttons.left_trigger, ButtonMgr.State.isRepeating)) {
                changeTicks = true;
                tickChoice--;
                if (tickChoice < 0) tickChoice = tickOptions.length - 1;
                defaultTicks = tickOptions[tickChoice];
            }
            sleep(10);
        }

        // Set up the robot and related variables; this is done after init so changes can be made.
        if (dualHub) {
            robot.motorNames = new String []  {
                    "motor0", "motor1", "motor2", "motor3",
                    "motor0B", "motor1B", "motor2B", "motor3B"
            };
        } else {
            robot.motorNames = new String []  {
                    "motor0", "motor1", "motor2", "motor3"
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
        brake = new boolean[numMotors];
        newVel = new double[numMotors];
        oldVel = new double[numMotors];
        newRPM = new double[numMotors];
        spinMultiplier = new double[numMotors];
        maxRPM = new double[numMotors];

        if (ticksPerRev == null || ticksPerRev.length != numMotors || changeTicks) {
            ticksPerRev = new double[numMotors];
            for (int i = 0; i < numMotors; i++) {
                ticksPerRev[i] = defaultTicks;
            }
        }

        for (int i = 0; i < numMotors; i++) {
            binding[i] = 0;
            reverse[i] = false;
            live[i] = false;
            brake[i] = false;
            newVel[i] = 0;
            oldVel[i] = 0;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            updateTickSettings(i, ticksPerRev[i]);
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

            // stop all motors (back)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.back, ButtonMgr.State.wasPressed)) {
                for (int i = 0; i < numMotors; i++) {
                    robot.motorArray[i].setPower(0);
                    oldVel[i] = 0;
                    newVel[i] = 0;
                    newRPM[i] = 0;
                }
            }

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
                    if (buttonMgr.getState(1, ButtonMgr.Buttons.right_trigger, ButtonMgr.State.isRepeating)) {
                        int index = findTickIndex(ticksPerRev[i]);
                        if (++index >= tickOptions.length) index = 0;
                        updateTickSettings(i, tickOptions[index]);
                    }
                    if (buttonMgr.getState(1, ButtonMgr.Buttons.left_trigger, ButtonMgr.State.isRepeating)) {
                        int index = findTickIndex(ticksPerRev[i]);
                        if (--index < 0) index = tickOptions.length - 1;
                        updateTickSettings(i, tickOptions[index]);
                    }
                }
            }

            // modify the selected motor if none of the bound buttons were pressed
            if (!anyChange) {
                newRPM[active] += gamepad1.left_stick_y * -largeChange * maxRPM[active];
                newRPM[active] += gamepad1.right_stick_y * -smallChange * maxRPM[active];
                newRPM[active] = Math.max(-maxRPM[active], Math.min(maxRPM[active], newRPM[active]));
                newVel[active] = newRPM[active] / spinMultiplier[active];
                if (buttonMgr.getState(1, ButtonMgr.Buttons.right_trigger, ButtonMgr.State.isRepeating)) {
                    int index = findTickIndex(ticksPerRev[active]);
                    if (++index >= tickOptions.length) index = 0;
                    updateTickSettings(active, tickOptions[index]);
                }
                if (buttonMgr.getState(1, ButtonMgr.Buttons.left_trigger, ButtonMgr.State.isRepeating)) {
                    int index = findTickIndex(ticksPerRev[active]);
                    if (--index < 0) index = tickOptions.length - 1;
                    updateTickSettings(active, tickOptions[index]);
                }
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
            telemetry.addLine("triggers to cycle motor types");
            telemetry.addLine();
            telemetry.addLine("hold a/b/x/y to change power for bound motors");
            telemetry.addLine("otherwise active motor only will change");
            telemetry.addLine("  left stick for large changes");
            telemetry.addLine("  right stick for small changes");
            telemetry.addLine();
            telemetry.addLine("[sel] [name] [bind] [live][brake][dir] (spec) [setrpm] [rpm]");
            telemetry.addLine();

            // Build telemetry strings
            for (int i = 0; i < numMotors; i++) {
                String telString;
                telString = (i == active) ? "=> " : "     ";
                telString += (robot.motorNames[i] + "               ").substring(0, 10);
                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + " ";
                telString += (live[i] ? "L" : "_"); // + " ";
                telString += (brake[i] ? "B" : "_"); // + " ";
                telString += (reverse[i] ? "R" : "F") + "   ";
                telString += "(" + String.format("%04d", rpmOptions[findTickIndex(ticksPerRev[i])]) + ")   ";
                telString += String.format("%05d", (int)newRPM[i]) + "   ";
                telString += String.format("%05d", (int)(robot.motorArray[i].getVelocity() * spinMultiplier[i]));
                telString += (i == active) ? " <=" : "     ";
                telemetry.addLine(telString);
            }

            telemetry.update();

        }
    }

    public int findTickIndex(double ticks) {
        for (int i = 0; i < tickOptions.length; i++) {
            if (ticks == tickOptions[i]) return i;
        }
        return 0;
    }

    public void updateTickSettings(int motorNum, double ticks) {
        ticksPerRev[motorNum] = ticks;
        spinMultiplier[motorNum] = 60.0 / ticks;
        maxRPM[motorNum] = 168200 / ticks;
    }

}