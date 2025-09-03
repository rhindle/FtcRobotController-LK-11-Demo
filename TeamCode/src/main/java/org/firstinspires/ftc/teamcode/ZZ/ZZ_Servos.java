package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.RobotV2;


@TeleOp (name="ZZ_Servos", group="Test")
//@Disabled
public class ZZ_Servos extends LinearOpMode {

    RobotV2 robot;
    ButtonMgr buttonMgr;

    char[] binding;
    char[] binderKeys;
    boolean[] reverse;
    boolean[] live;
    boolean[] enabled;
    double[] newPos;
    static double[] oldPos;
    int numServos;

    int active;

    final double smallChange = .0001;
    final double largeChange = .005;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new RobotV2(this);
        buttonMgr = new ButtonMgr(this);
        robot.init();

        numServos = robot.servoNames.length;
        binding = new char[numServos];
        reverse = new boolean[numServos];
        live = new boolean[numServos];
        enabled = new boolean[numServos];
        newPos = new double[numServos];
        if (oldPos == null || oldPos.length != numServos) {
            oldPos = new double[numServos];
            for (int i = 0; i < numServos; i++) {
                oldPos[i] = 0.5;
            }
        }
        for (int i = 0; i < numServos; i++) {
            binding[i] = 0;
            reverse[i] = false;
            live[i] = false;
            enabled[i] = false;
            newPos[i] = oldPos[i];
        }

        binderKeys = new char[] {'a', 'b', 'x', 'y'};

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.update();
            sleep(100);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();

            // move the active selection up and down
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasPressed)) {
                active--;
                if (active < 0) active = numServos - 1;
            }
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasPressed)) {
                active++;
                if (active > numServos - 1) active = 0;
            }

            // set selected servo forward/reverse (left)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_left, ButtonMgr.State.wasPressed)) {
                reverse[active] = !reverse[active];
                robot.servoArray[active].setDirection(reverse[active] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            }

            // set selected servo to live or not (right)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.dpad_right, ButtonMgr.State.wasPressed)) {
                live[active] = !live[active];
                if (live[active]) oldPos[active] += 0.000001;    // for the initial go live
            }

            // disable the selected servo (back)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.back, ButtonMgr.State.wasPressed)) {
                enabled[active] = false;
                ((ServoImplEx)robot.servoArray[active]).setPwmDisable();
            }

            // neutral-ize the selected servo (start)
            if (buttonMgr.getState(1, ButtonMgr.Buttons.start, ButtonMgr.State.wasPressed)) {
                newPos[active] = 0.5;
            }

            // add, change, or remove key bindings
            for (char binderKey : binderKeys) {
                if (buttonMgr.getState(1, String.valueOf(binderKey), ButtonMgr.State.wasDoubleTapped)) {
                    if (binding[active] != binderKey) binding[active] = binderKey;
                    else binding[active] = 0;
                }
            }

            // modify the new position by left and right stick for all bound servos
            boolean anyChange = false;
            for (int i = 0; i < numServos; i++) {
                if (binding[i] != 0 && buttonMgr.getState(1, String.valueOf(binding[i]), ButtonMgr.State.isPressed)) {
                    newPos[i] += gamepad1.left_stick_y * -largeChange;
                    newPos[i] += gamepad1.right_stick_y * -smallChange;
                    newPos[i] = Math.max(0, Math.min(1, newPos[i]));
                    anyChange = true;
                }
            }

            // modify the selected servo if none of the bound buttons were pressed
            if (!anyChange) {
                newPos[active] += gamepad1.left_stick_y * -largeChange;
                newPos[active] += gamepad1.right_stick_y * -smallChange;
                newPos[active] = Math.max(0, Math.min(1, newPos[active]));
            }

            // update the servo position if live and the position has changed
            for (int i = 0; i < numServos; i++) {
                if (live[i] && newPos[i] != oldPos[i]) {
                    robot.servoArray[i].setPosition(newPos[i]);
                    oldPos[i] = newPos[i];
                    enabled[i] = true;
                }
            }

            telemetry.addLine("==============  ZZ Servo Tester  ==============");
            telemetry.addLine();
            telemetry.addLine("up/down to select servo   |  doubletap a/b/x/y to bind");
            telemetry.addLine("left for forward/reverse    |  right for live/not");
            telemetry.addLine("back for disable                 |  start for neutral (0.5)");
            telemetry.addLine();
            telemetry.addLine("hold a/b/x/y to change position for bound servos");
            telemetry.addLine("otherwise active servo only will change");
            telemetry.addLine("  left stick for large changes");
            telemetry.addLine("  right stick for small changes");
            telemetry.addLine();
            telemetry.addLine("[sel] [name] [bind] [live][enabled][dir] [current] [new]");
            telemetry.addLine();

            // Build telemetry strings
            for (int i = 0; i < numServos; i++) {
                String telString;
                telString = (i == active) ? "=>  " : "      ";
                telString += (robot.servoNames[i] + "               ").substring(0, 10);
                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + "    ";
                telString += (live[i] ? "L" : "_"); // + " ";
                telString += (enabled[i] ? "E" : "_"); // + " ";
                telString += (reverse[i] ? "R" : "F") + "    ";
                telString += String.format("%.3f", oldPos[i]) + "    ";
                telString += String.format("%.3f", newPos[i]);
                telemetry.addLine(telString);
            }

            telemetry.update();

        }
    }
}