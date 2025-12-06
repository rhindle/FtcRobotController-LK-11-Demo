package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp (name="ZZ_3Shooter", group="Test")
//@Disabled
public class ZZ_3Shooter extends LinearOpMode {

    ZZ_Robot_2025 robot;
    ZZ_ButtonMgr buttonMgr;

    ZZ_ServoSSR servoGreen;
    ZZ_ServoSSR servoPink;
    ZZ_ServoSSR servoBlue;
    ZZ_ServoSSR servoWhite;

    ZZ_StateMachine launchGreen;
    ZZ_StateMachine launchPink;
    ZZ_StateMachine launchBlue;
    ZZ_StateMachine launchAll;
    ZZ_StateMachine resetAll;

    static String spinMotor1Name = "motor2B";
    static String spinMotor2Name = "motor3B";
    static String launchServoGreen = "servo0B";
    static String launchServoPink = "servo1B";
    static String launchServoBlue = "servo2B";
    static String launchServoWhite = "servo3B";

    DcMotorEx spinMotor1;
    DcMotorEx spinMotor2;

    final double servoGreenDock             = 0.485;
    final double servoGreenLaunch           = 0.220;
    final double servoPinkDock              = 0.490;
    final double servoPinkLaunch            = 0.753;
    final double servoBlueDock              = 0.461;
    final double servoBlueLaunch            = 0.718;
    final double servoWhiteClearance        = 0.541;
    final double servoWhiteMax              = 0.587;
    final double servoWhiteDock             = 0.471;
    final double servoWhiteLaunch           = servoWhiteMax;

    static double motorSpinSpeed                = 3500;
    final boolean motor1reverse             = false;
    final boolean motor2reverse             = false;

    boolean[] reverse;
    boolean[] live;
    boolean[] brake;
//    boolean absolute = false;
//    double[] newVel;
//    double[] oldVel;
//    double[] spinMultiplier;
//    double[] maxRPM;
//    double[] newRPM;
    int numMotors;
    double spinTicks = 28.0;
    int spinRPM = 6000;

//    static double[] tickOptions = {28.0, 103.8, 145.1, 384.5, 537.7, 751.8, 1425.1, 1993.6, 2786.2, 3895.9, 5281.1};
//    static int[] rpmOptions = {6000, 1620, 1150, 435, 312, 223, 117, 84, 60, 43, 30};
//    static double[] ticksPerRev;
//    static double defaultTicks = tickOptions[0];
//    int tickChoice = 0;
//    boolean changeTicks = false;

    int numServos;

//    int selected;
//
//    long[] motorTimeout;
//    final long timeout = 30000; //30000
//    boolean[] paused;

//    final double servoDisengage = 0.5;
//    final double servoEngage = 0.410;
//    boolean servoEnabled = false;
//    double servoPos = -1;

//    static int motor1 = 0;
//    static int motor2 = 1;
//    static int servo1 = 0;
//    static boolean engaged = false;

    final double smallChange = .0001;
    final double largeChange = .005;

    public static PIDFCoefficients launchSpinPID = new PIDFCoefficients(100,0,0,12.4);

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        robot = new ZZ_Robot_2025(this);
        buttonMgr = new ZZ_ButtonMgr(this);
        ZZ_StateMachine.reset();

        // Wait for the opMode to be "started" and allow configuration changes
        while (!isStarted()) {
            buttonMgr.updateAll();
            telemetry.addLine("==========  ZZ Shoot Tester (init) ===========");
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
        reverse = new boolean[numMotors];
        live = new boolean[numMotors];
        brake = new boolean[numMotors];
//        newVel = new double[numMotors];
//        oldVel = new double[numMotors];
//        newRPM = new double[numMotors];
//        spinMultiplier = new double[numMotors];
//        maxRPM = new double[numMotors];

//        if (ticksPerRev == null || ticksPerRev.length != numMotors || changeTicks) {
//            ticksPerRev = new double[numMotors];
//            for (int i = 0; i < numMotors; i++) {
//                ticksPerRev[i] = defaultTicks;
//            }
//        }

        numServos = robot.servoNames.length;

        // set all motors to default
        for (int i = 0; i < numMotors; i++) {
            reverse[i] = false;
            live[i] = false;
            brake[i] = false;
//            newVel[i] = 0;
//            oldVel[i] = 0;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            updateTickSettings(i, ticksPerRev[i]);

            robot.motorArray[i].setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);
        }

        servoBlue = new ZZ_ServoSSR(robot.getServoByName(launchServoBlue));
        servoPink = new ZZ_ServoSSR(robot.getServoByName(launchServoPink));
        servoGreen = new ZZ_ServoSSR(robot.getServoByName(launchServoGreen));
        servoWhite = new ZZ_ServoSSR(robot.getServoByName(launchServoWhite));
        spinMotor1 = robot.getMotorByName(spinMotor1Name);
        spinMotor2 = robot.getMotorByName(spinMotor2Name);

        if (motor1reverse) spinMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        if (motor2reverse) spinMotor2.setDirection(DcMotorEx.Direction.REVERSE);


        buildStateMachines();
        resetAll.restart();

//        // Second level setup
//        while (opModeIsActive()) {
//            buttonMgr.updateAll();
//            telemetry.addLine("===========  ZZ PTO Tester (setup) ============");
//            telemetry.addLine();
//            telemetry.addLine("Press X to pick motor 1");
//            telemetry.addLine("Press Y to pick motor 2");
//            telemetry.addLine("Press B to pick servo");
//            telemetry.addLine();
//            telemetry.addLine("Current Selections: " + (robot.zz_dualHub ? "Dual Hubs" : "Single Hub"));
//            telemetry.addLine("  Motor 1 = " + robot.motorNames[motor1]);
//            telemetry.addLine("  Motor 2 = " + robot.motorNames[motor2]);
//            telemetry.addLine("  Servo    = " + robot.servoNames[servo1]);
//            telemetry.addLine();
//            telemetry.addLine("Press Start to accept and continue");
//            telemetry.update();
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.wasPressed)) {
//                motor1++;
//                if (motor1==motor2) motor1++;
//                if (motor1 > numMotors - 1) motor1 = 0;
//                if (motor1==motor2) motor1++;
//            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
//                motor2++;
//                if (motor2==motor1) motor2++;
//                if (motor2 > numMotors - 1) motor2 = 0;
//                if (motor2==motor1) motor2++;
//            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.b, ZZ_ButtonMgr.State.wasPressed)) {
//                servo1++;
//                if (servo1 > numServos - 1) servo1 = 0;
//            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
//                break;
//            }
//            sleep(10);
//        }
//
//        live[motor1] = true;
//        live[motor2] = true;
//
//        int selections = 3;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            ZZ_StateMachine.runLoop();

            //run the launchers
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.wasPressed)) {
                launchPink.restart();
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                launchBlue.restart();
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.b, ZZ_ButtonMgr.State.wasPressed)) {
                launchGreen.restart();
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.a, ZZ_ButtonMgr.State.wasPressed)) {
                launchAll.restart();
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
                resetAll.restart();
            }

            // run the motors
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.wasPressed)) {
                spinMotor1.setVelocity(motorSpinSpeed / (60.0 / spinTicks));
                spinMotor2.setVelocity(motorSpinSpeed / (60.0 / spinTicks));
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_down, ZZ_ButtonMgr.State.wasPressed)) {
                spinMotor1.setPower(0);
                spinMotor2.setPower(0);
            }

            // stop all
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.back, ZZ_ButtonMgr.State.wasPressed)) {
                spinMotor1.setPower(0);
                spinMotor2.setPower(0);
                servoBlue.disable();
                servoPink.disable();
                servoGreen.disable();
                servoWhite.disable();
            }

            // modify spin speed
            motorSpinSpeed += gamepad1.left_stick_y * -largeChange * spinRPM;
            motorSpinSpeed += gamepad1.right_stick_y * -smallChange * spinRPM;
            motorSpinSpeed = Math.max(-spinRPM, Math.min(spinRPM, motorSpinSpeed));
            // rely on user to use the up button?  or flag and set here


//            // engage or disengage the servo
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.a, ZZ_ButtonMgr.State.wasPressed)) {
//                engaged = true;
//                robot.servoArray[servo1].setPosition(servoEngage);
//                servoPos = servoEngage;
//                servoEnabled = true;
//            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.b, ZZ_ButtonMgr.State.wasPressed)) {
//                engaged = false;
//                robot.servoArray[servo1].setPosition(servoDisengage);
//                servoPos = servoDisengage;
//                servoEnabled = true;
//            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
//                // engaged will be left as is
//                servoEnabled = false;
//                ((ServoImplEx) robot.servoArray[servo1]).setPwmDisable();
//            }

//            // move the active selection up and down
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.isRepeating)) {
//                selected--;
//                if (selected < 0) selected = selections - 1;
//            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_down, ZZ_ButtonMgr.State.isRepeating)) {
//                selected++;
//                if (selected > selections - 1) selected = 0;
//            }
//
//            int selMotor = 0;
//            if (selected == 0) selMotor = motor1;
//            if (selected == 1) selMotor = motor2;
//
//            if (selected == 0 || selected == 1) {  // a motor is selected
//
//                // set selected motor forward/reverse (left)
//                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_left, ZZ_ButtonMgr.State.wasPressed)) {
//                    updateTimeout(selMotor);
//                    reverse[selMotor] = !reverse[selMotor];
//                    robot.motorArray[selMotor].setDirection(reverse[selMotor] ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
//                    if (live[selMotor])
//                        oldPow[selMotor] += 0.000001;    // to force a direction change if running
//                }
//
//                // set selected motor to live or not (right)
//                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_right, ZZ_ButtonMgr.State.wasPressed)) {
//                    if (paused[selMotor]) {
//                        updateTimeout(selMotor);
//                    } else {
//                        updateTimeout(selMotor);
//                        live[selMotor] = !live[selMotor];
//                        if (live[selMotor])
//                            oldPow[selMotor] += 0.000001;    // for the initial go live
//                    }
//                }
//
//                // set selected motor to brake or not (left bumper)
//                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.left_bumper, ZZ_ButtonMgr.State.wasPressed)) {
//                    updateTimeout(selMotor);
//                    brake[selMotor] = !brake[selMotor];
//                    if (brake[selMotor]) {
//                        robot.motorArray[selMotor].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//                    } else {
//                        robot.motorArray[selMotor].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//                    }
//                }
//
//                // set selected motor to use encoder or not (right bumper)
//                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.right_bumper, ZZ_ButtonMgr.State.wasPressed)) {
//                    updateTimeout(selMotor);
//                    encoder[selMotor] = !encoder[selMotor];
//                    if (encoder[selMotor]) {
//                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                    } else {
//                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                }
//
//                // reset the encoder for the selected motor (start)
//                if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
//                    updateTimeout(selMotor);
//                    robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                    if (encoder[selMotor]) {
//                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                    } else {
//                        robot.motorArray[selMotor].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                }
//            }

//            // stop all motors (back)
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.back, ZZ_ButtonMgr.State.wasPressed)) {
//                for (int i = 0; i < numMotors; i++) {
//                    robot.motorArray[i].setPower(0);
//                    oldPow[i] = 0;
//                    newPow[i] = 0;
//                }
//            }

//            // decide if power is driven by left or right stick
//            if (engaged) {
//                newPow[motor1] = -gamepad1.left_stick_y;
//                newPow[motor2] = -gamepad1.left_stick_y;
//            }
//            else {
//                newPow[motor1] = -gamepad1.left_stick_y;
//                newPow[motor2] = -gamepad1.right_stick_y;
//            }
//
//            // update the motor power if live and the position has changed
//            for (int i = 0; i < numMotors; i++) {
//                if (live[i] && newPow[i] != oldPow[i]) {
//                    updateTimeout(i);
//                    robot.motorArray[i].setPower(newPow[i]);
//                    oldPow[i] = newPow[i];
//                }
//            }
//
//            // timeout the motor if not interacted with (trying to avoid lightly stalled motors)
//            for (int i = 0; i < numMotors; i++) {
//                if (motorTimeout[i] < System.currentTimeMillis()) {
//                    if (robot.motorArray[i].getPower() != 0) {
//                        paused[i] = true;
//                        robot.motorArray[i].setPower(0);
//                        // leaving oldPow[i] alone intentionally
//                    }
//                }
//            }

            telemetry.addLine("=============  ZZ Shoot Tester  =============");
            telemetry.addLine();
            telemetry.addLine("X to launch Pink");
            telemetry.addLine("Y to launch Blue");
            telemetry.addLine("B to launch Green");
            telemetry.addLine("A to launch ALL");
            telemetry.addLine("Start to reset ALL");
            telemetry.addLine();
            telemetry.addLine("UP to spin motors");
            telemetry.addLine("DOWN to stop motors");
            telemetry.addLine();
            telemetry.addLine("Back to stop all");
            telemetry.addLine();
            telemetry.addLine("~~~~~~");
            telemetry.addLine();

//            telemetry.addLine("up/down to select actuator");
//            telemetry.addLine();
//            telemetry.addLine("Motors:");
//            telemetry.addLine("  left for forward/reverse |  right for live/not");
//            telemetry.addLine("  l_bumper for brake/not  |  r_bumper for encoder/not");
//            telemetry.addLine("  back for stop all              |  start for reset encoder");
//            telemetry.addLine();
//            telemetry.addLine("Servo:");
//            telemetry.addLine("  a for engage  |  b for disengage  |  y for disable");
//            telemetry.addLine();
//            telemetry.addLine("When PTO is disengaged:");
//            telemetry.addLine("  LEFT STICK for motor1   | RIGHT STICK for motor2");
//            telemetry.addLine("When PTO is engaged:");
//            telemetry.addLine("  LEFT STICK for both motors");
//            telemetry.addLine();
//            telemetry.addLine("[sel] [name] [live][encoder][brake][dir] [current] [new] [encoder] [vel]");
//            telemetry.addLine();

            // Build telemetry strings
            //motors
//            for (int i = 0; i < numMotors; i++) {
//                String telString;
//                telString = (i == active) ? "=> " : "     ";
//                telString += (robot.motorNames[i] + "               ").substring(0, 10);
//                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + " ";
//                telString += (live[i] ? "L" : "_"); // + " ";
//                telString += (brake[i] ? "B" : "_"); // + " ";
//                telString += (reverse[i] ? "R" : "F") + "   ";
//                telString += "(" + String.format("%04d", rpmOptions[findTickIndex(ticksPerRev[i])]) + ")   ";
//                telString += String.format("%05d", (int)newRPM[i]) + "   ";
//                telString += String.format("%05d", (int)(robot.motorArray[i].getVelocity() * spinMultiplier[i]));
//                telString += (i == active) ? " <=" : "     ";
//                telemetry.addLine(telString);
//            }
            String telString;
            telString = "Set Speed:   " + String.format("%05d", (int)motorSpinSpeed);
            telemetry.addLine(telString);
            telString = "SpinMotor 1: " + String.format("%05d", getMotorSpinSpeed(spinMotor1));
            telemetry.addLine(telString);
            telString = "SpinMotor 2: " + String.format("%05d", getMotorSpinSpeed(spinMotor2));
            telemetry.addLine(telString);

            telemetry.addLine();

            //servo
//            int i = servo1;
//            String telString;
//            telString = (i == -1) ? "=>  " : "      ";
//            telString += (robot.servoNames[i] + "               ").substring(0, 10);
//            telString += (servoEnabled ? "E" : "_") + "    ";
//            telString += String.format("%.3f", servoPos);
//            telString += (engaged ? "  engaged" : "  disengaged");
//            telemetry.addLine(telString);

            for (int i = 0; i < numServos; i++) {
//                String telString;
                telString = (i == -1) ? "=>  " : "      ";
                telString += (robot.servoNames[i] + "               ").substring(0, 10);
//                telString += ((binding[i] != 0) ? String.valueOf(binding[i]) : " ") + "    ";
//                telString += (live[i] ? "L" : "_"); // + " ";
//                telString += (enabled[i] ? "E" : "_"); // + " ";
//                telString += (reverse[i] ? "R" : "F") + "    ";
                telString += String.format("%.3f", robot.servoArray[i].getPosition()) + "    ";
//                telString += String.format("%.3f", newPos[i]);
                telString += (i == -1) ? "  <=" : "      ";
                telemetry.addLine(telString);
            }

            ZZ_StateMachine.addTelemetry(telemetry);

            telemetry.update();
        }
    }

//    public int findTickIndex(double ticks) {
//        for (int i = 0; i < tickOptions.length; i++) {
//            if (ticks == tickOptions[i]) return i;
//        }
//        return 0;
//    }
//
//    public void updateTickSettings(int motorNum, double ticks) {
//        ticksPerRev[motorNum] = ticks;
//        spinMultiplier[motorNum] = 60.0 / ticks;
//        maxRPM[motorNum] = 168200 / ticks;
//    }

    public int getMotorSpinSpeed(DcMotorEx m) {
        return (int) (m.getVelocity() * 60.0 / spinTicks);
    }

    void buildStateMachines() {
        ZZ_StateMachine task;

        launchGreen = new ZZ_StateMachine("green");
        task = launchGreen;
        //task.setGroups("launcher", "green");
        //task.setStopGroups("launcher", "green");    // groups to kill
        task.setMemberGroups("all", "green");  // will be killed by
        task.setAutoRestart(false);
        //task.setStopRunnable( () -> {} );
        //task.setTimeLimit(5000);
        //task.setTimeoutRunnable( () -> {} );
        //task.setEndCriteria( () -> false );
        //task.setEndCriteriaRunnable( () -> {} );
        task.addRunOnce(() -> servoGreen.setPosition(servoGreenLaunch));
        task.addWaitFor(() -> servoGreen.isDone());
        task.addDelayOf(500);
        task.addRunOnce(() -> servoGreen.setPosition(servoGreenDock));

        launchPink = new ZZ_StateMachine("pink");
        task = launchPink;
        task.setMemberGroups("all", "pink");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> servoPink.setPosition(servoPinkLaunch));
        task.addWaitFor(() -> servoPink.isDone());
        task.addDelayOf(500);
        task.addRunOnce(() -> servoPink.setPosition(servoPinkDock));

        launchBlue = new ZZ_StateMachine("blue");
        task = launchBlue;
        task.setMemberGroups("all", "blue");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce( () -> servoBlue.setPosition(servoBlueLaunch) );
        task.addRunOnce( () -> servoWhite.setPosition(servoWhiteLaunch) );
        task.addWaitFor( () -> servoBlue.isDone());
        task.addRunOnce( () -> servoWhite.setPosition(servoWhiteClearance) );
        task.addDelayOf( 500);
        task.addRunOnce( () -> servoBlue.setPosition(servoBlueDock) );
        task.addDelayOf( 150);
        task.addRunOnce( () -> servoWhite.setPosition(servoWhiteDock) );

        launchAll = new ZZ_StateMachine("all");
        task = launchAll;
        task.setStopGroups("green", "blue", "pink");    // groups to kill
        task.setMemberGroups("reset");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(launchBlue::restartNoStop);
        task.addRunOnce(launchGreen::restartNoStop);
        task.addRunOnce(launchPink::restartNoStop);
        task.addWaitFor(launchPink::isDone);
        task.addWaitFor(launchGreen::isDone);
        task.addWaitFor(launchBlue::isDone);

        resetAll = new ZZ_StateMachine("reset");
        task = resetAll;
        task.setStopGroups("green", "blue", "pink");    // groups to kill
        //task.setMemberGroups("blue");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce( ()-> {
            servoPink.setPosition(servoPinkDock);
            servoGreen.setPosition(servoGreenDock);
            servoBlue.setPosition(servoBlueDock); // potential for jamming?
            servoWhite.setPosition(servoWhiteDock);
        });

    }

}