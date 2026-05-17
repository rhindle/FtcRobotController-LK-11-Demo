package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="YY_Turret", group="Test")
//@Disabled
public class YY_Turret extends LinearOpMode {

    ZZ_Robot_2025 robot;
    ZZ_ButtonMgr buttonMgr;

    ZZ_ServoSSR servoGateL;
    ZZ_ServoSSR servoGateR;
    ZZ_ServoSSR servoHood;
    ZZ_ServoSSR servoTurretL;
    ZZ_ServoSSR servoTurretR;

    DcMotorEx motorSpin1;
    DcMotorEx motorSpin2;
    DcMotorEx motorIntake;
    DcMotorEx motorTransfer;

    ZZ_StateMachine smIntakeOn;
    ZZ_StateMachine smIntakeOff;
    ZZ_StateMachine smRelax;
    ZZ_StateMachine smLaunch;
    ZZ_StateMachine smSpinDown;

    static String motorIntakeName = "motor0B";
    static String motorTransferName = "motor1B";
    static String motorSpin1Name = "motor2B";
    static String motorSpin2Name = "motor3B";

    static String servoGateLName = "servo0B";
    static String servoGateRName = "servo1B";
    static String servoHoodName = "servo2B";
    static String servoTurretLName = "servo3B";
    static String servoTurretRName = "servo4B";

    final double servoGateLOpen = 1;
    final double servoGateLClosed = 0;
    final double servoGateROpen = 1;
    final double servoGateRClosed = 0;

    static double servoHoodPos = 0.5;
    static double motorSpinSpeed                = 1500;
    static double motorSpinIdleSpeed            = 500;
    final boolean motor1reverse             = false;
    final boolean motor2reverse             = false;

    boolean[] reverse;
    boolean[] live;
    boolean[] brake;
    int numMotors;
    double spinTicks = 28.0;
    int spinRPM = 6000;

    int numServos;

    final double spinSmallChange = .0001;
    final double spinLargeChange = .005;
    final double hoodChange = .002;
    final int reverseTicks = 10;
    boolean intakeRunning = false;

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
//            telemetry.addLine("Press UP to toggle launch PID");
            telemetry.addLine();
            telemetry.addLine("Current Selection: " + (robot.zz_dualHub ? "Dual Hubs" : "Single Hub"));
//            telemetry.addLine("PID: " + (robot.zz_spinPID ? "Launch" : "default"));
            telemetry.update();
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_dualHub = false;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_dualHub = true;
            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.wasPressed)) {
//                robot.zz_spinPID = !robot.zz_spinPID;
//            }
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

        numServos = robot.servoNames.length;

        // set all motors to default
        for (int i = 0; i < numMotors; i++) {
            reverse[i] = false;
            live[i] = false;
            brake[i] = false;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//            if (robot.zz_spinPID) robot.motorArray[i].setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);
        }

        servoGateL = new ZZ_ServoSSR(robot.getServoByName(servoGateLName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(250);
        servoGateR = new ZZ_ServoSSR(robot.getServoByName(servoGateRName)).setDirectionSSR(Servo.Direction.REVERSE).setSweepTime(250);
        servoHood = new ZZ_ServoSSR(robot.getServoByName(servoHoodName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1200);
        servoTurretL = new ZZ_ServoSSR(robot.getServoByName(servoTurretLName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1500);
        servoTurretR = new ZZ_ServoSSR(robot.getServoByName(servoTurretRName)).setDirectionSSR(Servo.Direction.FORWARD).setSweepTime(1500);

        motorSpin1 = robot.getMotorByName(motorSpin1Name);
        motorSpin2 = robot.getMotorByName(motorSpin2Name);
        motorIntake = robot.getMotorByName(motorIntakeName);
        motorTransfer = robot.getMotorByName(motorTransferName);

        motorSpin1.setDirection(DcMotorEx.Direction.REVERSE);
        motorSpin2.setDirection(DcMotorEx.Direction.REVERSE);
        motorSpin1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);
        motorSpin2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);

        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);

        buildStateMachines();
//        resetAll.restart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.runLoop();  // this will take care of clearing out the bulk reads
            buttonMgr.updateAll();
            ZZ_StateMachine.runLoop();

            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_left, ZZ_ButtonMgr.State.wasPressed)) {
                servoGateL.setPosition(servoGateLOpen);
                servoGateR.setPosition(servoGateROpen);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_right, ZZ_ButtonMgr.State.wasPressed)) {
                servoGateL.setPosition(servoGateLClosed);
                servoGateR.setPosition(servoGateRClosed);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.isPressed)) {
                servoHoodPos += hoodChange;
                servoHoodPos = Math.max(0, Math.min(1, servoHoodPos));
                servoHood.setPosition(servoHoodPos);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_down, ZZ_ButtonMgr.State.isPressed)) {
                servoHoodPos -= hoodChange;
                servoHoodPos = Math.max(0, Math.min(1, servoHoodPos));
                servoHood.setPosition(servoHoodPos);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.left_bumper, ZZ_ButtonMgr.State.isPressed)) {
                // modify spin speed
                motorSpinSpeed += gamepad1.left_stick_y * -spinLargeChange * spinRPM;
                motorSpinSpeed += gamepad1.right_stick_y * -spinSmallChange * spinRPM;
                motorSpinSpeed = Math.max(-spinRPM, Math.min(spinRPM, motorSpinSpeed));
            }

            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.a, ZZ_ButtonMgr.State.wasPressed)) {
                // toggle intake on  (close gate, start intake)
                intakeRunning = !intakeRunning;
                if (intakeRunning) smIntakeOn.restart();
                else smIntakeOff.restart();
//                smIntakeOn.restart();
            }
//            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.a, ZZ_ButtonMgr.State.wasReleased)) {
//                // toggle intake off  (stop intake, release pressure)
//                smIntakeOff.restart();
//            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.isPressed)) {
                // spin up
                ZZ_StateMachine.stopGroups("spinner");
                motorSpin1.setVelocity(motorSpinSpeed / (60.0 / spinTicks));
                motorSpin2.setVelocity(motorSpinSpeed / (60.0 / spinTicks));
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                //intake+transfer   (open gate, start both motors)
                intakeRunning = false;
                smLaunch.restart();
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.b, ZZ_ButtonMgr.State.wasPressed)) {
                // idle
//                motorSpin1.setVelocity(motorSpinIdleSpeed / (60.0 / spinTicks));
//                motorSpin2.setVelocity(motorSpinIdleSpeed / (60.0 / spinTicks));
                smSpinDown.restart();
                motorIntake.setPower(0);
                motorTransfer.setPower(0);
                servoGateL.setPosition(servoGateLClosed);
                servoGateR.setPosition(servoGateRClosed);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.right_bumper, ZZ_ButtonMgr.State.wasPressed)) {
                // reverse
                motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motorTransfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motorIntake.setPower(-1);
                motorTransfer.setPower(-1);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.right_bumper, ZZ_ButtonMgr.State.wasReleased)) {
                motorIntake.setPower(0);
                motorTransfer.setPower(0);
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.start, ZZ_ButtonMgr.State.wasPressed)) {
                // stop spin
                ZZ_StateMachine.stopGroups("spinner");
                motorSpin1.setPower(0);
                motorSpin2.setPower(0);
            }
            // stop all
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.back, ZZ_ButtonMgr.State.wasPressed)) {
                ZZ_StateMachine.stopAll();
                motorSpin1.setPower(0);
                motorSpin2.setPower(0);
                motorIntake.setPower(0);
                motorTransfer.setPower(0);
                servoHood.disable();
                servoGateL.disable();
                servoGateR.disable();
                servoTurretL.disable();
                servoTurretR.disable();
            }

            telemetry.addLine("=============  YY Turret Tester  =============");
            telemetry.addLine();
            telemetry.addLine("DPAD L/R for gate Open/Close");
            telemetry.addLine("DPAD U/D to change hood position");
            telemetry.addLine("A to toggle Intake");
            telemetry.addLine("X to High Spin Speed");
            telemetry.addLine("Y to Intake+Transfer (i.e., launch)");
            telemetry.addLine("B to Idle");
            telemetry.addLine("RB to reverse Intake+Transfer");
            telemetry.addLine();
            telemetry.addLine("Back to stop all");
            telemetry.addLine();
            telemetry.addLine("LB to adjust spin speed");
            telemetry.addLine("  left stick for large changes");
            telemetry.addLine("  right stick for small changes");
            telemetry.addLine();
            telemetry.addLine("~~~~~~");
            telemetry.addLine();

            // Build telemetry strings
            //motors
            String telString;
            telString = "Set Speed:   " + String.format("%05d", (int)motorSpinSpeed);
            telemetry.addLine(telString);
            telString = "SpinMotor 1: " + String.format("%05d", getMotorSpinSpeed(motorSpin1));
            telemetry.addLine(telString);
            telString = "SpinMotor 2: " + String.format("%05d", getMotorSpinSpeed(motorSpin2));
            telemetry.addLine(telString);

            telemetry.addLine();

            //servo
            for (int i = 0; i < numServos; i++) {
                telString = (i == -1) ? "=>  " : "      ";
                telString += (robot.servoNames[i] + "               ").substring(0, 10);
                telString += String.format("%.3f", robot.servoArray[i].getPosition()) + "    ";
                telString += (i == -1) ? "  <=" : "      ";
                telemetry.addLine(telString);
            }

            telemetry.addLine();
            ZZ_StateMachine.addTelemetry(telemetry);

            telemetry.update();
        }
    }

    public int getMotorSpinSpeed(DcMotorEx m) {
        return (int) (m.getVelocity() * 60.0 / spinTicks);
    }

    void buildStateMachines() {
        ZZ_StateMachine task;

        // toggle intake on  (close gate, start intake)
        // toggle intake off  (stop intake, release pressure)
        //intake+transfer   (open gate, start both motors)
        // release pressure

        smIntakeOn = new ZZ_StateMachine("intakeOn");
        task = smIntakeOn;
        task.setGroups("intake");  // will be killed by
        //task.setStopGroups("launcher", "green");    // groups to kill
        //task.setMemberGroups("all", "green");  // will be killed by
        task.setAutoRestart(false);
        //task.setStopRunnable( () -> {} );
        //task.setTimeLimit(5000);
        //task.setTimeoutRunnable( () -> {} );
        //task.setEndCriteria( () -> false );
        //task.setEndCriteriaRunnable( () -> {} );
        task.addRunOnce(() -> {
            motorTransfer.setPower(0);
            servoGateL.setPosition(servoGateLClosed);
            servoGateR.setPosition(servoGateRClosed);
        });
        task.addWaitFor(() -> servoGateL.isDone() && servoGateR.isDone());
        task.addRunOnce(() -> {
            motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorIntake.setPower(1);
        });

        smIntakeOff = new ZZ_StateMachine("intakeOff");
        task = smIntakeOff;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            motorIntake.setPower(0);
            motorTransfer.setPower(0);
            smRelax.restartNoStop();
        });
        task.addWaitFor(() -> smRelax.isDone());

        smLaunch = new ZZ_StateMachine("launch");
        task = smLaunch;
        task.setGroups("intake", "transfer", "spinner");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            motorIntake.setPower(0);
            motorTransfer.setPower(0);
            motorIntake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorTransfer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            servoGateL.setPosition(servoGateLOpen);
            servoGateR.setPosition(servoGateROpen);
        });
        task.addWaitFor(() -> servoGateL.isDone() && servoGateR.isDone());
        task.addRunOnce(() -> {
            motorIntake.setPower(1);
            motorTransfer.setPower(1);
        });

        smRelax = new ZZ_StateMachine("relax");
        task = smRelax;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            motorIntake.setPower(0);
            motorTransfer.setPower(0);
            motorIntake.setTargetPosition(motorIntake.getCurrentPosition()-reverseTicks);
            motorTransfer.setTargetPosition(motorTransfer.getCurrentPosition()-reverseTicks);
            motorIntake.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorTransfer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motorIntake.setPower(1);
            motorTransfer.setPower(1);
        });

        smSpinDown = new ZZ_StateMachine("spinDown");
        task = smSpinDown;
        task.setGroups("spinner");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            motorSpin1.setPower(0);
            motorSpin2.setPower(0);
        });
        task.addWaitFor(() -> getMotorSpinSpeed(motorSpin1) < motorSpinIdleSpeed);
        task.addRunOnce(() -> {
            motorSpin1.setVelocity(motorSpinIdleSpeed / (60.0 / spinTicks));
            motorSpin2.setVelocity(motorSpinIdleSpeed / (60.0 / spinTicks));
        });

    }

}