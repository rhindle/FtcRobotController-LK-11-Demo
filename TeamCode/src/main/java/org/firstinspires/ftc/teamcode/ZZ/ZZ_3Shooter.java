package org.firstinspires.ftc.teamcode.ZZ;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp (name="ZZ_3Shooter", group="Test")
@Disabled
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
    int numMotors;
    double spinTicks = 28.0;
    int spinRPM = 6000;

    int numServos;

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
            telemetry.addLine("Press UP to toggle launch PID");
            telemetry.addLine();
            telemetry.addLine("Current Selection: " + (robot.zz_dualHub ? "Dual Hubs" : "Single Hub"));
            telemetry.addLine("PID: " + (robot.zz_spinPID ? "Launch" : "default"));
            telemetry.update();
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.x, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_dualHub = false;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.y, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_dualHub = true;
            }
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.wasPressed)) {
                robot.zz_spinPID = !robot.zz_spinPID;
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

        numServos = robot.servoNames.length;

        // set all motors to default
        for (int i = 0; i < numMotors; i++) {
            reverse[i] = false;
            live[i] = false;
            brake[i] = false;
            robot.motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            robot.motorArray[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.motorArray[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            if (robot.zz_spinPID) robot.motorArray[i].setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, launchSpinPID);
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
            if (buttonMgr.getState(1, ZZ_ButtonMgr.Buttons.dpad_up, ZZ_ButtonMgr.State.isPressed)) {
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

            telemetry.addLine("=============  ZZ Shoot Tester  =============");
            telemetry.addLine();
            telemetry.addLine("X to launch Pink");
            telemetry.addLine("Y to launch Blue");
            telemetry.addLine("B to launch Green");
            telemetry.addLine("A to launch ALL");
            telemetry.addLine("Start to reset ALL");
            telemetry.addLine();
            telemetry.addLine("UP to spin motors");
            telemetry.addLine("  left stick for large changes");
            telemetry.addLine("  right stick for small changes");
            telemetry.addLine("DOWN to stop motors");
            telemetry.addLine();
            telemetry.addLine("Back to stop all");
            telemetry.addLine();
            telemetry.addLine("~~~~~~");
            telemetry.addLine();

            // Build telemetry strings
            //motors
            String telString;
            telString = "Set Speed:   " + String.format("%05d", (int)motorSpinSpeed);
            telemetry.addLine(telString);
            telString = "SpinMotor 1: " + String.format("%05d", getMotorSpinSpeed(spinMotor1));
            telemetry.addLine(telString);
            telString = "SpinMotor 2: " + String.format("%05d", getMotorSpinSpeed(spinMotor2));
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