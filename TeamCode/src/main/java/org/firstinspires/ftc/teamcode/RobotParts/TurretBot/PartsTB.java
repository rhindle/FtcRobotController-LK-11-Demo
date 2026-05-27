package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.EncoderTracker;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Odometry;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.i2c.AdafruitNeoDriver;

public class PartsTB extends Parts {
    public PartsTB(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void setup(){
        // We do this after the construct because we may want to adjust some settings before creating all the sub-parts
        if (isSetup) {
            //throw new RuntimeException("Parts can only be setup once");
            return;
        }
        useRobotV2 = true;
        isSetup = true;
        pinpointRobotOffset = new Position(-86,-118,0);  // 14273 new chassis measured

//        pinpointRobotOffset = new Position(-56.0,52.0,0);  // In mm, Refer to User Guide, Y offset of X, X offset of Y, R will be ignored
//        odoRobotOffset = new Position (2.25,0,0);  // if this is inherent to the robot, should it be in PartsDS?
//        slamraRobotOffset = new Position(-8,-0.75,0); //new Position(-8,-1,0);

        robotV2 = new RobotTB(this);
        buttonMgr = new ButtonMgr(opMode);
        controls = new ControlsTB(this);
        drivetrain = new DrivetrainTB(this);

        if (useIMU) imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
        autoDrive = new AutoDriveTB(this);
        userDrive = new UserDriveTB(this);
        TB_Intake.setup(this);
        TB_Turret.setup(this);
        TB_Misc.setup(this);
        TB_Tasks.setup(this);
        TB_TasksAuto.setup(this);

//        sb_Auto = new SB_Auto(this);

        if (useODO) {
            odometry = new Odometry(this);
            odometry.odoFieldStart = decideStartPosition(false); //fieldStartPosition;
            odometry.odoRobotOffset = odoRobotOffset;
        }
        if (usePinpoint) {
            pinpoint = new PinpointTB(this);
            pinpoint.pinpointFieldStart = decideStartPosition(true); //fieldStartPosition;
            pinpoint.pinpointRobotOffset = pinpointRobotOffset;  // Rotation will be ignored
        }
        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = decideStartPosition(false); //fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
        }
        if (useEncoderTracker) {
            encoderTracker = new EncoderTracker(this);
//            encoderTracker.encoderRobotPosition = fieldStartPosition.clone();  // this is going to be updated so must be a clone, not a direct reference
//            encoderTracker.encoderRobotPositionAbsolute = fieldStartPosition.clone();  // as above
            encoderTracker.encoderRobotPosition = decideStartPosition(false).clone();
            encoderTracker.encoderRobotPositionAbsolute = decideStartPosition(false).clone();
        }

        if (useNeoMatrix) neo = new NeoMatrix(opMode, "neo", 8, 8, AdafruitNeoDriver.ColorOrder.GRB);  //RGB for fairy string

        if (useLimeLight) TB_LL.setup(this);

        drivetrain.accelControl = true;  //!!!!!!!!!! todo: remove this if drivers don't like
        drivetrain.accelControlRamp = 250;
        drivetrain.minimizeCycleTime = false;
        userDrive.useHeadingHold = false;
        userDrive.useHoldPosition = false;
        // modify the following defaults to work with turretbot (P should probably be lower)
        autoDrive.PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
        autoDrive.PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
        StateMachine.reset();
    }

    Position decideStartPosition(boolean pinpoint) {
        Position startPos;
        if (TB_Misc.isAuto()) startPos=fieldStartPosition;         // all autonomous starts need position set
        else if (TB_Misc.noPosition) startPos=fieldStartPosition;  // if position not set yet in teleop
        //if (pinpoint) startPos=
        else startPos = TB_Misc.currentPosition;                   // use the stored position (assuming the robot has not moved)
        return startPos;
    }

    @Override
    public void preInit() {
        robotV2.initialize();
        if (useIMU) imuMgr.initialize();
        if (usePinpoint) {
            pinpoint.initialize();  // reset was removed from init; only do it in auto (or if no position)
            if (TB_Misc.isAuto() || TB_Misc.noPosition) {
                pinpoint.reset();
                opMode.sleep(500);
                pinpoint.preInit();   // this sets the position
//                TB_Misc.noPosition = false;
            }
        }
        positionMgr.initialize();
        if (useSlamra) slamra.initialize();
        if (useLimeLight) TB_LL.initialize();
        TB_Intake.initialize();
        TB_Turret.initialize();
        TB_Tasks.initialize();
        TB_TasksAuto.initialize();
          // init servos here only in autonomous; in teleop, no movement is permitted
          // (and they should have already been init in autonomous anyway)
        if (TB_Misc.isAuto() && !TB_Misc.servosInit) TB_Tasks.smInitServos.restart();
        TB_Misc.noPosition = false;   // todo: REVISIT THIS LAST MINUTE HACK =============================
    }

    @Override
    public void initLoop() {
        buttonMgr.initLoop();
        if (useIMU) imuMgr.initLoop();
        if (usePinpoint) pinpoint.initLoop();
        if (useSlamra) slamra.initLoop();
        if (useLimeLight) TB_LL.initLoop();
        positionMgr.initLoop();
        TelemetryMgr.Update();
    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        if (useIMU) imuMgr.preRun();
        if (usePinpoint && TB_Misc.isAuto()) pinpoint.preRun(); // this sets the position (again)
        if (useODO) odometry.initialize();
        userDrive.initialize();
        autoDrive.initialize();
        if (useODO) odometry.runLoop();  // get some things squared away before the regular runLoops start
        autoDrive.runLoop();
        if (useSlamra) slamra.preRun();
        if (useLimeLight) TB_LL.preRun();
        if (useEncoderTracker) encoderTracker.preRun();
        if (TB_Misc.isTeleOp() && !TB_Misc.servosInit) TB_Tasks.smInitServos.restart();
    }

    @Override
    public void runLoop() {
        addTelemetryLoopStart();

        robotV2.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (useEncoderTracker) encoderTracker.runLoop();
        if (usePinpoint) pinpoint.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        if (useLimeLight) TB_LL.runLoop();
        positionMgr.runLoop();
        controls.runLoop();
        userDrive.runLoop();
        autoDrive.runLoop();
        drivetrain.runLoop();
        TB_Intake.runLoop();
        TB_Turret.runLoop();
        StateMachine.runLoop();

        addTelemetryLoopEnd();
        StateMachine.addTelemetry();
        TelemetryMgr.Update();
        TB_Misc.currentPosition = positionMgr.robotPosition;
    }

    @Override
    public void autoRunLoop() {
        if (!opMode.opModeIsActive()) return;
        addTelemetryLoopStart();

        robotV2.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (useEncoderTracker) encoderTracker.runLoop();
        if (usePinpoint) pinpoint.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        if (useLimeLight) TB_LL.runLoop();
        positionMgr.runLoop();
        controls.runLoop();  // todo: comment this out?
//        userDrive.runLoop();
        autoDrive.runLoop();
        drivetrain.runLoop();
        TB_Intake.runLoop();
        TB_Turret.runLoop();
        StateMachine.runLoop();

        addTelemetryLoopEnd();
        StateMachine.addTelemetry();
        TelemetryMgr.Update();
        TB_Misc.currentPosition = positionMgr.robotPosition;
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.stop();
        drivetrain.stop();
        if (useLimeLight) TB_LL.stop();
    }

    private void addTelemetryLoopStart() {
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "Loop time (ms)", JavaUtil.formatNumber(Functions.calculateLoopTime(), 0));
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "IMU raw heading", useIMU ? JavaUtil.formatNumber(imuMgr.returnImuHeadingRaw(),2) : "(not used)");
        if (useODO) odometry.addTeleOpTelemetry();
    }

    @SuppressLint("DefaultLocale")
    private void addTelemetryLoopEnd() {
        TelemetryMgr.message(TelemetryMgr.Category.CONTROLS, "speed ", JavaUtil.formatNumber(controls.driveData.driveSpeed, 2));
        TelemetryMgr.message(TelemetryMgr.Category.CONTROLS, "angle ", JavaUtil.formatNumber(controls.driveData.driveAngle, 2));
        TelemetryMgr.message(TelemetryMgr.Category.CONTROLS, "rotate", JavaUtil.formatNumber(controls.driveData.rotate, 2));
        TelemetryMgr.message(TelemetryMgr.Category.USERDRIVE, "storedHeading", JavaUtil.formatNumber(userDrive.storedHeading, 2));
        TelemetryMgr.message(TelemetryMgr.Category.USERDRIVE, "deltaHeading", JavaUtil.formatNumber(userDrive.deltaHeading, 2));
        TelemetryMgr.message(TelemetryMgr.Category.IMU, "IMU-Modified", useIMU ? JavaUtil.formatNumber(imuMgr.returnImuRobotHeading(),2) : "(not used)");
    }
}
