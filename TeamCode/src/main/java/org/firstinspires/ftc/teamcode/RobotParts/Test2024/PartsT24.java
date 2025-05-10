package org.firstinspires.ftc.teamcode.RobotParts.Test2024;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.AutoDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.EncoderTracker;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.i2c.AdafruitNeoDriver;

public class PartsT24 extends Parts {
    public PartsT24(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void setup(){
        // We do this after the construct because we may want to adjust some settings before creating all the sub-parts
        if (isSetup) {
            //throw new RuntimeException("Parts can only be setup once");
            return;
        }
//        odoRobotOffset = new Position (2.25,0,0);          // map odo to robot (so it holds turn position better)
        isSetup = true;
        robot = new RobotT24(this);
        buttonMgr = new ButtonMgr(opMode);
        controls = new ControlsT24(this);
        drivetrain = new DrivetrainT24(this);

        if (useIMU) imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
//        autoDrive = new AutoDriveT24(this);
        autoDrive = new AutoDrive(this);
        userDrive = new UserDriveT24(this);
        t24Misc = new T24Misc(this);
//        t24Grabber = new T24Grabber(this);
        t24MultiGrabber = new T24MultiGrabber(this);
//        t24Auto = new T24Auto(this);

        if (useODO) {
            odometry = new OdometryT24(this);
            odometry.odoFieldStart = fieldStartPosition;
            odometry.odoRobotOffset = odoRobotOffset;
        }
        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
        }
        if (useEncoderTracker) {
            encoderTracker = new EncoderTracker(this);
            encoderTracker.encoderRobotPosition = fieldStartPosition.clone();  // this is going to be updated so must be a clone, not a direct reference
            encoderTracker.encoderRobotPositionAbsolute = fieldStartPosition.clone();  // as above
        }

        if (useNeoMatrix) neo = new NeoMatrix(opMode, "neo", 8, 8, AdafruitNeoDriver.ColorOrder.GRB);  //RGB for fairy string

        drivetrain.accelControl = false;
        drivetrain.minimizeCycleTime = false;
        userDrive.useHeadingHold = false;

    }

    @Override
    public void preInit() {
        robot.initialize();
        if (useIMU) imuMgr.initialize();
        positionMgr.initialize();
        if (useSlamra) slamra.initialize();
//        t24Grabber.initialize();
        t24MultiGrabber.initialize();
    }

    @Override
    public void initLoop() {
        buttonMgr.initLoop();
        if (useIMU) imuMgr.initLoop();
        if (useSlamra) slamra.initLoop();
        positionMgr.initLoop();
        TelemetryMgr.Update();
    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        if (useIMU) imuMgr.preRun();
        if (useODO) odometry.initialize();
        userDrive.initialize();
//        autoDrive.initialize();
        if (useODO) odometry.runLoop();  // get some things squared away before the regular runLoops start
//        autoDrive.runLoop();
        if (useSlamra) slamra.preRun();
        if (useEncoderTracker) encoderTracker.preRun();
    }

    @Override
    public void runLoop() {
        addTelemetryLoopStart();

        robot.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (useEncoderTracker) encoderTracker.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        positionMgr.runLoop();
        controls.runLoop();
        userDrive.runLoop();
//        autoDrive.runLoop();
        drivetrain.runLoop();
//        t24Grabber.runLoop();
        t24MultiGrabber.runLoop();

        addTelemetryLoopEnd();
        TelemetryMgr.Update();
    }

    @Override
    public void autoRunLoop() {
        if (!opMode.opModeIsActive()) return;
        addTelemetryLoopStart();

        robot.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (useEncoderTracker) encoderTracker.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        positionMgr.runLoop();
        controls.runLoop();
////        userDrive.runLoop();
//        autoDrive.runLoop();
        drivetrain.runLoop();

        addTelemetryLoopEnd();
        TelemetryMgr.Update();
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.stop();
        drivetrain.stop();
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
        opMode.telemetry.addLine()
                .addData("Encoder 0", robot.motor0.getCurrentPosition())
                .addData("1", robot.motor1.getCurrentPosition())
                .addData("2", robot.motor2.getCurrentPosition())
                .addData("3", robot.motor3.getCurrentPosition());
    }
}
