package org.firstinspires.ftc.teamcode.RobotParts.Test2025;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Odometry;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.i2c.AdafruitNeoDriver;

public class PartsT25 extends Parts {
    public PartsT25(LinearOpMode opMode) {
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
        pinpointRobotOffset = new Position(-56.0,52.0,0);  // In mm, Refer to User Guide, Y offset of X, X offset of Y, R will be ignored
        odoRobotOffset = new Position (2.25,0,0);  // if this is inherent to the robot, should it be in PartsDS?
        slamraRobotOffset = new Position(-8,-0.75,0); //new Position(-8,-1,0);

        robotV2 = new RobotT25(this);
        buttonMgr = new ButtonMgr(opMode);
        controls = new ControlsT25(this);
        drivetrain = new DrivetrainT25(this);

        if (useIMU) imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
        autoDrive = new AutoDriveT25(this);
        userDrive = new UserDriveT25(this);
        t25_Misc = new T25_Misc(this);
        t25_Effector = new T25_Effector(this);
//        sb_Auto = new SB_Auto(this);

        if (useODO) {
            odometry = new OdometryT25(this);
            odometry.odoFieldStart = fieldStartPosition;
            odometry.odoRobotOffset = odoRobotOffset;
        }
        if (usePinpoint) {
            pinpoint = new PinpointT25(this);
            pinpoint.pinpointFieldStart = fieldStartPosition;
            pinpoint.pinpointRobotOffset = pinpointRobotOffset;  // Rotation will be ignored
        }
        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
        }

        if (useNeoMatrix) neo = new NeoMatrix(opMode, "neo", 8, 8, AdafruitNeoDriver.ColorOrder.GRB);  //RGB for fairy string

        drivetrain.accelControl = false;
        drivetrain.minimizeCycleTime = false;
        userDrive.useHeadingHold = false;

    }

    @Override
    public void preInit() {
        robotV2.initialize();
        if (useIMU) imuMgr.initialize();
        if (usePinpoint) {
            pinpoint.initialize();
            opMode.sleep(500);
            pinpoint.preInit();
        }
        positionMgr.initialize();
        if (useSlamra) slamra.initialize();
        t25_Effector.initialize();
    }

    @Override
    public void initLoop() {
        buttonMgr.initLoop();
        if (useIMU) {
//            imuMgr.robotV2 = true;
            imuMgr.initLoop();
        }
        if (usePinpoint) pinpoint.initLoop();
        if (useSlamra) slamra.initLoop();
        positionMgr.initLoop();
        TelemetryMgr.Update();
    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        if (useIMU) imuMgr.preRun();
        if (usePinpoint) pinpoint.preRun();
        if (useODO) odometry.initialize();
        userDrive.initialize();
//        autoDrive.initialize();
        if (useODO) odometry.runLoop();  // get some things squared away before the regular runLoops start
//        autoDrive.runLoop();
        if (useSlamra) slamra.preRun();
    }

    @Override
    public void runLoop() {
        addTelemetryLoopStart();

        robotV2.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (usePinpoint) pinpoint.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        positionMgr.runLoop();
        controls.runLoop();
        userDrive.runLoop();
//        autoDrive.runLoop();
        drivetrain.runLoop();
//        t24Grabber.runLoop();
        t25_Effector.runLoop();

        addTelemetryLoopEnd();
        TelemetryMgr.Update();
    }

    @Override
    public void autoRunLoop() {
        if (!opMode.opModeIsActive()) return;
        addTelemetryLoopStart();

        robotV2.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (usePinpoint) pinpoint.runLoop();
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
    }
}
