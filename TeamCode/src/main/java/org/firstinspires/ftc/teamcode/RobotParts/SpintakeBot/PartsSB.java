package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.AutoDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Odometry;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.i2c.AdafruitNeoDriver;

public class PartsSB extends Parts {
    public PartsSB(LinearOpMode opMode) {
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
        robot = new RobotSB(this);
        buttonMgr = new ButtonMgr(opMode);
        controls = new ControlsSB(this);
        drivetrain = new DrivetrainSB(this);

        if (useIMU) imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
        autoDrive = new AutoDriveSB(this);
        userDrive = new UserDriveSB(this);
        sb_Misc = new SB_Misc(this);
        sb_Intake = new SB_Intake(this);
//        sb_Auto = new SB_Auto(this);

        if (useODO) {
            odometry = new Odometry(this);
            odometry.odoFieldStart = fieldStartPosition;
            odometry.odoRobotOffset = odoRobotOffset;
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
        robot.initialize();
        if (useIMU) imuMgr.initialize();
        positionMgr.initialize();
        if (useSlamra) slamra.initialize();
        sb_Intake.initialize();
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
    }

    @Override
    public void runLoop() {
        addTelemetryLoopStart();

        robot.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        positionMgr.runLoop();
        controls.runLoop();
        userDrive.runLoop();
//        autoDrive.runLoop();
        drivetrain.runLoop();
//        t24Grabber.runLoop();
        sb_Intake.runLoop();

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
