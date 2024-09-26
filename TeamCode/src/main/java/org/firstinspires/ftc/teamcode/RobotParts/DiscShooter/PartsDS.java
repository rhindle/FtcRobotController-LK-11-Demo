package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.NeoMatrix;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.i2c.AdafruitNeoDriver;

public class PartsDS extends Parts {
    public PartsDS(LinearOpMode opMode) {
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
        robot = new RobotDS(this);
        buttonMgr = new ButtonMgr(opMode);
        controls = new ControlsDS(this);
        drivetrain = new DrivetrainDS(this);

        if (useIMU) imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
        autoDrive = new AutoDriveDS(this);
        userDrive = new UserDriveDS(this);
        dsLed = new DSLed(this);
        dsShooter = new DSShooter(this);
        dsSpeedControl = new DSSpeedControl(this);
        dsMisc = new DSMisc(this);
        dsAuto = new DSAuto(this);

        if (useAprilTag) dsApriltag = new DSAprilTag(this);
        if (useODO) {
            odometry = new OdometryDS(this);
            odometry.odoFieldStart = fieldStartPosition;
            odometry.odoRobotOffset = odoRobotOffset;
        }
        if (useSlamra) {
            slamra = new Slamra(this);
            slamra.slamraFieldStart = fieldStartPosition;
            slamra.slamraRobotOffset = slamraRobotOffset;
        }
        DSMisc.firstLock = true;

        if (useNeoMatrix) neo = new NeoMatrix(opMode, "neo", 8, 8, AdafruitNeoDriver.ColorOrder.GRB);  //RGB for fairy string
    }

    @Override
    public void preInit() {
        robot.initialize();
        if (useIMU) imuMgr.initialize();
        positionMgr.initialize();
        dsShooter.initialize();
        if (useSlamra) slamra.initialize();
        if (useAprilTag) dsApriltag.initialize();
        dsLed.initialize();
    }

    @Override
    public void initLoop() {
        buttonMgr.initLoop();
        if (useIMU) imuMgr.initLoop();
        if (useSlamra) slamra.initLoop();
        if (useAprilTag) dsApriltag.initLoop();
        positionMgr.initLoop();
        dsLed.initLoop();
        dsShooter.initLoop();
        TelemetryMgr.Update();
    }

    @Override
    public void preRun() {
        drivetrain.initialize();
        if (useIMU) imuMgr.preRun();
        if (useODO) odometry.initialize();
        userDrive.initialize();
        autoDrive.initialize();
        autoDrive.setNavTarget(new NavigationTarget(DSMisc.tagReadPos, dsMisc.toleranceHigh));

        if (useODO) odometry.runLoop();  // get some things squared away before the regular runLoops start
        autoDrive.runLoop();

        if (useSlamra) slamra.preRun();
        dsLed.preRun();
        dsShooter.preRun();
    }

    @Override
    public void runLoop() {
        addTelemetryLoopStart();

        robot.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        if (useSlamra) slamra.runLoop();
        if (useODO) odometry.runLoop();   // run odometry after IMU and slamra so it has up to date headings available
        if (useAprilTag) dsApriltag.runLoop();
        positionMgr.runLoop();
        controls.runLoop();
        userDrive.runLoop();
        dsSpeedControl.runLoop();
        autoDrive.runLoop();
        drivetrain.runLoop();
        dsShooter.runLoop();
        tagPositionAndLEDs();
        dsLed.runLoop();

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
        if (useAprilTag) dsApriltag.runLoop();
        positionMgr.runLoop();
        controls.runLoop();
//        userDrive.runLoop();
//        dsSpeedControl.runLoop();
        autoDrive.runLoop();
        drivetrain.runLoop();
        dsShooter.runLoop();
        tagPositionAndLEDs();
        //dsLed.runLoop();
        dsLed.autoRunLoop();

        addTelemetryLoopEnd();
        TelemetryMgr.Update();
    }

    @Override
    public void stop() {
        if (useSlamra) slamra.stop();
        if (useAprilTag) dsApriltag.stop();
        dsShooter.stop();
        drivetrain.stop();
    }

    public void tagPositionAndLEDs () {
        /* AprilTag experiment follows, to be moved elsewhere eventually? */   //todo:move this? (including firstlock variable)
        if (useAprilTag) {
            Position roboTagPosition = dsApriltag.getTagRobotPosition();
            if (roboTagPosition != null) {
                if (useSlamra) slamra.setupFieldOffset(roboTagPosition);
                if (useODO) odometry.setupFieldOffset(roboTagPosition);
                if (useIMU) imuMgr.setupFieldOffset(roboTagPosition);
            }
            if (dsApriltag.tagRobotPosition!=null){
                dsLed.updateGraphic('1', DSLed.MessageColor.G_GRUE);
                if (DSMisc.firstLock && !userDrive.isDriving) {   //todo:make this better
                    DSMisc.firstLock = false;
                    autoDrive.setNavTarget(new NavigationTarget(DSMisc.tagReadPos, dsMisc.toleranceHigh));
                    dsLed.displayMessage('#', DSLed.MessageColor.GREEN);
                }
            } else if (dsApriltag.instantTagRobotPosition!=null) {
                dsLed.updateGraphic('1', DSLed.MessageColor.G_ORANGE);
            } else {
                dsLed.updateGraphic('1', DSLed.MessageColor.G_RED);
            }
        }

        if (positionMgr.robotPosition!=null) {
            if (dsApriltag.strongLocked) {
                dsLed.updateGraphic('2', DSLed.MessageColor.G_GREEN);
            } else {
                dsLed.updateGraphic('2', DSLed.MessageColor.G_ORANGE);
            }
        } else {
            dsLed.updateGraphic('2', DSLed.MessageColor.G_RED);
            if (useAprilTag) dsApriltag.strongLocked=false;  // if all is lost, allow a weak lock again
        }
    }

    private void addTelemetryLoopStart() {
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "Loop time (ms)", JavaUtil.formatNumber(Functions.calculateLoopTime(), 0));
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "IMU raw heading", useIMU ? JavaUtil.formatNumber(imuMgr.returnImuHeadingRaw(),2) : "(not used)");
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "autoLaunchPos", DSMisc.autoLaunchPos.toString(2));
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
        if (useAprilTag) {
            Position robo = dsApriltag.getTagRobotPosition();
            if (robo != null)
                TelemetryMgr.message(TelemetryMgr.Category.APRILTAG, String.format("robotPos XYZ %6.1f %6.1f %6.1f  (inch, inch, deg)", robo.X, robo.Y, robo.R));
            else TelemetryMgr.message(TelemetryMgr.Category.APRILTAG, "robotpos - no tag position");
        }
    }
}
