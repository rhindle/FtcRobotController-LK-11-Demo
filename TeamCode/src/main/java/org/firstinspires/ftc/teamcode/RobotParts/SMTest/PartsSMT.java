package org.firstinspires.ftc.teamcode.RobotParts.SMTest;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.Functions;

public class PartsSMT extends Parts {
    public PartsSMT(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void setup(){
        // We do this after the construct because we may want to adjust some settings before creating all the sub-parts
        if (isSetup) {
            return;
        }
        isSetup = true;
        robot = new RobotSMT(this);
        buttonMgr = new ButtonMgr(opMode);
        controls = new ControlsSMT(this);

        if (useIMU) imuMgr = new ImuMgr(this);
        positionMgr = new PositionMgr(this);
        smt_Misc = new SMT_Misc(this);
        smt_LED = new SMT_LED(this);
    }

    @Override
    public void preInit() {
        robot.initialize();
        if (useIMU) imuMgr.initialize();
        positionMgr.initialize();
        smt_LED.initialize();
    }

    @Override
    public void initLoop() {
        buttonMgr.initLoop();
        if (useIMU) imuMgr.initLoop();
        positionMgr.initLoop();
        smt_LED.initLoop();
        TelemetryMgr.Update();
    }

    @Override
    public void preRun() {
        if (useIMU) imuMgr.preRun();
    }

    @Override
    public void runLoop() {
        addTelemetryLoopStart();

        robot.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        positionMgr.runLoop();
        controls.runLoop();
        StateMachine.runLoop();
        smt_LED.runLoop();

        addTelemetryLoopEnd();
        StateMachine.addTelemetry();
        TelemetryMgr.Update();
    }

    @Override
    public void autoRunLoop() {
        if (!opMode.opModeIsActive()) return;
        addTelemetryLoopStart();

        robot.runLoop();
        buttonMgr.runLoop();
        if (useIMU) imuMgr.runLoop();
        positionMgr.runLoop();
        controls.runLoop();
        smt_LED.runLoop();

        addTelemetryLoopEnd();
        StateMachine.addTelemetry();
        TelemetryMgr.Update();
    }

    @Override
    public void stop() {
        smt_LED.stop();
    }

    private void addTelemetryLoopStart() {
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "Loop time (ms)", JavaUtil.formatNumber(Functions.calculateLoopTime(), 0));
        TelemetryMgr.message(TelemetryMgr.Category.BASIC, "IMU raw heading", useIMU ? JavaUtil.formatNumber(imuMgr.returnImuHeadingRaw(),2) : "(not used)");
    }

    @SuppressLint("DefaultLocale")
    private void addTelemetryLoopEnd() {
        TelemetryMgr.message(TelemetryMgr.Category.IMU, "IMU-Modified", useIMU ? JavaUtil.formatNumber(imuMgr.returnImuRobotHeading(),2) : "(not used)");
        opMode.telemetry.addLine()
                .addData("Encoder 0", robot.motor0.getCurrentPosition())
                .addData("1", robot.motor1.getCurrentPosition())
                .addData("2", robot.motor2.getCurrentPosition())
                .addData("3", robot.motor3.getCurrentPosition());
    }
}
