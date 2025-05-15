package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DrivePowers;

public class Drivetrain {

    /* Public OpMode members. */
    public Parts parts;

    private DcMotorEx motorLF, motorRF, motorLR, motorRR;
    DrivePowers drivePowers, drivePowersLast;
    public boolean minimizeCycleTime = true;   // skip small power changes to improve cycle time (each motor power transaction degrades cycle time)
    public double ignoreDiff = .025;            // absolute power difference to ignore
    public boolean accelControl = true;
    public double accelControlRamp = 200;      // time allowed for transition from 0 to 1 power
    public long lastLoopTime = System.currentTimeMillis();
    public long currentLoopTime;

    /* Constructor */
    public Drivetrain(Parts parts) {
        construct(parts);
    }

    void construct(Parts parts) {
        this.parts = parts;
    }

    public void initialize() {
        initMotors();
        drivePowers = new DrivePowers();
        drivePowersLast = new DrivePowers();
    }

    public void preInit() {
    }

    public void initLoop() {
    }

    public void preRun() {
    }

    public void runLoop() {
        applyDrivePowers();
    }

    public void stop() {
        stopDriveMotors(true);
    }

    public void eStop() {
        stopDriveMotors(true);
    }

    public void applyDrivePowers() {
        applyDrivePowers(true);
    }

    public void applyDrivePowers(boolean addTelemetry) {
        currentLoopTime = System.currentTimeMillis();
        if (addTelemetry) TelemetryMgr.message(Category.DRIVETRAIN, "raw", drivePowers.toString(2));
        if (accelControl) drivePowers = applyAccelRateLimit(drivePowers, drivePowersLast);
        if (addTelemetry) TelemetryMgr.message(Category.DRIVETRAIN, "lim", drivePowers.toString(2));
        if (minimizeCycleTime) drivePowers = adjustPowers(drivePowers, drivePowersLast);
        if (addTelemetry) TelemetryMgr.message(Category.DRIVETRAIN, "adj", drivePowers.toString(2));
        motorLF.setPower(drivePowers.v0);
        motorRF.setPower(drivePowers.v1);
        motorLR.setPower(drivePowers.v2);
        motorRR.setPower(drivePowers.v3);
        drivePowersLast = drivePowers.clone();
        lastLoopTime = currentLoopTime;
    }

    public DrivePowers getDrivePowers() {
        return drivePowers;
    }

    // By design, drive powers variables can be changed multiple times (e.g., by AutoDrive and UserDrive),
    // but are only applied once per runLoop()
    public void setDrivePowers(DrivePowers drivePowers) {
        this.drivePowers = drivePowers;
    }

    public void setDrivePowers(double[] mPow) {
        setDrivePowers(new DrivePowers(mPow[0], mPow[1], mPow[2], mPow[3]));
    }

    public void setDrivePowers(double p0, double p1, double p2, double p3) {
        setDrivePowers(new DrivePowers(p0, p1, p2, p3));
    }

    public void stopDriveMotors() {
        setDrivePowers(0, 0, 0, 0);
    }

    public void stopDriveMotors(boolean immediate) {
        setDrivePowers(0, 0, 0, 0);
        drivePowersLast = drivePowers.clone();  // this is so it doesn't try to rate limit
        if (immediate) applyDrivePowers(false);
    }

    private DrivePowers adjustPowers(DrivePowers powerRequested, DrivePowers powerLast) {
        double[] newPow = {powerRequested.v0, powerRequested.v1, powerRequested.v2, powerRequested.v3};
        double[] oldPow = {powerLast.v0, powerLast.v1, powerLast.v2, powerLast.v3};
        double[] adjPow = new double[4];
        // only change motor power if it has changed more than absEff or is 0.
        for (int i = 0; i < 4; i++) {
            if (newPow[i] == 0) adjPow[i] = 0;
            else if (Math.abs(newPow[i] - oldPow[i]) < ignoreDiff) adjPow[i] = oldPow[i];
            else adjPow[i] = newPow[i];
        }
        return new DrivePowers(adjPow);
    }

    private DrivePowers applyAccelRateLimit(DrivePowers powerRequested, DrivePowers powerLast) {
        // This is a simple linear substitute for motion profiling, which might be a good idea to add at a later date
        // The value of this as written is dubious; it does not maintain direction and rotation of the robot
        double[] newPow = {powerRequested.v0, powerRequested.v1, powerRequested.v2, powerRequested.v3};
        double[] oldPow = {powerLast.v0, powerLast.v1, powerLast.v2, powerLast.v3};
        double[] adjPow = new double[4];
        double rateLimit = Math.min((currentLoopTime - lastLoopTime) / accelControlRamp, 1);
        for (int i = 0; i < 4; i++) {
            double change = newPow[i] - oldPow[i];
            if (Math.abs(change) > rateLimit) {
                change = rateLimit * Math.signum(change);
            }
            adjPow[i] = oldPow[i] + change;
        }
        return new DrivePowers(adjPow);
    }

    public void initMotors() {
        motorLF = parts.robot.motor0;
        motorRF = parts.robot.motor1;
        motorLR = parts.robot.motor2;
        motorRR = parts.robot.motor3;

        if (!parts.reverseDrive) {
            motorLF.setDirection(DcMotorEx.Direction.REVERSE);
            motorLR.setDirection(DcMotorEx.Direction.REVERSE);
            motorRF.setDirection(DcMotorEx.Direction.FORWARD);
            motorRR.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            motorLF.setDirection(DcMotorEx.Direction.FORWARD);
            motorLR.setDirection(DcMotorEx.Direction.FORWARD);
            motorRF.setDirection(DcMotorEx.Direction.REVERSE);
            motorRR.setDirection(DcMotorEx.Direction.REVERSE);
        }

        if (parts.useDrivetrainEncoders) {
            motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            motorLF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorLR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // This method removes the normal restriction on RPM when using RUN_USING_ENCODER
    public void removeEncoderSpeedLimits() {
        DcMotorEx[] motors = {motorLF, motorRF, motorLR, motorRR};
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }

    public int[] getDriveEncoderValues() {
        return new int[]{
            motorLF.getCurrentPosition(),
            motorRF.getCurrentPosition(),
            motorLR.getCurrentPosition(),
            motorRR.getCurrentPosition()
        };
    }
}
