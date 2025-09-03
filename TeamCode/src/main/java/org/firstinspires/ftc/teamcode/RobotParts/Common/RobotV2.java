package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.i2c.DFR304Range;
import org.firstinspires.ftc.teamcode.Tools.i2c.QwiicLEDStickLK;

import java.util.Arrays;
import java.util.List;

public class RobotV2 implements PartsInterface {
    /* Public OpMode members. */

    public String[] motorNames = {
            "motor0", "motor1", "motor2", "motor3",
            "motor0B", "motor1B", "motor2B", "motor3B"
    };
    public DcMotorEx[] motorArray;

    public String[] servoNames = {
            "servo0", "servo1", "servo2", "servo3", "servo4", "servo5",
            "servo0B", "servo1B", "servo2B", "servo3B", "servo4B", "servo5B"
    };
    public Servo[] servoArray;

    public String[] digitalNames = {
            "digital0", "digital1", "digital2", "digital3", "digital4", "digital5", "digital6", "digital7",
            "digital0B", "digital1B", "digital2B", "digital3B", "digital4B", "digital5B", "digital6B", "digital7B"
    };
    public DigitalChannel[] digitalArray;

    public String[] analogNames = {
            "analog0", "analog1", "analog2", "analog3",
            "analog0B", "analog1B", "analog2B", "analog3B"
    };
    public AnalogInput[] analogArray;

    // todo: May want to divvy up robot-unique variables like done in Parts class?
    // todo: Consider also moving sensors to a separate class, since they are more likely to vary by robot / opMode.
    public NormalizedColorSensor sensorColor    = null;
    public DistanceSensor sensor2MLeft = null;
    public DistanceSensor sensor2MMiddle = null;
    public DistanceSensor sensor2MRight = null;
    public DFR304Range ultraSensor = null;
    public QwiicLEDStickLK qled = null;

    public IMU sensorIMU        = null;
    public RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    List<LynxModule> allHubs = null;  // gets populated in init()

    /* local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();

    LinearOpMode opMode;
    public HardwareMap hardwareMap;

    /* Constructor */
    public RobotV2(Parts parts){
        construct(parts);
    }
    public RobotV2(LinearOpMode opMode){
        construct(opMode);
    }

    void construct(Parts parts){
        this.opMode = parts.opMode;
        this.hardwareMap = parts.opMode.hardwareMap;
    }
    void construct(LinearOpMode opMode){
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
    }

    public void initialize(){
        // initialization is complex and messy in this class, so move it elsewhere for readability
        init();
    }

    public void preInit() {
    }

    public void initLoop() {
    }

    public void preRun() {
    }

    public void runLoop() {
        // Bulk Reads - Important Step 4: If you are using MANUAL mode, you must clear the BulkCache once per control cycle
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    public void stop() {
    }

    /* Initialize standard Hardware interfaces */
    public void init() {

        initOptionsStart();

        motorArray = new DcMotorEx[motorNames.length];
        servoArray = new Servo[servoNames.length];
        digitalArray = new DigitalChannel[digitalNames.length];
        analogArray = new AnalogInput[analogNames.length];

        // Bulk Reads - Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        // Bulk Reads - Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        allHubs = hardwareMap.getAll(LynxModule.class);
        // Bulk Reads - Important Step 3: Option B. Set all Expansion hubs to use the MANUAL Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initMotors();
        initServos();
        initDigital();
        initAnalog();

        // todo: This can probably be moved to ImuMgr
        // Initialize IMU directly
        sensorIMU = hardwareMap.get(IMU.class, "imu");
        sensorIMU.initialize(
            new IMU.Parameters(
                hubOrientation
            )
        );

        initOptionsEnd();
    }

    public void initMotors() {
        DcMotorEx.RunMode runmode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        for (int i = 0; i < motorNames.length; i++) {
            motorArray[i] = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            motorArray[i].setPower(0);
            motorArray[i].setMode(runmode);
        }
    }

    public void initServos() {
        for (int i = 0; i < servoNames.length; i++) {
            servoArray[i] = hardwareMap.get(Servo.class, servoNames[i]);
        }
    }

    public void initDigital() {
        for (int i = 0; i < digitalNames.length; i++) {
            digitalArray[i] = hardwareMap.get(DigitalChannel.class, digitalNames[i]);
            digitalArray[i].setMode(DigitalChannel.Mode.INPUT);
        }
    }

    public void initAnalog() {
        for (int i = 0; i < analogNames.length; i++) {
            analogArray[i] = hardwareMap.get(AnalogInput.class, analogNames[i]);
        }
    }

    public Servo getServoByName(String name) {
        return servoArray[Arrays.asList(servoNames).indexOf(name)];
        //for (int i = 0; i < servoNames.length; i++) {
        //    if (servoNames[i].equals(name)) {
        //        return servoArray[i];
        //    }
        //}
        //return null;
    }

    public DcMotorEx getMotorByName(String name) {
        return motorArray[Arrays.asList(motorNames).indexOf(name)];
    }

    public DigitalChannel getDigitalByName(String name) {
        return digitalArray[Arrays.asList(digitalNames).indexOf(name)];
    }

    public AnalogInput getAnalogByName(String name) {
        return analogArray[Arrays.asList(analogNames).indexOf(name)];
    }

    public void initOptionsStart() {
        hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        // could customize motorNames, etc., arrays here if needed
    }

    public void initOptionsEnd() {
        /* Following left as examples for subclass @override method */

        //        ultraSensor = hardwareMap.get(DFR304Range.class, "uSensor");
        //        DFR304Range.Parameters uParameters = new DFR304Range.Parameters();
        //        uParameters.maxRange = DFR304Range.MaxRange.CM500;
        //        uParameters.measureMode = DFR304Range.MeasureMode.PASSIVE;
        //        ultraSensor.initialize(uParameters);

                //i2c sensors
        //        sensorColor = hwMap.get(ColorSensor.class, "sensorColorRange");
        //        sensorDistance = hwMap.get(DistanceSensor.class, "sensorColorRange");
        //        sensor2MLeft = hardwareMap.get(DistanceSensor.class, "2MdistL");
        //        sensor2MMiddle = hardwareMap.get(DistanceSensor.class, "2MdistM");
        //        sensor2MRight = hardwareMap.get(DistanceSensor.class, "2MdistR");
        //        qled = hardwareMap.get(QwiicLEDStick.class, "led");
    }
}
