package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

import java.util.List;

public class RobotV2 implements PartsInterface {
    /* Public OpMode members. */

    public DcMotorEx[] motorArray;
    public final String[] motorName = {
            "motor0", "motor1", "motor2", "motor3",
            "motor0B", "motor1B", "motor2B", "motor3B"
    };

    public Servo[] servoArray;
    public final String[] servoName = {
            "servo0", "servo1", "servo2", "servo3", "servo4", "servo5",
            "servo0B", "servo1B", "servo2B", "servo3B", "servo4B", "servo5B"
    };

    public DigitalChannel[] digitalArray;
    public final String[] digitalName = {
            "digital0", "digital1", "digital2", "digital3", "digital4", "digital5", "digital6", "digital7",
            "digital0B", "digital1B", "digital2B", "digital3B", "digital4B", "digital5B", "digital6B", "digital7B"
    };

    public AnalogInput[] analogArray;
    public final String[] analogName = {
            "analog0", "analog1", "analog2", "analog3",
            "analog0B", "analog1B", "analog2B", "analog3B"
    };

    // todo: May want to divvy up robot-unique variables like done in Parts class?
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

    // Bulk Reads - Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
    List<LynxModule> allHubs = null;  // gets populated in construct()

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
        // Bulk Reads - Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        allHubs = hardwareMap.getAll(LynxModule.class);
    }

    void construct(LinearOpMode opMode){
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        // Bulk Reads - Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        allHubs = hardwareMap.getAll(LynxModule.class);
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

        settingOptions();

        // Bulk Reads - Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        // Bulk Reads - Important Step 3: Option B. Set all Expansion hubs to use the MANUAL Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        initMotors();
        initServos();
        initDigital();
        initAnalog();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "sensorIMU".

        // todo: This can probably be moved to ImuMgr
        // Initialize IMU directly
        sensorIMU = hardwareMap.get(IMU.class, "imu");
        sensorIMU.initialize(
            new IMU.Parameters(
                hubOrientation
            )
        );

        initOptions();
    }

    public void initMotors() {
        DcMotorEx.RunMode runmode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        for (int i = 0; i < motorName.length; i++) {
            motorArray[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
            motorArray[i].setDirection(DcMotorEx.Direction.FORWARD);
            motorArray[i].setPower(0);
            motorArray[i].setMode(runmode);
        }
    }

    public void initServos() {
        for (int i = 0; i < servoName.length; i++) {
            servoArray[i] = hardwareMap.get(Servo.class, servoName[i]);
        }
    }

    public void initDigital() {
        for (int i = 0; i < digitalName.length; i++) {
            digitalArray[i] = hardwareMap.get(DigitalChannel.class, digitalName[i]);
            digitalArray[i].setMode(DigitalChannel.Mode.INPUT);
        }
    }

    public void initAnalog() {
        for (int i = 0; i < analogName.length; i++) {
            analogArray[i] = hardwareMap.get(AnalogInput.class, analogName[i]);
        }
    }

    public void settingOptions() {
        hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

    public void initOptions() {
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
