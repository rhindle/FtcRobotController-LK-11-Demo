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

import java.util.List;

public class Robot implements PartsInterface {
    /* Public OpMode members. */

    public DcMotorEx    motor0   = null;
    public DcMotorEx    motor1   = null;
    public DcMotorEx    motor2   = null;
    public DcMotorEx    motor3   = null;
    public DcMotorEx    motor0B   = null;
    public DcMotorEx    motor1B   = null;
    public DcMotorEx    motor2B   = null;
    public DcMotorEx    motor3B   = null;

    public Servo    servo0   = null;
    public Servo    servo1   = null;
    public Servo    servo2   = null;
    public Servo    servo3   = null;
    public Servo    servo4   = null;
    public Servo    servo5   = null;
    public Servo    servo0B   = null;
    public Servo    servo1B   = null;
    public Servo    servo2B   = null;
    public Servo    servo3B   = null;
    public Servo    servo4B   = null;
    public Servo    servo5B   = null;

    // todo: May want to divvy up robot-unique variables like done in Parts class?
    public NormalizedColorSensor sensorColor    = null;
    public DistanceSensor sensor2MLeft = null;
    public DistanceSensor sensor2MMiddle = null;
    public DistanceSensor sensor2MRight = null;
    public DFR304Range ultraSensor = null;
    public QwiicLEDStickLK qled = null;

    public DigitalChannel   digital0 = null;
    public DigitalChannel   digital1 = null;
    public DigitalChannel   digital2 = null;
    public DigitalChannel   digital3 = null;
    public DigitalChannel   digital4 = null;
    public DigitalChannel   digital5 = null;
    public DigitalChannel   digital6 = null;
    public DigitalChannel   digital7 = null;
    public DigitalChannel   digital0B = null;
    public DigitalChannel   digital1B = null;
    public DigitalChannel   digital2B = null;
    public DigitalChannel   digital3B = null;
    public DigitalChannel   digital4B = null;
    public DigitalChannel   digital5B = null;
    public DigitalChannel   digital6B = null;
    public DigitalChannel   digital7B = null;
    // Add B here as needed

    public AnalogInput  analog0 = null;
    public AnalogInput  analog1 = null;
    public AnalogInput  analog2 = null;
    public AnalogInput  analog3 = null;
    public AnalogInput  analog0B = null;
    public AnalogInput  analog1B = null;
    public AnalogInput  analog2B = null;
    public AnalogInput  analog3B = null;
    // Add B here as needed

//    public BNO055IMU sensorIMU      = null;
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
    public Robot(Parts parts){
        construct(parts);
    }

    void construct(Parts parts){
        this.opMode = parts.opMode;
        this.hardwareMap = parts.opMode.hardwareMap;
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

        //        sensorIMU = hardwareMap.get(BNO055IMU.class, "imu");
        //        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        //        sensorIMU.initialize(parameters);

        initOptions();
    }

    public void initMotors() {
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor0B = hardwareMap.get(DcMotorEx.class, "motor0B");
        motor1B = hardwareMap.get(DcMotorEx.class, "motor1B");
        motor2B = hardwareMap.get(DcMotorEx.class, "motor2B");
        motor3B = hardwareMap.get(DcMotorEx.class, "motor3B");

        motor0.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor1.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor2.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor3.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor0B.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor1B.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor2B.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors
        motor3B.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using spur gear motors

        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor0B.setPower(0);
        motor1B.setPower(0);
        motor2B.setPower(0);
        motor3B.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        DcMotorEx.RunMode runmode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        motor0.setMode(runmode);
        motor1.setMode(runmode);
        motor2.setMode(runmode);
        motor3.setMode(runmode);
        motor0B.setMode(runmode);
        motor1B.setMode(runmode);
        motor2B.setMode(runmode);
        motor3B.setMode(runmode);
    }

    public void initServos() {
        // Define and initialize ALL installed servos.
        servo0 = hardwareMap.get(Servo.class,"servo0");
        servo1 = hardwareMap.get(Servo.class,"servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        servo3 = hardwareMap.get(Servo.class,"servo3");
        servo4 = hardwareMap.get(Servo.class,"servo4");
        servo5 = hardwareMap.get(Servo.class,"servo5");
        servo0B = hardwareMap.get(Servo.class,"servo0B");
        servo1B = hardwareMap.get(Servo.class,"servo1B");
        servo2B = hardwareMap.get(Servo.class,"servo2B");
        servo3B = hardwareMap.get(Servo.class,"servo3B");
        servo4B = hardwareMap.get(Servo.class,"servo4B");
        servo5B = hardwareMap.get(Servo.class,"servo5B");
    }

    public void initDigital() {
        digital0 = hardwareMap.get(DigitalChannel.class, "digital0");
        digital1 = hardwareMap.get(DigitalChannel.class, "digital1");
        digital2 = hardwareMap.get(DigitalChannel.class, "digital2");
        digital3 = hardwareMap.get(DigitalChannel.class, "digital3");
        digital4 = hardwareMap.get(DigitalChannel.class, "digital4");
        digital5 = hardwareMap.get(DigitalChannel.class, "digital5");
        digital6 = hardwareMap.get(DigitalChannel.class, "digital6");
        digital7 = hardwareMap.get(DigitalChannel.class, "digital7");
        digital0B = hardwareMap.get(DigitalChannel.class, "digital0B");
        digital1B = hardwareMap.get(DigitalChannel.class, "digital1B");
        digital2B = hardwareMap.get(DigitalChannel.class, "digital2B");
        digital3B = hardwareMap.get(DigitalChannel.class, "digital3B");
        digital4B = hardwareMap.get(DigitalChannel.class, "digital4B");
        digital5B = hardwareMap.get(DigitalChannel.class, "digital5B");
        digital6B = hardwareMap.get(DigitalChannel.class, "digital6B");
        digital7B = hardwareMap.get(DigitalChannel.class, "digital7B");

        digital0.setMode(DigitalChannel.Mode.INPUT);
        digital1.setMode(DigitalChannel.Mode.INPUT);
        digital2.setMode(DigitalChannel.Mode.INPUT);
        digital3.setMode(DigitalChannel.Mode.INPUT);
        digital4.setMode(DigitalChannel.Mode.INPUT);
        digital5.setMode(DigitalChannel.Mode.INPUT);
        digital6.setMode(DigitalChannel.Mode.INPUT);
        digital7.setMode(DigitalChannel.Mode.INPUT);
        digital0B.setMode(DigitalChannel.Mode.INPUT);
        digital1B.setMode(DigitalChannel.Mode.INPUT);
        digital2B.setMode(DigitalChannel.Mode.INPUT);
        digital3B.setMode(DigitalChannel.Mode.INPUT);
        digital4B.setMode(DigitalChannel.Mode.INPUT);
        digital5B.setMode(DigitalChannel.Mode.INPUT);
        digital6B.setMode(DigitalChannel.Mode.INPUT);
        digital7B.setMode(DigitalChannel.Mode.INPUT);
    }

    public void initAnalog() {
        analog0 = hardwareMap.get(AnalogInput.class, "analog0");
        analog1 = hardwareMap.get(AnalogInput.class, "analog1");
        analog2 = hardwareMap.get(AnalogInput.class, "analog2");
        analog3 = hardwareMap.get(AnalogInput.class, "analog3");
        analog0B = hardwareMap.get(AnalogInput.class, "analog0B");
        analog1B = hardwareMap.get(AnalogInput.class, "analog1B");
        analog2B = hardwareMap.get(AnalogInput.class, "analog2B");
        analog3B = hardwareMap.get(AnalogInput.class, "analog3B");
    }

    public void settingOptions() {
        hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

//    public void disableServo(Servo servo) {
//        ServoControllerEx controller = (ServoControllerEx) servo.getController();
//        int servoPort = servo.getPortNumber();
//        controller.setServoPwmDisable(servoPort);
////        if (controller.isServoPwmEnabled(servoPort)) controller.setServoPwmDisable(servoPort);  //todo:check this; also I think 10.0 adds methods to Servo
////        else controller.setServoPwmEnable(servoPort);
//    }
//
//    public void enableServo(Servo servo) {
//        ServoControllerEx controller = (ServoControllerEx) servo.getController();
//        int servoPort = servo.getPortNumber();
//        controller.setServoPwmEnable(servoPort);
//    }

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
