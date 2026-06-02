package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.PartsInterfaceStatic;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

import java.util.List;


public class TB_LL implements PartsInterfaceStatic {

    public static Parts parts;
    static Limelight3A limelight;

    // public variables for configuration
    static int sizeOfBuffer = 50;                           // number of transforms for averaging & standard deviation
    static public double acceptableTx = 17.5;               // acceptable Tx (left/right) in degrees; edges less accurate
    static public Position acceptableStdDev = new Position(1,1,1);  // Std Dev in x inches, y inches, z degrees
    static public long manualIndicatorTimeout = 5000;       // ms to wait before turning off LED
    static public long automaticIndicatorTimeout = 500;     // ms to wait before turning off LED
    static public boolean automaticTransform = false;       // apply the transform automatically? Otherwise manual
//    static public String ledServoName = "servo5";          // name of the servo for the LED indicator

    // internal variables
    static final Position zero = new Position(0,0,0);
    static int bufferPointer = 0;
    static boolean transformValid = false;                  // Flips to true once buffer has filled
    static public boolean stdDevValid = false;              // True when the standard deviation is acceptable
    static Position[] transformBuffer = new Position[sizeOfBuffer];     // Holds a buffer of transforms for smoothing
    static Position llPosition = new Position();              // Holds the robot position reported by the limelight
    static Position llLastPosition = new Position();          // Holds the last valid position (to verify changing)
    static int llStuck = 0;                                 // Counter for determining if the position is changing
    static Position llSmoothTransform = new Position();       // Holds averaged transform
    static Position llStandardDeviation = new Position();     // Holds standard deviation of buffer
    static public Position llSavedTransform = new Position(); // Holds a transform once requested by driver
    static Position llLastValidTransform = new Position();    // Holds the last valid transform
    static double llLastValidTransformTime;                 // Time for tracking LED behavior
    static public Position llFusedPosition = new Position();  // Holds a transformed position
    static double llTimeStamp = 0;                          //
    static double llLastTimeStamp = 0;                      //
    static boolean isStuck = false;                         //
//    static Servo ledIndicator;

    // classificationId Defaults to 21.
    // Valid values are 21(GPP), 22(PGP), 23(PPG).
    private static Integer classificationId = 21;

//    /* Constructor */
//    public TB_LL(Parts parts){
//        construct(parts);
//    }
//
//    void construct(Parts parts){
//        this.parts = parts;
//    }

    public static void setup(Parts p) {
        parts = p;
    }

//    @Override
    public static void onRun() {
        LLStatus status = limelight.getStatus();
//        parent.opMode.telemetry.addData("Name", "%s", status.getName());
//        parent.opMode.telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        parent.opMode.telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
//            Pose3D botpose = result.getBotpose();
//            parent.opMode.telemetry.addData("tx", result.getTx());
//            parent.opMode.telemetry.addData("txnc", result.getTxNC());
//            parent.opMode.telemetry.addData("ty", result.getTy());
//            parent.opMode.telemetry.addData("tync", result.getTyNC());
//            parent.opMode.telemetry.addData("Botpose", botpose.toString());
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                parent.opMode.telemetry.addData("April Tag", "ID: %d", fr.getFiducialId());
                TelemetryMgr.message(TelemetryMgr.Category.LL, "April Tag", "ID: %d", fr.getFiducialId());
                int id = fr.getFiducialId();
                if (id == 21 || id == 22 || id == 23) {
                    classificationId = id;
                }
            }
            // Access color results
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            for (LLResultTypes.ColorResult cr : colorResults) {
//                parent.opMode.telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//            }
        } else {
            TelemetryMgr.message(TelemetryMgr.Category.LL, "Limelight", "No data available");
        }

        positionTransformLoop(result);
    }

    public Integer getClassificationId() {
        return classificationId;
    }

//    @Override
    public static void initialize() {
        limelight = parts.robotV2.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
//        if (ledServoName != null && !ledServoName.isEmpty())
//            ledIndicator = new ServoSSR(parts.robotV2.getServoByName(ledServoName));
    }


    public static void preInit() {
    }

    public static void initLoop() {
    }

    public static void preRun() {
    }

    public static void runLoop() {
        onRun();
    }

    public static void stop() {
        limelight.stop();
    }

    static void positionTransformLoop (LLResult llResult) {

        stdDevValid = false;

        // Clear the LED indicator if timeout has passed
        if (llLastValidTransformTime != 0 && System.currentTimeMillis()-llLastValidTransformTime >
                (automaticTransform ? automaticIndicatorTimeout : manualIndicatorTimeout)) {
            llLastValidTransformTime = 0;
            setLedIndicator(rgbIndicatorColor.Off);
        }

        // Calculate the fused position using the odo position and saved transform
        Position currentPos = parts.positionMgr.beforeOverride;
        if (currentPos != null && llSavedTransform != null) {
            llFusedPosition = llSavedTransform.transformPosition(currentPos);
//            DecodeSettings.storeFusedPosition(llFusedPosition);
            TelemetryMgr.message(TelemetryMgr.Category.LL, "Fused", llFusedPosition.toString());
        }

        // Verify that the results are updating (not stuck) and abort if not;
        // this deals with LL disconnects where the result stops updating
        llTimeStamp = llResult.getTimestamp();
        TelemetryMgr.message(TelemetryMgr.Category.LL, "LLTIME", llTimeStamp);
        if (llTimeStamp != 0 && llTimeStamp == llLastTimeStamp) {
            llStuck++;
            if (llStuck > 5) {
                isStuck = true;
                setLedIndicator(rgbIndicatorColor.Orange);
                TelemetryMgr.message(TelemetryMgr.Category.LL, "LLPOS", "Stuck / Disconnect");
                finalTelemetry();
                return;
            }
        }
        else if (isStuck) {
            llStuck = 0;
            isStuck = false;
            ////setLedIndicator(rgbIndicatorColor.Off);
            // The above was causing "disco lights" strobing because apparently the limelight on the
            // real bot isn't updating as often as on the test bot?
            // The following hack uses the led clear process above to delay turning off the LED.
            llLastValidTransformTime = System.currentTimeMillis() + 350 -
                    (automaticTransform ? automaticIndicatorTimeout : manualIndicatorTimeout);
        }
        else {
            llStuck = 0;
        }
        llLastTimeStamp = llTimeStamp;

        if (llResult != null && llResult.isValid() && llStuck == 0) {

            // Get the robot position as calculated by MegaTag in the LL
            llPosition = new Position(
                    llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).x,
                    llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).y,
                    llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));


            // Calculate an "offset" for the tag in the image for the purpose of ignoring
            // positions that are less accurate (e.g., off to the edges of the video frame)
            Position llOffset = new Position(
                    llResult.getTx(),  // How far left or right the target is (degrees)
                    llResult.getTy(),  // How far up or down the target is (degrees)
                    llResult.getTa()); // How big the target looks (0%-100% of the image)

            TelemetryMgr.message(TelemetryMgr.Category.LL, "LLPOS", llPosition.toString());

            // Ignore zero position such as when viewing the obelisk (or other reasons?)
            if (llPosition.isEqualTo(zero)) {finalTelemetry(); return;}

            // Don't use the position if the offset is unacceptable.
            // For now, this is based on Tx (left/right), but could add other parameters
            if (Math.abs(llOffset.X) > acceptableTx) {finalTelemetry(); return;}

            // Calculate a transformation Vector3 for rotating the odometry position to the
            // Limelight position. This is a "transformation of coordinates" to rotate
            // xy-Cartesian positions. The odometry position will be rotated by this position
            // to match the Limelight position.
            if (currentPos == null) {finalTelemetry(); return;}
            Position llTransform = currentPos.getOffset(llPosition);

            // Add that transform to the buffer array for averaging (to smooth out noisy data)
            // and to calculate a standard deviation to determine if the buffer is good/stable.
            addTransformToArray(llTransform);
            if (!transformValid) {finalTelemetry(); return;}  // if the buffer isn't filled yet, leave
            llSmoothTransform = getAverageTransform(transformBuffer);
            llStandardDeviation = getStandardDeviation(llSmoothTransform, transformBuffer);

            // If the standard deviation is acceptable, store the transform (and apply it if auto is set)
            stdDevValid = llStandardDeviation != null &&
                    Math.abs(llStandardDeviation.X) <= acceptableStdDev.X &&
                    Math.abs(llStandardDeviation.Y) <= acceptableStdDev.Y &&
                    Math.abs(llStandardDeviation.R) <= acceptableStdDev.R;
            if (stdDevValid) {
                llLastValidTransform = llSmoothTransform.clone();
                llLastValidTransformTime = System.currentTimeMillis();
                if (automaticTransform) {
                    setLedIndicator(rgbIndicatorColor.Violet);
                    applyTransform();
                }
                else {
                    setLedIndicator(rgbIndicatorColor.Azure);
                }
            }

            finalTelemetry();

        }
        else {
            // if there isn't a valid result, keep the telemetry consistent
            TelemetryMgr.message(TelemetryMgr.Category.LL, "LLPOS", "n/a");
            finalTelemetry();
        }
    }

    static void finalTelemetry () {
        // Used to keep the telemetry consistent each loop (not bouncing)
        TelemetryMgr.message(TelemetryMgr.Category.LL, "Accpt", acceptableStdDev.toString());
        TelemetryMgr.message(TelemetryMgr.Category.LL, "StDev", llStandardDeviation.toString());
        TelemetryMgr.message(TelemetryMgr.Category.LL, "Trans", llSavedTransform.toString());
        TelemetryMgr.message(TelemetryMgr.Category.LL, "LastV", llLastValidTransform.toString());

    }

    static void setLedIndicator(rgbIndicatorColor color) {
        // Update the color of the LED if it exists
//        if (ledIndicator != null) ledIndicator.setPosition(color.color);
        TB_Turret.LEDSettingLL[0] = color.color;
    }

    static public void toggleAuto() {
        // With auto set, the last valid transform will always update the saved transform
        // With auto not set (i.e., manual), the user must manually update the transform by calling applyTransform()
        automaticTransform = !automaticTransform;
    }
    static public void setAuto(boolean auto) {
        automaticTransform = auto;
    }

    static public void setSizeOfBuffer(int size) {
        // Allows the user to change the size of the buffer array for averaging and standard deviation
        if (size < 2 || size > 100) return;
        sizeOfBuffer = size;
        transformBuffer = new Position[sizeOfBuffer];
        bufferPointer = 0;
        transformValid = false;
    }

    static public void applyTransform() {
        // Updates the saved transform to the last valid transform
        llSavedTransform = llLastValidTransform.clone();
        parts.positionMgr.overrideTransform = llSavedTransform;
        if (!automaticTransform) {
            setLedIndicator(rgbIndicatorColor.Off);
            llLastValidTransformTime = 0;
        }
    }

//    public void applyTransformIfCurrent() {
//        if (stdDevValid) llSavedTransform = llLastValidTransform.copy();
//    }

    static public Position getFusedPosition() {
        return llFusedPosition;
    }

    static void addTransformToArray (Position transform) {
        // Adds transform to the buffer array and increments the pointer
        if (transform == null) return;  //never add null values to the array
        transformBuffer[bufferPointer] = transform;
        bufferPointer++;
        if (bufferPointer >= transformBuffer.length) {
            bufferPointer = 0;
            transformValid = true;  // once the buffer is full, calculations can be performed on it
        }
    }

    static Position getAverageTransform(Position[] buffer) {
        // Calculate the smoothed transform (add, then divide)
        if (!transformValid) return null;
        double x = 0, y = 0, z = 0;
        for (Position vector3 : buffer) {
            x += vector3.X;
            y += vector3.Y;
            z += vector3.R;
        }
        return new Position(x / buffer.length, y / buffer.length, z / buffer.length);
    }

    static Position getStandardDeviation(Position average, Position[] buffer) {
        // Calculate the standard deviation
        if (!transformValid || average == null) return null;
        double x = 0, y = 0, z = 0;
        for (Position vector3 : buffer) {
            if (vector3 == null) continue;
            x += Math.pow(vector3.X - average.X, 2);
            y += Math.pow(vector3.Y - average.Y, 2);
            z += Math.pow(vector3.R - average.R, 2);
        }
        x = Math.sqrt(x / buffer.length);
        y = Math.sqrt(y / buffer.length);
        z = Math.sqrt(z / buffer.length);
        return new Position(x, y, z);
    }

    public enum rgbIndicatorColor {
        Off (0.0),
        Red (0.279),
        Orange (0.333),
        Yellow (0.388),
        Sage (0.444),
        Green (0.500),
        Azure (0.555),
        Blue (0.611),
        Indigo (0.666),
        Violet (0.715), //(0.722),
        White (1.0);

        private final double color;

        rgbIndicatorColor(double color) {
            this.color = color;
        }
    }

}
