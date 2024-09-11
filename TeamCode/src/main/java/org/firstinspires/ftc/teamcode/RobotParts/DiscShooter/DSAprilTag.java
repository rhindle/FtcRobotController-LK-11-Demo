package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class DSAprilTag implements PartsInterface {

    Parts parts;

    private static final boolean USE_WEBCAM = true;  //false;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public Position instantTagRobotPosition;
    public Position tagRobotPosition;
    Position[] lastPositions = new Position[10];
    int lastPositionPointer = 0;
    public Position acceptableStDev = new Position(0.2, 0.2, 0.2);
    public boolean strongLocked = false;
    public double strongLockMaxAngle = 10.0;     // in degrees
    public double strongLockMaxDistance = 48.0;  // in inches
    public Position camOffset = new Position(-8.2, -1.5, 0); //new Position(-1, 3, 0);

    /* Constructor */
    public DSAprilTag(Parts parts) {
        construct(parts);
    }

    void construct(Parts parts) {
        this.parts = parts;
    }

    public void initialize() {
        lkAprilTag();
        for (int i = 0; i < lastPositions.length; i++) {
            lastPositions[i] = new Position();
        }
    }

    public void preInit() {
    }

    public void initLoop() {
        updateAprilTag();
    }

    public void preRun() {
    }

    public void runLoop() {
        updateAprilTag();
    }

    public void stop() {
        visionPortal.close();
    }

    public void enableStreaming(boolean streamBoo) {
        if (streamBoo) {
            visionPortal.resumeStreaming();
        } else {
            visionPortal.stopStreaming();
        }
    }

    private void lkAprilTag() {
        AprilTagMetadata myAprilTagMetadata; //, myAprilTagMetadata2;
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;

        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCenterStageTagLibrary());

        //myAprilTagMetadata = new AprilTagMetadata(20, "LAK 36h11 ID20", 5, DistanceUnit.INCH);
        // since we didn't provide a VectorF fieldPosition or Quaternion fieldOrientation, this will be at origin (0,0,0)
        //todo: update metadata to match the field position (and fix the quaternion)
        // see: https://www.andre-gaschler.com/rotationconverter/
        // archived at: http://www.ftc14273.org/rotationconverter/
        myAprilTagMetadata = new AprilTagMetadata(20,
                "LAK 36h11 ID20",
                5,
                new VectorF(0f, 0f, 6.25f),
                DistanceUnit.INCH,
                Quaternion.identityQuaternion());

        myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);

        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        aprilTag = myAprilTagProcessorBuilder.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(parts.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    @SuppressLint("DefaultLocale")
    private void updateAprilTag() {

        instantTagRobotPosition = null;
        tagRobotPosition = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        TelemetryMgr.message(Category.APRILTAG, "# AprilTags Detected", currentDetections.size());

        // Add "key" information to telemetry
        TelemetryMgr.message(Category.APRILTAG_EXT, "XYZ = X (Right), Y (Forward), Z (Up) dist.");
        TelemetryMgr.message(Category.APRILTAG_EXT, "PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        TelemetryMgr.message(Category.APRILTAG_EXT, "RBE = Range, Bearing & Elevation");

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                TelemetryMgr.message(Category.APRILTAG, String.format("=== (ID %d) %s", detection.id, detection.metadata.name));
                TelemetryMgr.message(Category.APRILTAG, String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                TelemetryMgr.message(Category.APRILTAG, String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                TelemetryMgr.message(Category.APRILTAG_EXT, String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                // todo: clean up AprilTag work
                // raw camera values (ftcPose in it's native coordinate system) of XY
                Position camRaw = new Position(detection.ftcPose.x, detection.ftcPose.y, 0);
                TelemetryMgr.message(Category.APRILTAG_EXT, String.format("camRaw   XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", camRaw.X, camRaw.Y, camRaw.R));

                // transform the camera raw position using the yaw to align with field
                Position camTrans = transPos(new Position(0, 0, -detection.ftcPose.yaw), camRaw);
                TelemetryMgr.message(Category.APRILTAG_EXT, String.format("camTrans XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", camTrans.X, camTrans.Y, camTrans.R));

                // rotate the camera XY 90deg to match the field by switching axes
                Position camRot = new Position(-camTrans.Y, camTrans.X, camTrans.R);
                TelemetryMgr.message(Category.APRILTAG_EXT, String.format("camRot   XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", camRot.X, camRot.Y, camRot.R));

                // Do everything in one step: Switch ftcPose to field XY and transform by yaw to align with field
                Position camTry2 = transPos(new Position(0, 0, -detection.ftcPose.yaw),
                        new Position(-detection.ftcPose.y, detection.ftcPose.x, 0));
                TelemetryMgr.message(Category.APRILTAG, String.format("camTry2  XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", camTry2.X, camTry2.Y, camTry2.R));

                // Switch ftcPose to field XY relative to tag, add robot offset, and transform by yaw to align with field
                Position camPos = transPos(new Position(0, 0, -detection.ftcPose.yaw),
                        new Position(-detection.ftcPose.y + camOffset.X, detection.ftcPose.x + camOffset.Y, 0));
                TelemetryMgr.message(Category.APRILTAG, String.format("camPos   XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", camPos.X, camPos.Y, camPos.R));

                // Get the tag's field position
                // the library has bad x values!  60.3, but in reality it's 63.5.  So let's add 3.2 inches.
                double adjustment = 0; //3.2;
                float[] fieldPos = detection.metadata.fieldPosition.getData();
                TelemetryMgr.message(Category.APRILTAG_EXT, String.format("field XYR %6.1f %6.1f %6.1f  (inch)", fieldPos[0], fieldPos[1], fieldPos[2]));
                Position tagPos = new Position(detection.metadata.fieldPosition.get(0) + adjustment, detection.metadata.fieldPosition.get(1), 0);
                TelemetryMgr.message(Category.APRILTAG_EXT, String.format("tagPos   XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", tagPos.X, tagPos.Y, tagPos.R));

                // Calculate the robot position based on camera position and tag position
                Position robotPos = new Position(tagPos.X + camPos.X, tagPos.Y + camPos.Y, camPos.R);
                TelemetryMgr.message(Category.APRILTAG, String.format("robotPos XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", robotPos.X, robotPos.Y, robotPos.R));
                instantTagRobotPosition = robotPos;

            } else {
                TelemetryMgr.message(Category.APRILTAG, String.format("\n==== (ID %d) Unknown", detection.id));
                TelemetryMgr.message(Category.APRILTAG, String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            processTagPosition();
        }   // end for() loop

        if (currentDetections.size() == 0) {
            for (int j = 0; j < 8; j++) {
                TelemetryMgr.message(Category.APRILTAG, "X");
            }
            for (int j = 0; j < 6; j++) {
                TelemetryMgr.message(Category.APRILTAG_EXT, "X");
            }
        }

    }   // end method telemetryAprilTag()

    @SuppressLint("DefaultLocale")
    void processTagPosition() {
        int length = lastPositions.length;

        lastPositions[lastPositionPointer] = instantTagRobotPosition;
        lastPositionPointer++;
        if (lastPositionPointer >= length) lastPositionPointer = 0;

        // get the mean of array
        Position meanPosition = new Position();
        for (Position i : lastPositions) {
            if (i == null) i = new Position();
            meanPosition.X += i.X / length;
            meanPosition.Y += i.Y / length;
            meanPosition.R += i.R / length;
        }
        TelemetryMgr.message(Category.APRILTAG, String.format("meanPos XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", meanPosition.X, meanPosition.Y, meanPosition.R));

        // calculate the standard deviation
        Position stdevPosition = new Position();
        for (Position i : lastPositions) {
            if (i == null)
                i = new Position();   // got a null reference after running for a while; should track this down
            stdevPosition.X += Math.pow(i.X - meanPosition.X, 2);
            stdevPosition.Y += Math.pow(i.Y - meanPosition.Y, 2);
            stdevPosition.R += Math.pow(i.R - meanPosition.R, 2);
        }
        stdevPosition.X = Math.sqrt(stdevPosition.X / length);
        stdevPosition.Y = Math.sqrt(stdevPosition.Y / length);
        stdevPosition.R = Math.sqrt(stdevPosition.R / length);
        TelemetryMgr.message(Category.APRILTAG, String.format("stdvPos XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", stdevPosition.X, stdevPosition.Y, stdevPosition.R));

        if (stdevPosition.X <= acceptableStDev.X && stdevPosition.Y <= acceptableStDev.Y && stdevPosition.R <= acceptableStDev.R) {
//            if (Math.abs(meanPosition.R) <= strongLockMaxAngle) {
            if (Math.abs(meanPosition.R) <= strongLockMaxAngle && Math.abs(meanPosition.X) <= strongLockMaxDistance) {   //todo:embetter this
                strongLocked = true;
                tagRobotPosition = meanPosition;
            }
            if (!strongLocked) {
                // until the first strong lock, update anyway
                // may want to move the robot automatically to a better position for a stronger lock
                tagRobotPosition = meanPosition;
            }
        }
    }

    public Position getTagRobotPosition() {
        return tagRobotPosition;
    }

    Position transPos(Position pos1, Position pos2) {
        return new Position(
                (pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R)))),
                (pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R)))),
                (pos1.R + pos2.R)
        );
    }
}
