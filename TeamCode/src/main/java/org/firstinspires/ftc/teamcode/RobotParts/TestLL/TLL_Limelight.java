package org.firstinspires.ftc.teamcode.RobotParts.TestLL;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class TLL_Limelight implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   Limelight3A limelight;
   ServoSSR rgb;
   rgbIndicatorColor setColor = rgbIndicatorColor.Off;

   int transformPointer = 0;
   final int transformNumber = 50;
   final double acceptableTx = 17.5;
   Position[] transformBuffer = new Position[transformNumber];
   boolean transformValid = false;
   Position llSmoothTransform;
   Position llStandardDeviation;
   final Position acceptableStdDev = new Position(1,1,1);
   boolean stdevValid = false;

   Position[] positionHistory = new Position[5];
   double[] positionHistoryTime = new double[positionHistory.length];
   long llLatency = 25;
   Position latentPosition = new Position();
   Position[] transformBufferLat = new Position[transformNumber];
   Position llSmoothTransformLat;
   Position llStandardDeviationLat;
   boolean stdevValidLat = false;
   Position odoPosition;

   /* Constructor */
   public TLL_Limelight(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){

      //see:  https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming

      limelight = parts.robotV2.hardwareMap.get(Limelight3A.class, "limelight");
      limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
      limelight.start(); // This tells Limelight to start looking!

      limelight.pipelineSwitch(0); // Switch to pipeline number 0

      rgb = new ServoSSR(parts.robotV2.getServoByName("servo0"));

      // make sure there's no nulls when calculating historical position
      for (int i = 0; i < positionHistory.length; i++) {
         positionHistory[i] = new Position();
      }
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      setColor = rgbIndicatorColor.Off;

      updatePositionHistory();

      LLResult result = limelight.getLatestResult();
//      if (result != null && result.isValid()) {
//         double tx = result.getTx(); // How far left or right the target is (degrees)
//         double ty = result.getTy(); // How far up or down the target is (degrees)
//         double ta = result.getTa(); // How big the target looks (0%-100% of the image)
//
//         parts.opMode.telemetry.addData("Target X", tx);
//         parts.opMode.telemetry.addData("Target Y", ty);
//         parts.opMode.telemetry.addData("Target Area", ta);
//      } else {
//         parts.opMode.telemetry.addData("Limelight", "No Targets");
//      }

      boolean telDone = false;
      if (result != null && result.isValid()) {
         Pose3D botpose = result.getBotpose();
         if (botpose != null) {
            double x = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
            double y = botpose.getPosition().toUnit(DistanceUnit.INCH).y;
            double z = botpose.getPosition().toUnit(DistanceUnit.INCH).z;
            double r = botpose.getOrientation().getYaw(AngleUnit.DEGREES);


//            parts.opMode.telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            TelemetryMgr.message(TelemetryMgr.Category.LL, "MT1 Location","("
                    + roundIt(x) + ", "
                    + roundIt(y) + ", "
                    + roundIt(z) + ", "
                    + roundIt(r) + ")");

            double[] stdDevs = result.getStddevMt1();
            double x_stddev = stdDevs[0] / 0.0254;  //google ai thinks it's in meters?
            double y_stddev = stdDevs[1] / 0.0254;
            double r_stddev = stdDevs[5];   // degrees?  radians?
            Position llStdDev = new Position(x_stddev,y_stddev,r_stddev);

            TelemetryMgr.message(TelemetryMgr.Category.LL, "MT1 StdDev", llStdDev.toString());

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            Position llOffset = new Position(tx,ty,ta);

            TelemetryMgr.message(TelemetryMgr.Category.LL, "Offset", llOffset.toString());

            Position llPosition = new Position(
                    result.getBotpose().getPosition().toUnit(DistanceUnit.INCH).x,
                    result.getBotpose().getPosition().toUnit(DistanceUnit.INCH).y,
                    result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));

            TelemetryMgr.message(TelemetryMgr.Category.LL, "LLPOS", llPosition.toString());

//            Position llTransform = llPosition.getOffset(parts.positionMgr.beforeOverride);
            Position llTransform = odoPosition.getOffset(llPosition);
            Position llTransformLat = latentPosition.getOffset(llPosition);

            TelemetryMgr.message(TelemetryMgr.Category.LL, "Transform", llTransform.toString());

            // if we consider data good, send it to the smoother

            if (Math.abs(llOffset.X) <= acceptableTx) {  // don't use the position if it's too far off center
               llSmoothTransform = transformSmoother(llTransform, transformBuffer);
               llSmoothTransformLat = transformSmoother(llTransform, transformBufferLat);

               setColor = rgbIndicatorColor.Yellow;
            }
            TelemetryMgr.message(TelemetryMgr.Category.LL, "Pointer", Integer.toString(transformPointer));
            TelemetryMgr.message(TelemetryMgr.Category.LL, "Smoothed", llSmoothTransform==null ? "null" : llSmoothTransform.toString());

            llStandardDeviation = getSmootherSTDEV(transformBuffer);
            llStandardDeviationLat = getSmootherSTDEV(transformBufferLat);
            if (llStandardDeviation != null) stdevValid = isStdevValid(llStandardDeviation);
            if (llStandardDeviationLat != null) stdevValidLat = isStdevValid(llStandardDeviationLat);

            TelemetryMgr.message(TelemetryMgr.Category.LL, "StdDev", llStandardDeviation==null ? "null" : llStandardDeviation.toString());

            //Position effectivePosition = llSmoothTransform==null ? parts.positionMgr.beforeOverride : parts.positionMgr.beforeOverride.transformPosition(llSmoothTransform);
            Position effectivePosition = llSmoothTransform==null ? odoPosition : llSmoothTransform.transformPosition(odoPosition);
            TelemetryMgr.message(TelemetryMgr.Category.LL, "Effective", effectivePosition.toString());

            TelemetryMgr.message(TelemetryMgr.Category.LL, "_Latent", latentPosition.toString());
            TelemetryMgr.message(TelemetryMgr.Category.LL, "_Transform", llTransformLat.toString());
            TelemetryMgr.message(TelemetryMgr.Category.LL, "_Smoothed", llSmoothTransformLat==null ? "null" : llSmoothTransformLat.toString());
            TelemetryMgr.message(TelemetryMgr.Category.LL, "_StdDev", llStandardDeviationLat==null ? "null" : llStandardDeviationLat.toString());
            Position effectivePositionLat = llSmoothTransformLat==null ? odoPosition : llSmoothTransformLat.transformPosition(odoPosition);
            TelemetryMgr.message(TelemetryMgr.Category.LL, "_Effective", effectivePositionLat.toString());

            telDone = true;
         }
      }

      if (!telDone) {
         TelemetryMgr.message(TelemetryMgr.Category.LL, "MT1 Location","n/a");
         TelemetryMgr.message(TelemetryMgr.Category.LL, "MT1 StdDev", "n/a");
         TelemetryMgr.message(TelemetryMgr.Category.LL, "Offset", "n/a");
         TelemetryMgr.message(TelemetryMgr.Category.LL, "LLPOS", "n/a");
         TelemetryMgr.message(TelemetryMgr.Category.LL, "Transform", "n/a");
         TelemetryMgr.message(TelemetryMgr.Category.LL, "Pointer", Integer.toString(transformPointer));
         TelemetryMgr.message(TelemetryMgr.Category.LL, "Smoothed", llSmoothTransform==null ? "null" : llSmoothTransform.toString());
         TelemetryMgr.message(TelemetryMgr.Category.LL, "StdDev", llStandardDeviation==null ? "null" : llStandardDeviation.toString());
//         Position effectivePosition = llSmoothTransform==null ? parts.positionMgr.beforeOverride : parts.positionMgr.beforeOverride.transformPosition(llSmoothTransform);
         Position effectivePosition = llSmoothTransform==null ? odoPosition : llSmoothTransform.transformPosition(odoPosition);
         TelemetryMgr.message(TelemetryMgr.Category.LL, "Effective", effectivePosition.toString());

         TelemetryMgr.message(TelemetryMgr.Category.LL, "_Latent", latentPosition.toString());
         TelemetryMgr.message(TelemetryMgr.Category.LL, "_Transform", "n/a");
         TelemetryMgr.message(TelemetryMgr.Category.LL, "_Smoothed", llSmoothTransformLat==null ? "null" : llSmoothTransformLat.toString());
         TelemetryMgr.message(TelemetryMgr.Category.LL, "_StdDev", llStandardDeviationLat==null ? "null" : llStandardDeviationLat.toString());
         Position effectivePositionLat = llSmoothTransformLat==null ? odoPosition : llSmoothTransformLat.transformPosition(odoPosition);
         TelemetryMgr.message(TelemetryMgr.Category.LL, "_Effective", effectivePositionLat.toString());
      }

      if (stdevValid) {
         setColor = rgbIndicatorColor.Green;
      }
      rgb.setPosition(setColor.color);

   }

   public void stop() {
   }

   public void updatePositionHistory() {
      odoPosition = parts.positionMgr.beforeOverride;

      for (int i = positionHistory.length - 1; i > 0; i--) {
         positionHistory[i] = positionHistory[i-1];
         positionHistoryTime[i] = positionHistoryTime[i-1];
      }

      positionHistory[0] = odoPosition;
      positionHistoryTime[0] = System.currentTimeMillis();

      //update position with latency
      double timeTarget = positionHistoryTime[0] - llLatency;
      int index;
      for (index = 0; index < positionHistory.length; index++) {
         if (positionHistoryTime[index] < timeTarget) {
            break;
         }
      }
      if (index==positionHistory.length) {
         index--;
      }
      latentPosition = new Position(
              Functions.interpolate(
                      timeTarget,
                      positionHistoryTime[index-1],
                      positionHistoryTime[index],
                      positionHistory[index-1].X,
                      positionHistory[index].X
              ),
              Functions.interpolate(
                      timeTarget,
                      positionHistoryTime[index-1],
                      positionHistoryTime[index],
                      positionHistory[index-1].Y,
                      positionHistory[index].Y
              ),
              Functions.interpolate(
                      timeTarget,
                      positionHistoryTime[index-1],
                      positionHistoryTime[index],
                      positionHistory[index-1].R,
                      positionHistory[index].R
              )
      );
   }

   // add a new transform to the array for smoothing
   public Position transformSmoother(Position transform, Position[] buffer) {
      if (transform != null) {  // don't add null values to the buffer array
         buffer[transformPointer] = transform;
         transformPointer++;
         if (transformPointer >= transformNumber) {
            transformPointer = 0;
            transformValid = true;  // valid once the buffer has been filled
         }
      }
      if (!transformValid) {
         return null;
      }
      // calculate the smoothed transform (add, then divide)
      Position transformSum = new Position();
      for (int i = 0; i < transformNumber; i++) {
         transformSum.add(buffer[i]);
      }
      transformSum.divide(transformNumber);
      return transformSum;
   }

   public Position getSmootherSTDEV (Position[] buffer) {
      if (!transformValid) {
         return null;
      }
      // calculate the standard deviation
      Position stdevPosition = new Position();
      for (Position i : buffer) {
         if (i == null)
            i = new Position();   // in a previous class, got a null reference after running for a while; never tracked down
         stdevPosition.X += Math.pow(i.X - llSmoothTransform.X, 2);
         stdevPosition.Y += Math.pow(i.Y - llSmoothTransform.Y, 2);
         stdevPosition.R += Math.pow(i.R - llSmoothTransform.R, 2);
      }
      stdevPosition.X = Math.sqrt(stdevPosition.X / transformNumber);
      stdevPosition.Y = Math.sqrt(stdevPosition.Y / transformNumber);
      stdevPosition.R = Math.sqrt(stdevPosition.R / transformNumber);

      return stdevPosition;
   }

   public boolean isStdevValid(Position stdevPos) {
      return Math.abs(stdevPos.X) <= acceptableStdDev.X &&
              Math.abs(stdevPos.Y) <= acceptableStdDev.Y &&
              Math.abs(stdevPos.R) <= acceptableStdDev.R;
   }

   @SuppressLint("DefaultLocale")
   public String roundIt(double d) {
      return String.format("%.2f", d);
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