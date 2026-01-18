package org.firstinspires.ftc.teamcode.RobotParts.TestLL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class TLL_Limelight implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   Limelight3A limelight;

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
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
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

      if (result != null && result.isValid()) {
         Pose3D botpose = result.getBotpose();
         if (botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            double z = botpose.getPosition().z;
            double r = botpose.getOrientation().getYaw(AngleUnit.DEGREES);


//            parts.opMode.telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            TelemetryMgr.message(TelemetryMgr.Category.LL, "MT1 Location","(" + x + ", " + y + ", " + z + ", " + r + ")");

            Position position = new Position(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y, result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));
            Position transform = position.getOffset(parts.positionMgr.robotPosition);

            TelemetryMgr.message(TelemetryMgr.Category.LL, "Offset", transform.toString());


         }
      }


   }

   public void stop() {
   }

}