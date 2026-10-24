package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.AutoDrive;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;

public class AutoDriveTB extends AutoDrive {
   public AutoDriveTB(Parts parts) {
      super(parts);
   }

   @Override
   public void extra() {
      if (TB_Misc.useLedLightShow) {
         TB_WFLED.driveData = new DriveData(powerTranslate, powerRotate, navAngle);
      }
   }
}