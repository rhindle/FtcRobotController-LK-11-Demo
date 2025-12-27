package org.firstinspires.ftc.teamcode.RobotParts.Test2025;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;

public class DrivetrainT25 extends Drivetrain {
   public DrivetrainT25(Parts parts) {
      super(parts);
   }

   @Override
   public void setDriveMotors() {
//      motorLF = parts.robotV2.getMotorByName("motor0");  //reverse
//      motorRF = parts.robotV2.getMotorByName("motor1");
//      motorLR = parts.robotV2.getMotorByName("motor2");
//      motorRR = parts.robotV2.getMotorByName("motor3");
      motorLF = parts.robotV2.getMotorByName("motor0");  //reverse
      motorRF = parts.robotV2.getMotorByName("motor0B");  //forward
      motorLR = parts.robotV2.getMotorByName("motor1");  //reverse
      motorRR = parts.robotV2.getMotorByName("motor1B");  //forward
   }
}
