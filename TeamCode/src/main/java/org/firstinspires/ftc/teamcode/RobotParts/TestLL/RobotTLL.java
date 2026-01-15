package org.firstinspires.ftc.teamcode.RobotParts.TestLL;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.RobotV2;

public class RobotTLL extends RobotV2 {
    public RobotTLL(Parts parts) {
        super(parts);
    }

    @Override
    public void initOptionsStart() {
        // This is relevant if using the new method getRobotYawPitchRollAngles(); see IMUmgr class
        hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

    @Override
    public void initOptionsEnd() {
//        sensorColor = hardwareMap.get(NormalizedColorSensor.class, "color");
    }
}
