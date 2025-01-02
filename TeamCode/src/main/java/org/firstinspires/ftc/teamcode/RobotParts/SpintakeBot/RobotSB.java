package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Robot;

public class RobotSB extends Robot {
    public RobotSB(Parts parts) {
        super(parts);
    }

    @Override
    public void settingOptions() {
        // This is relevant if using the new method getRobotYawPitchRollAngles(); see IMUmgr class
        hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

    @Override
    public void initOptions() {
    }
}
