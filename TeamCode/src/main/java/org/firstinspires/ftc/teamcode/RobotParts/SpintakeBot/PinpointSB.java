package org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Pinpoint;
import org.firstinspires.ftc.teamcode.Tools.i2c.GoBildaPinpointDriver;

public class PinpointSB extends Pinpoint {

    public PinpointSB(Parts parts) {
        super(parts);
    }

    @Override
    public void pinpointSettings() {
        deviceName = "pinpoint";
        xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        resolution = 13.26291192f;
        //offsets are taken care of with parts.pinpointRobotOffset
    }
}
