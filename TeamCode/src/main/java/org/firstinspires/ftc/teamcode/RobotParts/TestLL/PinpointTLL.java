package org.firstinspires.ftc.teamcode.RobotParts.TestLL;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Pinpoint;
import org.firstinspires.ftc.teamcode.Tools.i2c.GoBildaPinpointDriver;

public class PinpointTLL extends Pinpoint {

    public PinpointTLL(Parts parts) {
        super(parts);
    }

    @Override
    public void pinpointSettings() {
        deviceName = "odo";
        xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED; //FORWARD;
        resolution = 19.89436789f; //13.26291192f;
        //offsets are taken care of with parts.pinpointRobotOffset
    }
}
