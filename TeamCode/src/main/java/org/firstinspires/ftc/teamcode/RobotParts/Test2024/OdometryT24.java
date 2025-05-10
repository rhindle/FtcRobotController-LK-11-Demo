package org.firstinspires.ftc.teamcode.RobotParts.Test2024;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Odometry;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;

public class OdometryT24 extends Odometry {

    public OdometryT24(Parts parts) {
        super(parts);
    }

    @Override
    public void configureEncoders() {
        double TPI = 82300 / 48.0;
        double TPR = 169619;
        /* direction should be -1 or 1 and is to account for the port being used elsewhere
           and needing to be set in direction opposite to what odometry needs */
        odoEncXL = new EncoderSetting(parts.robot.motor2B, 1, TPI);
        odoEncXR = new EncoderSetting(parts.robot.motor1B, 1, TPI);
        odoEncY  = new EncoderSetting(parts.robot.motor0B, 1, TPI);
        odoEncXL.encoderPort.setDirection(DcMotorEx.Direction.REVERSE);
        odoEncXR.encoderPort.setDirection(DcMotorEx.Direction.REVERSE);
        odoEncY.encoderPort.setDirection(DcMotorEx.Direction.FORWARD);
        odoEncXL.encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoEncXR.encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoEncY.encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        eTicksPerRotate = TPR;
    }
}
