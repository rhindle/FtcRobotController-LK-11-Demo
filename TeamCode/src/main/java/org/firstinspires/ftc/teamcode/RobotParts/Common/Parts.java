package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.EncoderTracker;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.ImuMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Odometry;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Pinpoint;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.PositionMgr;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Position.Slamra;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSAprilTag;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSAuto;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSLed;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSMisc;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSSpeedControl;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.Intake.SB_Intake;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.SB_Auto;
import org.firstinspires.ftc.teamcode.RobotParts.SpintakeBot.SB_Misc;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.Intake.T24MultiGrabber;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.T24Grabber;
import org.firstinspires.ftc.teamcode.RobotParts.Test2024.T24Misc;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class Parts implements PartsInterface {

   /* Public OpMode members. */
   public LinearOpMode opMode;
   public Robot robot;
   public ButtonMgr buttonMgr;
   public Controls controls;
   public Drivetrain drivetrain;
   public ImuMgr imuMgr;
   public Odometry odometry;
   public Pinpoint pinpoint;
   public Slamra slamra;
   public EncoderTracker encoderTracker;
   public PositionMgr positionMgr;
   public AutoDrive autoDrive;
   public UserDrive userDrive;
   public Sensors sensors;
   public TelemetryMgr telemetryHandler;
   public NeoMatrix neo;

   public boolean useODO = false;
   public boolean usePinpoint = false;
   public boolean reverseDrive = false;
   public boolean useDistanceSensors = true;
   public boolean useDrivetrainEncoders = true;
   public boolean useSlamra = false;
   public boolean useEncoderTracker = false;
   public boolean useAprilTag = false;
   public boolean useNeoMatrix = false;
   public boolean useIMU = true;   // todo: make everything work when this is disabled; odometry requires heading for example.
   public boolean useForzaControls = false;
   public Position robotPosition = new Position();
   public Position fieldStartPosition;
   public Position odoRobotOffset;
   public Position pinpointRobotOffset;
   public Position slamraRobotOffset;
   public boolean isSetup = false;
   public double speedMaximum = 1;

   //---DiscShooter Unique
   public DSAprilTag dsApriltag;
   public DSLed dsLed;
   public DSShooter dsShooter;
   public DSSpeedControl dsSpeedControl;
   public DSMisc dsMisc;
   public DSAuto dsAuto;
   //---

   //---Test24 Unique
   public T24Misc t24Misc;
   public T24Grabber t24Grabber;
   public T24MultiGrabber t24MultiGrabber;
//   public T24Auto t24Auto;
   //---

   //---SpintakeBot Unique
   public SB_Misc sb_Misc;
   public SB_Intake sb_Intake;
//   public SB_Intake_Backup sb_Intake;
   public SB_Auto sb_Auto;
   //---

   /* Constructor */
   public Parts(LinearOpMode opMode){
      construct(opMode);
   }

   void construct(LinearOpMode opMode){
      this.opMode = opMode;
      telemetryHandler = new TelemetryMgr(opMode);
   }

   public void setup(){
   }

   public void initialize() {
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
   }

   public void autoRunLoop() {
   }

   public void stop() {
   }

}