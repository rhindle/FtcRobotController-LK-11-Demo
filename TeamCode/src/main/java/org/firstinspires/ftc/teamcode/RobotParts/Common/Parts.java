package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSAprilTag;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSAuto;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSLed;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSMisc;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.DSSpeedControl;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
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
   public Slamra slamra;
   public PositionMgr positionMgr;
   public AutoDrive autoDrive;
   public UserDrive userDrive;
   public Sensors sensors;
   public TelemetryMgr telemetryHandler;
   public NeoMatrix neo;

   public boolean useODO = false;
   public boolean reverseDrive = false;
   public boolean useDistanceSensors = true;
   public boolean useDrivetrainEncoders = true;
   public boolean useSlamra = false;
   public boolean useAprilTag = false;
   public boolean useNeoMatrix = false;
   public boolean useIMU = true;   // todo: make everything work when this is disabled; odometry requires heading for example.
   public boolean useForzaControls = false;
   public Position robotPosition = new Position();
   public Position fieldStartPosition;
   public Position odoRobotOffset;
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
//   public T24Auto t24Auto;
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