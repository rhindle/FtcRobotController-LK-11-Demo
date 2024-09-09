package org.firstinspires.ftc.teamcode.RobotParts.Common;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

public class Sensors implements PartsInterface {

   //todo: This is not a very good "Common" part. Consider making sweeping changes!

   Parts parts;
   HardwareMap hardwareMap;

   public DistanceSensor sensor2MLeft = null;
   public DistanceSensor sensor2MMiddle = null;
   public DistanceSensor sensor2MRight = null;
   public DigitalChannel ledRED = null;
   public DigitalChannel ledGREEN = null;

   public DigitalChannel switch0B = null;

   public double distL, distM, distR;
   private int distCounter = 0;
   private boolean readDistSensors = false;

   /* Constructor */
   public Sensors(Parts parts){
      construct(parts);
   }

   public void construct(Parts parts){
      this.parts = parts;
      this.hardwareMap = parts.opMode.hardwareMap;
   }

   public void initialize(){
      if (parts.useDistanceSensors) {
         sensor2MLeft = hardwareMap.get(DistanceSensor.class, "2MdistL");
         sensor2MMiddle = hardwareMap.get(DistanceSensor.class, "2MdistM");
         sensor2MRight = hardwareMap.get(DistanceSensor.class, "2MdistR");
      }
      ledRED = hardwareMap.get(DigitalChannel.class, "digital6B");
      ledGREEN = hardwareMap.get(DigitalChannel.class, "digital7B");
      ledRED.setMode(DigitalChannel.Mode.OUTPUT);
      ledGREEN.setMode(DigitalChannel.Mode.OUTPUT);
      ledRED.setState(true);
      ledGREEN.setState(true);

      switch0B = hardwareMap.get(DigitalChannel.class, "digital0B");
      switch0B.setMode(DigitalChannel.Mode.INPUT);
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop(){
      if (parts.useDistanceSensors) updateDistanceSensors();
   }

   public void stop() {
   }

   private void updateDistanceSensors() {
      if (readDistSensors) {
         switch (distCounter) {
            case 0:
               distL = sensor2MLeft.getDistance(DistanceUnit.INCH);
               break;
            case 1:
               distM = sensor2MMiddle.getDistance(DistanceUnit.INCH);
               break;
            case 2:
               distR = sensor2MRight.getDistance(DistanceUnit.INCH);
               break;
            case 3:    // does it work better if we alternate?
               distM = sensor2MMiddle.getDistance(DistanceUnit.INCH);
               break;
         }
         distCounter++;
         if (distCounter > 3) distCounter = 0;
      } else {
         distL = -1;
         distM = -1;
         distR = -1;
      }
   }

   public void readDistSensors (boolean boo) {
      readDistSensors = boo;
      ledGREEN.setState(!readDistSensors);
   }
   public void readDistSensors () {
      readDistSensors = !readDistSensors;
      ledGREEN.setState(!readDistSensors);
   }

   public void setLedRED(boolean boo) {
      ledRED.setState(!boo);
   }

   public void setLedGREEN(boolean boo) {
      ledGREEN.setState(!boo);
   }

   public boolean getSwitch0B() {
      return switch0B.getState();
   }
}
