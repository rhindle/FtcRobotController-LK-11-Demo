/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ZZ;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class ZZ_Hardware_TestBot_B
{
   /* Public OpMode members. */
   public DcMotor        motor0   = null;
   public DcMotor        motor1   = null;
   public DcMotor        motor2   = null;
   public DcMotor        motor3   = null;
   public DcMotor        motor0b   = null;
   public DcMotor        motor1b   = null;
   public DcMotor        motor2b   = null;
   public DcMotor        motor3b   = null;

   public Servo          servo0   = null;
   public Servo          servo1   = null;
   public Servo          servo2   = null;
   public Servo          servo3   = null;
   public Servo          servo4   = null;
   public Servo          servo5   = null;
   public Servo          servo0b   = null;
   public Servo          servo1b   = null;
   public Servo          servo2b   = null;
   public Servo          servo3b   = null;
   public Servo          servo4b   = null;
   public Servo          servo5b   = null;

//    public ColorSensor    sensorColor    = null;
//    public DistanceSensor sensorDistance = null;

   public DigitalChannel digital0 = null;
   public DigitalChannel digital1 = null;
   public DigitalChannel digital2 = null;
   public DigitalChannel digital3 = null;
   public DigitalChannel digital4 = null;
   public DigitalChannel digital5 = null;
   public DigitalChannel digital6 = null;
   public DigitalChannel digital7 = null;

   public AnalogInput analog0 = null;
   public AnalogInput analog1 = null;
   public AnalogInput analog2 = null;
   public AnalogInput analog3 = null;

   public BNO055IMU      sensorIMU      = null;

//    public Orientation    angles;

   /* local OpMode members. */
   HardwareMap hwMap           =  null;
   private ElapsedTime period  = new ElapsedTime();

   /* Constructor */
   public ZZ_Hardware_TestBot_B(){

   }

   /* Initialize standard Hardware interfaces */
   public void init(HardwareMap ahwMap) {
      // Save reference to Hardware map
      hwMap = ahwMap;

      // Define and Initialize Motors
      motor0 = hwMap.get(DcMotor.class, "motor0");
      motor1 = hwMap.get(DcMotor.class, "motor1");
      motor2 = hwMap.get(DcMotor.class, "motor2");
      motor3 = hwMap.get(DcMotor.class, "motor3");
      motor0b = hwMap.get(DcMotor.class, "motor0B");
      motor1b = hwMap.get(DcMotor.class, "motor1B");
      motor2b = hwMap.get(DcMotor.class, "motor2B");
      motor3b = hwMap.get(DcMotor.class, "motor3B");

      motor0.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor3.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor0b.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor1b.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor2b.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
      motor3b.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

      motor0.setPower(0);
      motor1.setPower(0);
      motor2.setPower(0);
      motor3.setPower(0);
      motor0b.setPower(0);
      motor1b.setPower(0);
      motor2b.setPower(0);
      motor3b.setPower(0);

      // Set all motors to run without encoders.
      // May want to use RUN_USING_ENCODERS if encoders are installed.
      motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor0b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor1b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor2b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor3b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      // Define and initialize ALL installed servos.
      servo0 = hwMap.get(Servo.class,"servo0");
      servo1 = hwMap.get(Servo.class,"servo1");
      servo2 = hwMap.get(Servo.class,"servo2");
      servo3 = hwMap.get(Servo.class,"servo3");
      servo4 = hwMap.get(Servo.class,"servo4");
      servo5 = hwMap.get(Servo.class,"servo5");
      servo0b = hwMap.get(Servo.class,"servo0B");
      servo1b = hwMap.get(Servo.class,"servo1B");
      servo2b = hwMap.get(Servo.class,"servo2B");
      servo3b = hwMap.get(Servo.class,"servo3B");
      servo4b = hwMap.get(Servo.class,"servo4B");
      servo5b = hwMap.get(Servo.class,"servo5B");

      digital0 = hwMap.get(DigitalChannel.class, "digital0");
      digital1 = hwMap.get(DigitalChannel.class, "digital1");
      digital2 = hwMap.get(DigitalChannel.class, "digital2");
      digital3 = hwMap.get(DigitalChannel.class, "digital3");
      digital4 = hwMap.get(DigitalChannel.class, "digital4");
      digital5 = hwMap.get(DigitalChannel.class, "digital5");
      digital6 = hwMap.get(DigitalChannel.class, "digital6");
      digital7 = hwMap.get(DigitalChannel.class, "digital7");

      digital0.setMode(DigitalChannel.Mode.INPUT);
      digital1.setMode(DigitalChannel.Mode.INPUT);
      digital2.setMode(DigitalChannel.Mode.INPUT);
      digital3.setMode(DigitalChannel.Mode.INPUT);
      digital4.setMode(DigitalChannel.Mode.INPUT);
      digital5.setMode(DigitalChannel.Mode.INPUT);
      digital6.setMode(DigitalChannel.Mode.INPUT);
      digital7.setMode(DigitalChannel.Mode.INPUT);

      analog0 = hwMap.get(AnalogInput.class, "analog0");
      analog1 = hwMap.get(AnalogInput.class, "analog1");
      analog2 = hwMap.get(AnalogInput.class, "analog2");
      analog3 = hwMap.get(AnalogInput.class, "analog3");

      // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
      // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
      // and named "sensorIMU".
      sensorIMU = hwMap.get(BNO055IMU.class, "imu");
      BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
      sensorIMU.initialize(parameters);

      //    sensorColor = hwMap.get(ColorSensor.class, "topSensor");
      //    sensorDistance = hwMap.get(DistanceSensor.class, "topSensor");
   }
}
