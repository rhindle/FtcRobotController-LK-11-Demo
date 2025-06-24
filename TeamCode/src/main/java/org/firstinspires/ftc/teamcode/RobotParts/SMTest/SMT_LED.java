package org.firstinspires.ftc.teamcode.RobotParts.SMTest;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class SMT_LED implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public ServoSSR rgbIndicator;

   public StateMachine machine1;
   public StateMachine machine2;
   public StateMachine machine3;
   public StateMachine machine4;

   /* Constructor */
   public SMT_LED(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
      rgbIndicator = new ServoSSR(parts.robot.servo0);
   }

   public void initialize(){
      rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color);
      machine1 = new StateMachine("machine1");
      machine2 = new StateMachine("machine2");
      machine3 = new StateMachine("machine3");
      machine4 = new StateMachine("machine4");

      machine1.setGroups("led", "servo");
      machine1.setAutoReset(true);
      machine1.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color) );
      machine1.addStep( 1000);
      machine1.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine1.addStep( 1000);

      machine2.setGroups("led", "servo");
      machine2.setAutoReset(false);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Red.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Red.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Red.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Red.color) );
      machine2.addStep( 500);
      machine2.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine2.addStep( 500);

      machine3.setGroups("led", "servo");
      machine3.setAutoReset(false);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Blue.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Blue.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Blue.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Blue.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color) );
      machine3.addStep( 333);
      machine3.addStep( () -> machine1.restart() );

      machine4.setGroups("oops"); //not going to kill the others
      //machine4.setMemberGroup("led");
      machine4.setAutoReset(false);
      machine4.addStep( () -> machine1.pause() );
      machine4.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Sage.color) );
      machine4.addStep( 1000);
      machine4.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Azure.color) );
      machine4.addStep( 1000);
      machine4.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Sage.color) );
      machine4.addStep( 1000);
      machine4.addStep( () -> rgbIndicator.setPosition(rgbIndicatorColor.Azure.color) );
      machine4.addStep( 1000);
      machine4.addStep( () -> machine1.unPause() );


   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
   }

   public void stop() {
      machine1.stop(); // temporary solution
      machine2.stop();
      machine3.stop();
      machine4.stop();
      rgbIndicator.setPosition(rgbIndicatorColor.Off.color);
      //StateMachine.stopAll();
   }

   public enum rgbIndicatorColor {
      Off (0.0),
      Red (0.279),
      Orange (0.333),
      Yellow (0.388),
      Sage (0.444),
      Green (0.500),
      Azure (0.555),
      Blue (0.611),
      Indigo (0.666),
      Violet (0.715), //(0.722),
      White (1.0);

      private final double color;

      rgbIndicatorColor(double color) {
         this.color = color;
      }
   }
}