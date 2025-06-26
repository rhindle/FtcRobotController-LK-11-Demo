package org.firstinspires.ftc.teamcode.RobotParts.SMTest;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.ServoSSR;

public class SMT_LED implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public static ServoSSR rgbIndicator;
   public static DigitalChannel slideLimitSwitchNO = null;
   public static DigitalChannel slideLimitSwitchNC = null;
   public static DigitalChannel liftLimitSwitchNO = null;
   public static DigitalChannel liftLimitSwitchNC = null;

   public StateMachine machine1;
   public StateMachine machine2;
   public StateMachine machine3;
   public StateMachine machine4;
   public StateMachine machine5;
   public StateMachine limitTask;

   /* Constructor */
   public SMT_LED(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      rgbIndicator = new ServoSSR(parts.robot.servo0);
      rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color);
      slideLimitSwitchNO = parts.robot.digital1;
      slideLimitSwitchNC = parts.robot.digital0;
      liftLimitSwitchNO = parts.robot.digital3;
      liftLimitSwitchNC = parts.robot.digital2;

      machine1 = new StateMachine("machine1");
      machine2 = new StateMachine("machine2");
      machine3 = new StateMachine("machine3");
      machine4 = new StateMachine("machine4");
      limitTask = new StateMachine("limitTask");

      machine1.setGroups("led", "servo");
      machine1.setAutoReset(true);
      machine1.addRunStep( () -> setLedColor(rgbIndicatorColor.Violet) );
      machine1.addDelayOf( 1000);
      machine1.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine1.addDelayOf( 500);
      machine1.addRunStep( () -> setLedColor(rgbIndicatorColor.Violet) );
      machine1.addDelayOf( 500);
      machine1.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine1.addDelayOf( 250);
      machine1.addRunStep( () -> setLedColor(rgbIndicatorColor.Violet) );
      machine1.addDelayOf( 250);
      machine1.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine1.addDelayOf( 1000);

      machine2.setGroups("led", "servo");
      machine2.setAutoReset(false);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);

      machine3.setGroups("led", "servo");
      machine3.setAutoReset(false);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunStep( () -> machine1.restart() );

      machine4.setGroups("oops"); //not going to kill the others
      //machine4.setMemberGroup("led");
      machine4.setAutoReset(false);
      machine4.addRunStep( () -> machine1.pause() );
      machine4.addRunStep( () -> setLedColor(rgbIndicatorColor.Sage) );
      machine4.addDelayOf( 1000);
      machine4.addRunStep( () -> setLedColor(rgbIndicatorColor.Azure) );
      machine4.addDelayOf( 1000);
      machine4.addRunStep( () -> setLedColor(rgbIndicatorColor.Sage) );
      machine4.addDelayOf( 1000);
      machine4.addRunStep( () -> setLedColor(rgbIndicatorColor.Azure) );
      machine4.addDelayOf( 1000);
      machine4.addRunStep( () -> machine1.unPause() );

      machine5 = new StateMachine("machine5");
      machine5.setStopGroups("led", "servo");    // tasks to kill
      machine5.setMemberGroups("led", "servo");  // will be killed by
      machine5.setAutoReset(false);
      // when this starts, it will kill the other led tasks.
      machine5.addRunStep(machine1::restartNoStop);
      machine5.addDelayOf( 2000);
      machine5.addRunStep(machine1::pause);
      machine5.addRunStep(machine2::restartNoStop);
      machine5.addWaitFor(machine2::isDone);
      machine5.addRunStep(machine1::unPause);
      machine5.addDelayOf( 2000);
      machine5.addRunStep(machine4::restartNoStop);
      machine5.addWaitFor(machine4::isDone);
      machine5.addDelayOf( 2000);
      machine5.addRunStep(machine1::pause);
      machine5.addRunStep(machine3::restartNoStop);
      machine5.addWaitFor(machine3::isDone);
      // machine3 ends with starting machine1 and killing everything else
      // do something else?

      limitTask.setGroups("safety");
      limitTask.setAutoReset(true);
      limitTask.setNoBulkStop(true);  // keep this task running!
      //if (slideLimitSwitchNC.getState() && !slideLimitSwitchNO.getState()) slideTemp=1;
      limitTask.addWaitFor( () -> slideLimitSwitchNC.getState() );
      limitTask.addRunStep( () -> setLedColor(rgbIndicatorColor.Orange) );
      limitTask.addWaitFor( () -> !slideLimitSwitchNC.getState() );
      limitTask.addRunStep( () -> setLedColor(rgbIndicatorColor.Indigo) );
      limitTask.start();

   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
//      TelemetryMgr.message(TelemetryMgr.Category.LED, "Machine1", machine1.getStatus());
//      TelemetryMgr.message(TelemetryMgr.Category.LED, "Machine2", machine2.getStatus());
//      TelemetryMgr.message(TelemetryMgr.Category.LED, "Machine3", machine3.getStatus());
//      TelemetryMgr.message(TelemetryMgr.Category.LED, "Machine4", machine4.getStatus());
   }

   public void stop() {
//      machine1.stop(); // temporary solution
//      machine2.stop();
//      machine3.stop();
//      machine4.stop();
      StateMachine.stopClass();
      rgbIndicator.setPosition(rgbIndicatorColor.Off.color);
      //StateMachine.stopAll();
   }

   void setLedColor (rgbIndicatorColor colorName) {
      rgbIndicator.setPosition(colorName.color);
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