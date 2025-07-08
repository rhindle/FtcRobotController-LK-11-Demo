package org.firstinspires.ftc.teamcode.RobotParts.SMTest;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine.TaskStep;
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

   private StateMachine task;
   public StateMachine machine1;
   public StateMachine machine2;
   public StateMachine machine3;
   public StateMachine machine4;
   public StateMachine machine5;
   public StateMachine limitTask;
   public StateMachine testTask;

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
      task = machine1;
      task.setGroups("led", "servo");
      task.setAutoRestart(true);
      task.setStopRunnable( () -> setLedColor(rgbIndicatorColor.White) );
      task.setTimeLimit(5000);
      task.setTimeoutRunnable( () -> setLedColor(rgbIndicatorColor.Yellow) );
      task.setEndCriteria( () -> liftLimitSwitchNC.getState() );
      task.setEndCriteriaRunnable( () -> setLedColor(rgbIndicatorColor.Green) );
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Violet) );  //0
      task.addDelayOf( 1000);                                     //1
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );     //2
      task.addDelayOf( 500);                                      //3
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Violet) );  //4
      task.addDelayOf( 500);                                      //5
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );     //6
      task.addDelayOf( 250);                                      //7
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Violet) );  //8
      task.addDelayOf( 250);                                      //9
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );     //10
//      machine1.addRunOnce( () -> {                                         //11
//         if (liftLimitSwitchNC.getState()) machine1.gotoStep(7);
//         //if (liftLimitSwitchNC.getState()) machine1.end();
//      } );
      task.addDelayOf( 1000);                                     //12

      machine2 = new StateMachine("machine2");
      task = machine2;
      task.setGroups("led", "servo");
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      task.addDelayOf( 500);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 500);

      machine3 = new StateMachine("machine3");
      task = machine3;
      task.setGroups("led", "servo");
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      task.addDelayOf( 333);
      task.addRunOnce( () -> machine1.restart() );

      machine4 = new StateMachine("machine4");
      task = machine4;
      task.setGroups("oops"); //not going to kill the others
      //machine4.setMemberGroup("led");
      task.addRunOnce( () -> machine1.pause() );
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Sage) );
      task.addDelayOf( 1000);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Azure) );
      task.addDelayOf( 1000);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Sage) );
      task.addDelayOf( 1000);
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Azure) );
      task.addDelayOf( 1000);
      task.addRunOnce( () -> machine1.unPause() );

      machine5 = new StateMachine("machine5");
      task = machine5;
      task.setStopGroups("led", "servo");    // tasks to kill
      task.setMemberGroups("led", "servo");  // will be killed by
      // when this starts, it will kill the other led tasks.
      task.addRunOnce(machine1::restartNoStop);
      task.addDelayOf( 2000);
      task.addRunOnce(machine1::pause);
      task.addRunOnce(machine2::restartNoStop);
      task.addWaitFor(machine2::isDone);
      task.addRunOnce(machine1::unPause);
      task.addDelayOf( 2000);
      task.addRunOnce(machine4::restartNoStop);
      task.addWaitFor(machine4::isDone);
      task.addDelayOf( 2000);
      task.addRunOnce(machine1::pause);
      task.addRunOnce(machine3::restartNoStop);
      task.addWaitFor(machine3::isDone);
      // machine3 ends with starting machine1 and killing everything else
      // do something else?

      limitTask = new StateMachine("limitTask");
      task = limitTask;
      task.setGroups("safety");
      task.setAutoRestart(true);
      task.setNoBulkStop(true);  // keep this task running!
      task.addWaitFor( () -> slideLimitSwitchNC.getState() );
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Orange) );
      task.addWaitFor( () -> !slideLimitSwitchNC.getState() );
      task.addRunOnce( () -> setLedColor(rgbIndicatorColor.Indigo) );
      task.start();

      testTask = new StateMachine("testTask");
      task = testTask;
      task.setGroups("test");
      task.addSteps(
              new TaskStep( () -> setLedColor(rgbIndicatorColor.Violet) ),
              new TaskStep( 500 ),
              new TaskStep( () -> setLedColor(rgbIndicatorColor.Blue), () -> false, 2000)
      );

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
      StateMachine.stopByClass();
      setLedColor(rgbIndicatorColor.Off);
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