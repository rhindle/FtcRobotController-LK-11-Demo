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
      machine1.setGroups("led", "servo");
      machine1.setAutoRestart(true);
      machine1.setTimeoutRunnable( () -> setLedColor(rgbIndicatorColor.Yellow) );
      machine1.setStopRunnable( () -> setLedColor(rgbIndicatorColor.White) );
      machine1.setEndCriteriaRunnable( () -> setLedColor(rgbIndicatorColor.Green) );
      machine1.setEndCriteria( () -> liftLimitSwitchNC.getState() );
      machine1.setTimeLimit(5000);
      machine1.addRunOnce( () -> setLedColor(rgbIndicatorColor.Violet) );  //0
      machine1.addDelayOf( 1000);                                     //1
      machine1.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );     //2
      machine1.addDelayOf( 500);                                      //3
      machine1.addRunOnce( () -> setLedColor(rgbIndicatorColor.Violet) );  //4
      machine1.addDelayOf( 500);                                      //5
      machine1.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );     //6
      machine1.addDelayOf( 250);                                      //7
      machine1.addRunOnce( () -> setLedColor(rgbIndicatorColor.Violet) );  //8
      machine1.addDelayOf( 250);                                      //9
      machine1.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );     //10
//      machine1.addRunOnce( () -> {                                         //11
//         if (liftLimitSwitchNC.getState()) machine1.gotoStep(7);
//         //if (liftLimitSwitchNC.getState()) machine1.end();
//      } );
      machine1.addDelayOf( 1000);                                     //12

      machine2 = new StateMachine("machine2");
      machine2.setGroups("led", "servo");
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Red) );
      machine2.addDelayOf( 500);
      machine2.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine2.addDelayOf( 500);

      machine3 = new StateMachine("machine3");
      machine3.setGroups("led", "servo");
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Blue) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> setLedColor(rgbIndicatorColor.Off) );
      machine3.addDelayOf( 333);
      machine3.addRunOnce( () -> machine1.restart() );

      machine4 = new StateMachine("machine4");
      machine4.setGroups("oops"); //not going to kill the others
      //machine4.setMemberGroup("led");
      machine4.addRunOnce( () -> machine1.pause() );
      machine4.addRunOnce( () -> setLedColor(rgbIndicatorColor.Sage) );
      machine4.addDelayOf( 1000);
      machine4.addRunOnce( () -> setLedColor(rgbIndicatorColor.Azure) );
      machine4.addDelayOf( 1000);
      machine4.addRunOnce( () -> setLedColor(rgbIndicatorColor.Sage) );
      machine4.addDelayOf( 1000);
      machine4.addRunOnce( () -> setLedColor(rgbIndicatorColor.Azure) );
      machine4.addDelayOf( 1000);
      machine4.addRunOnce( () -> machine1.unPause() );

      machine5 = new StateMachine("machine5");
      machine5.setStopGroups("led", "servo");    // tasks to kill
      machine5.setMemberGroups("led", "servo");  // will be killed by
      // when this starts, it will kill the other led tasks.
      machine5.addRunOnce(machine1::restartNoStop);
      machine5.addDelayOf( 2000);
      machine5.addRunOnce(machine1::pause);
      machine5.addRunOnce(machine2::restartNoStop);
      machine5.addWaitFor(machine2::isDone);
      machine5.addRunOnce(machine1::unPause);
      machine5.addDelayOf( 2000);
      machine5.addRunOnce(machine4::restartNoStop);
      machine5.addWaitFor(machine4::isDone);
      machine5.addDelayOf( 2000);
      machine5.addRunOnce(machine1::pause);
      machine5.addRunOnce(machine3::restartNoStop);
      machine5.addWaitFor(machine3::isDone);
      // machine3 ends with starting machine1 and killing everything else
      // do something else?

      limitTask = new StateMachine("limitTask");
      limitTask.setGroups("safety");
      limitTask.setAutoRestart(true);
      limitTask.setNoBulkStop(true);  // keep this task running!
      limitTask.addWaitFor( () -> slideLimitSwitchNC.getState() );
      limitTask.addRunOnce( () -> setLedColor(rgbIndicatorColor.Orange) );
      limitTask.addWaitFor( () -> !slideLimitSwitchNC.getState() );
      limitTask.addRunOnce( () -> setLedColor(rgbIndicatorColor.Indigo) );
      limitTask.start();

      testTask = new StateMachine("testTask");
      testTask.setGroups("test");
      testTask.addSteps(
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