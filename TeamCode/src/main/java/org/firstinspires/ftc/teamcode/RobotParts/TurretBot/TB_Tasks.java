package org.firstinspires.ftc.teamcode.RobotParts.TurretBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.StateMachine;

class TB_Tasks {

    public static Parts parts;

    static StateMachine smIntakeOn;
    static StateMachine smIntakeOff;
    static StateMachine smRelax;
    static StateMachine smLaunch;
    static StateMachine smSpinDown;

    public static void setup(Parts p) {
        parts = p;
    }

    public static void initialize(){
        buildStateMachines();
    }

    static void buildStateMachines() {
        StateMachine task;

        // toggle intake on  (close gate, start intake)
        // toggle intake off  (stop intake, release pressure)
        //intake+transfer   (open gate, start both motors)
        // release pressure

        smIntakeOn = new StateMachine("intakeOn");
        task = smIntakeOn;
        task.setGroups("intake");  // will be killed by
        //task.setStopGroups("launcher", "green");    // groups to kill
        //task.setMemberGroups("all", "green");  // will be killed by
        task.setAutoRestart(false);
        //task.setStopRunnable( () -> {} );
        //task.setTimeLimit(5000);
        //task.setTimeoutRunnable( () -> {} );
        //task.setEndCriteria( () -> false );
        //task.setEndCriteriaRunnable( () -> {} );
        task.addRunOnce(TB_Intake::transferOff);
        task.addRunOnce(TB_Intake::gateClose);
        task.addWaitFor(() -> TB_Intake.servoGateL.isDone() && TB_Intake.servoGateR.isDone());
        task.addRunOnce(TB_Intake::intakeOn);

        smIntakeOff = new StateMachine("intakeOff");
        task = smIntakeOff;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> smRelax.restartNoStop());
        task.addWaitFor(() -> smRelax.isDone());

        smLaunch = new StateMachine("launch");
        task = smLaunch;
        task.setGroups("intake", "transfer", "spinner");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(TB_Intake::gateOpen);
        task.addWaitFor(() -> TB_Intake.servoGateL.isDone() && TB_Intake.servoGateR.isDone());
        task.addRunOnce(TB_Intake::transferOn);
        // here we need a timer or sensor to determine when to turn this off
        task.addDelayOf(2000);
        task.addRunOnce(TB_Intake::transferOff);
        task.addRunOnce(() -> smSpinDown.restartNoStop());

        smRelax = new StateMachine("relax");
        task = smRelax;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> {
            TB_Intake.motorIntake.setTargetPosition(TB_Intake.motorIntake.getCurrentPosition()-TB_Intake.reverseTicks);
            TB_Intake.motorTransfer.setTargetPosition(TB_Intake.motorTransfer.getCurrentPosition()-TB_Intake.reverseTicks);
            TB_Intake.motorIntake.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            TB_Intake.motorTransfer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            TB_Intake.motorIntake.setPower(1);
            TB_Intake.motorTransfer.setPower(1);
        });

        smSpinDown = new StateMachine("spinDown");
        task = smSpinDown;
        task.setGroups("spinner");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Turret::spinOff);
        task.addWaitFor(() -> TB_Turret.getMotorSpinSpeed(TB_Turret.motorSpin1) < TB_Turret.motorSpinIdleSpeed);
        task.addRunOnce(TB_Turret::spinIdle);

    }
}
