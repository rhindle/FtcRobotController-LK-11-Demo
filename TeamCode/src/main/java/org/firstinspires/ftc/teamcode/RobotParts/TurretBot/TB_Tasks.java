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
    static StateMachine smInitServos;
    static StateMachine smUnpauseTurret;
    static StateMachine smIntakeAutoStop;

    public static void setup(Parts p) {
        parts = p;
    }

    public static void initialize(){
        buildStateMachines();
    }

    static void buildStateMachines() {
        StateMachine task;

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
        task.addRunOnce(() -> TB_Turret.pauseTurret(true));  // todo: see if this works well
        task.addRunOnce(TB_Intake::transferOff);
        task.addRunOnce(TB_Intake::gateClose);
        task.addWaitFor(() -> TB_Intake.servoGateL.isDone() && TB_Intake.servoGateR.isDone());
        task.addRunOnce(TB_Intake::intakeOn);
        task.addRunOnce(() -> smIntakeAutoStop.restart());  // throws a null with :: notation

        smIntakeOff = new StateMachine("intakeOff");
        task = smIntakeOff;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> TB_Turret.pauseTurret(false));  // todo: see if this works well
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(() -> smRelax.restartNoStop());
        task.addWaitFor(() -> smRelax.isDone());

        smIntakeAutoStop = new StateMachine("intakeAuto");
        task = smIntakeAutoStop;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            if (TB_Intake.disableIntakeSensors) smIntakeAutoStop.end();
        });
        task.addWaitFor(() -> TB_Intake.definitely3);
//        task.addRunOnce(TB_Turret::indicateFullIntake);
        task.addRunOnce(smIntakeOff::restart);

        smUnpauseTurret = new StateMachine("unpauseTurret");
        task = smUnpauseTurret;
        task.setGroups("transfer");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            if (!TB_Turret.turretPaused) smUnpauseTurret.end();
        });
        task.addRunOnce(() -> TB_Turret.pauseTurret(false));
        task.addYield();  // this break should allow the turret to be repositioned
        task.addWaitFor(TB_Turret.servoTurretR::isDone);

        smLaunch = new StateMachine("launch");
        task = smLaunch;
        task.setGroups("intake", "transfer", "spinner");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(smUnpauseTurret::restartNoStop);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addRunOnce(TB_Intake::gateOpen);
        task.addWaitFor(() -> TB_Intake.servoGateL.isDone() && TB_Intake.servoGateR.isDone());
        task.addWaitFor(smUnpauseTurret::isDone);
        task.addRunOnce(TB_Intake::transferOn);
        // here we need a timer or sensor to determine when to turn this off
        task.addWaitFor(() -> TB_Intake.probably0, 1700);
        task.addDelayOf(300);
        task.addRunOnce(TB_Intake::transferOff);
//        task.addRunOnce(TB_Intake::intakeOff);  // do we want this on? off? whatever state it was?
        task.addRunOnce(TB_Intake::gateClose);
        task.addRunOnce(() -> smSpinDown.restartNoStop());
        task.addRunOnce(smIntakeOn::restart);

        smRelax = new StateMachine("relax");   //now does the opposite?
        task = smRelax;
        task.setGroups("intake");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Intake::intakeOff);
        task.addDelayOf(500);
        task.addRunOnce(() -> {
            TB_Intake.motorIntake.setTargetPosition(TB_Intake.motorIntake.getCurrentPosition()+TB_Intake.reverseTicks);
            TB_Intake.motorTransfer.setTargetPosition(TB_Intake.motorTransfer.getCurrentPosition()+TB_Intake.reverseTicks);
            TB_Intake.motorIntake.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            TB_Intake.motorTransfer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            TB_Intake.motorIntake.setPower(1);
            TB_Intake.motorTransfer.setPower(1);
        });
        task.addDelayOf(2000);
        task.addRunOnce(TB_Intake::transferOff);
        task.addRunOnce(TB_Intake::intakeOff);

        smSpinDown = new StateMachine("spinDown");
        task = smSpinDown;
        task.setGroups("spinner");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(TB_Turret::spinOff);
        task.addWaitFor(() -> TB_Turret.getMotorSpinSpeed(TB_Turret.motorSpin1) < TB_Turret.spinnerIdleSpeed);
        task.addRunOnce(TB_Turret::spinIdle);

        smInitServos = new StateMachine("initServos");
        task = smInitServos;
        task.setGroups("intake", "transfer");  // will be killed by
        task.setAutoRestart(false);
        task.addRunOnce(() -> {
            TB_Intake.servoGateL.setPosition(0.5);
            TB_Intake.servoGateR.setPosition(0.5);
            TB_Turret.servoHood.setPosition(0.5);
            TB_Turret.servoTurretL.setPosition(0.55);
            TB_Turret.servoTurretR.setPosition(0.55);
        });
        task.addDelayOf(500);
        task.addRunOnce(() -> {
            TB_Intake.gateOpen();
            TB_Turret.servoHood.setPosition(0.02);
            TB_Turret.servoTurretL.setPosition(0.45);
            TB_Turret.servoTurretR.setPosition(0.45);
        });
        task.addDelayOf(500);
        task.addRunOnce(() -> {
            TB_Intake.gateClose();
            TB_Turret.servoHood.setPosition(0.25);
            TB_Turret.servoTurretL.setPosition(0.5);
            TB_Turret.servoTurretR.setPosition(0.5);
            TB_Turret.armTurret(false);
        });
        task.addRunOnce(() -> {
            TB_Turret.turretLPosSetting[0] = TB_Turret.servoTurretL.getPosition();
            TB_Turret.turretRPosSetting[0] = TB_Turret.servoTurretR.getPosition();
            TB_Turret.hoodPosSetting[0] = TB_Turret.servoHood.getPosition();
            TB_Turret.spinner1VelocitySetting[0] = 0;
            TB_Turret.spinner2VelocitySetting[0] = 0;
            TB_Turret.LEDSettingTurret[0] = 0;
        });
        task.addRunOnce(() -> TB_Misc.servosInit = true);

    }
}
