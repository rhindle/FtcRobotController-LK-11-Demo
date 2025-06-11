package org.firstinspires.ftc.teamcode.RobotParts.Common;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.TaskStep;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Supplier;

public class StateMachine {

    // instance tracking
    static ArrayList<StateMachine> list = new ArrayList<StateMachine>();
    static HashMap<String, StateMachine> map = new HashMap<>();

    // internal variables
    String name;
    runState state = runState.STOPPED;
    boolean running = false;
    boolean paused = false;
    boolean done = false;
    boolean success = true;
    boolean autoReset = false;
    boolean abortOnTimeout = false;

    ArrayList<String> killGroup = new ArrayList<>();
    ArrayList<String> memberGroup = new ArrayList<>();

    int currentStep = -1;
    long overallTimeLimit = 0;  // this will be weird if paused

    long overallStartTime;
    long stepStartTime;

    Runnable stepRunnable;
    Supplier<Boolean> stepEnd;
    Long stepTimeLimit;

    Supplier<Boolean> endCriteria;
    ArrayList<Runnable> steps = new ArrayList<>();
    ArrayList<Supplier<Boolean>> ends = new ArrayList<>();
    ArrayList<Long> times = new ArrayList<>();

    Runnable stopRunnable;
    Runnable abortRunnable;
    Runnable timeoutRunnable;
    Runnable endCriteriaRunnable;
    // do we need pauseRunnable? unPauseRunnable?

    // future internal use?
    boolean bool1, bool2, bool3;
    long long1, long2, long3;
    double dub1, dub2, dub3;

    public StateMachine(String name) {
        list.add(this);
        map.put(name, this);
    }

    public static void runLoop() {
        for (StateMachine machine : list ) {
            // do the state machine stuff
            if (!machine.running) continue;
            if (timeout(machine.overallStartTime, machine.overallTimeLimit)) {    // whole machine timed out
                machine.state = runState.TIMEOUT;
                machine.running = false;
                machine.done = true; //???
                machine.success = false;
                machine.currentStep = -1;
                if (machine.timeoutRunnable != null) machine.timeoutRunnable.run();
                continue;
            }
            if (machine.endCriteria.get()) {              // hit the end criteria which suppose is a success?
                machine.state = runState.COMPLETED;
                machine.running = false;
                machine.done = true;
                machine.success = true;
                machine.currentStep = -1;
                if (machine.endCriteriaRunnable != null) machine.endCriteriaRunnable.run();
                continue;
            }
            if (machine.currentStep == -1) machine.nextStep();
            boolean doLoop;
            do {
                doLoop = false;
                if (machine.stepRunnable != null) machine.stepRunnable.run();   // run the runnable
                if (machine.stepEnd.get() || timeout(machine.stepStartTime, machine.stepTimeLimit)) {   // meets end criteria or exceeds time
                    if (machine.abortOnTimeout && timeout(machine.stepStartTime, machine.stepTimeLimit)) {  // if abortOnTimeout, any timeout is a failure
                        machine.state = runState.FAILED;  // or TIMEOUT?
                        machine.running = false;
                        machine.done = true;
                        machine.success = false;
                        machine.currentStep = -1;
                        if (machine.abortRunnable != null) machine.abortRunnable.run();
                        continue;
                    }
                    if (machine.nextStep()) {            // if there's another step, do it
                        doLoop = true;
                    }
                    else {                               // no more steps = success
                        machine.state = runState.COMPLETED;
                        machine.running = false;
                        machine.done = true;
                        machine.success = true;
                        machine.currentStep = -1;
                    }
                }
            } while (doLoop);  // the loop is so execution can continue until there is something to wait for
        }
//        for (StateMachine machine : map.values()) {
//            // whatever
//        }
//        for (Map.Entry<String, StateMachine> entry : map.entrySet()) {
//            String name = entry.getKey();
//            StateMachine machine = entry.getValue();
//            // whatever
//        }
    }

    private static boolean timeout (long start, long limit) {
        if (limit == 0) return false;
        return System.currentTimeMillis() - start >= limit;
    }

    private boolean nextStep() {
        currentStep++;
        if (currentStep >= steps.size()) return false;
        stepRunnable = steps.get(currentStep);
        stepEnd = ends.get(currentStep);
        stepTimeLimit = times.get(currentStep);
        stepStartTime = System.currentTimeMillis();
        return true;
    }

    private void assassinate() {
        if (killGroup.isEmpty()) return;                      // if killgroup is empty, nothing to do
        for (String kName : killGroup) {
            for (StateMachine machine : list) {
                if (machine == this) continue;                // don't want to kill ourself
                if (machine.memberGroup.isEmpty()) continue;  // if membergroup is empty, nothing to do
                for (String sName : machine.memberGroup) {
                    if (sName.equals(kName)) {
                        machine.stop();
                    }
                }
            }
        }
    }

    enum runState {
        RUNNING,
        PAUSED,
        STOPPED,
        COMPLETED,
        TIMEOUT,
        FAILED;
    }

    public void stop() {
        if (running || paused) {
            success = false;
            if (stopRunnable != null) stopRunnable.run();  // do we have to check for null?
        }
        state = runState.STOPPED;
        running = false;
        paused = false;
        done = true; //???
        currentStep = -1;
    }

    public void start() {
        state = runState.RUNNING;
        running = true;
        paused = false;
        done = false;
        success = false;
        currentStep = -1;  //what about unpausing?
        overallStartTime = System.currentTimeMillis();
        stepStartTime = overallStartTime;
        assassinate();
    }

    public boolean pause() {
        if (!paused && running) {
            state = runState.PAUSED;
            running = false;
            paused = true;
            return true;
        }
        return false;
    }

    public boolean unPause() {
        if (paused) {
            state = runState.RUNNING;
            running = true;
            paused = false;
            // todo: reconsider what to do about the timers. Should the timer statuses be saved upon pausing?
            // Here, we just reset them all the way
            overallStartTime = System.currentTimeMillis();
            stepStartTime = overallStartTime;
            return true;
        }
        return false;
    }

    public boolean isDone() {
        return done;
        //return state == runState.COMPLETED;
    }

    public boolean isSuccess() {
        return success;
        //return state == runState.COMPLETED;
    }

    public boolean isRunning() {
        return running;
        //return state == runState.RUNNING;
    }

    public boolean isTimedOut() {
        return state == runState.TIMEOUT;
    }

    public StateMachine setAutoReset(boolean state) {
        autoReset = state;
        return this;
    }

    public StateMachine setAbortOnTimeout(boolean state) {
        abortOnTimeout = state;
        return this;
    }

    public StateMachine setTimeLimit (long limit) {
        // sanitize input?
        overallTimeLimit = limit;
        return this;
    }

    public void setEndCriteria(Supplier<Boolean> endCriteria) {
        this.endCriteria = endCriteria;
    }

//    public void test() {
//        addSteps(
//            new TaskStep(()-> {
//                System.out.println("step 1");
//            }),
//            new TaskStep(() -> true, 300),
//            new TaskStep(() -> {}, () -> false, 0)
//        );
//    }

    public void addSteps(TaskStep... taskSteps) {
        for (TaskStep taskStep : taskSteps) {
            addStep(taskStep.step, taskStep.end, taskStep.time);
        }
    }

    // add a step with end state and time limit
    public void addStep(Runnable step, Supplier<Boolean> end, long time) {
        if (step == null || end == null) return;
        //long timeLimit = (time == null || time < 0) ? 0 : time;
        //if (!end.get() && time == 0) end = () -> true;  // would never advance to next step?
        if (time < 0) time = 0;
        steps.add(step);
        ends.add(end);
        times.add(time);
    }

    // add a step with end state, no time limit
    public void addStep(Runnable step, Supplier<Boolean> end) {
        addStep(step, end, 0);
    }

    // add a step with only end state
    public void addStep(Supplier<Boolean> end) {
        addStep( () -> {}, end, 0);
    }

    // add a step end state and time limit
    public void addStep(Supplier<Boolean> end, long time) {
        addStep( () -> {}, end, time);
    }

    // add a step that runs once (no end state, no time limit)
    public void addStep(Runnable step) {
        addStep(step, () -> true, 0);
    }

    // add a step that is just a delay
    public void addStep(long time) {
        addStep( () -> {}, () -> false, time);
    }

    // add a step that repeats for time
    public void addStep(Runnable step, long time) {
        addStep(step, () -> false, time);
    }

    public void setGroups (String... names) {
        memberGroup = new ArrayList<>(Arrays.asList(names));
        killGroup = new ArrayList<>(Arrays.asList(names));
    }

    public void setMemberGroup (String... names) {
        memberGroup = new ArrayList<>(Arrays.asList(names));
    }

    public void setKillGroup (String... names) {
        killGroup = new ArrayList<>(Arrays.asList(names));
    }

    public void addGroup (String name) {
        memberGroup.add(name);
        killGroup.add(name);
    }

    public void addMemberGroup (String name) {
        memberGroup.add(name);
    }

    public void addKillGroup (String name) {
        killGroup.add(name);
    }

    public void setStopRunnable (Runnable run) {
        stopRunnable = run;
    }

    public void setAbortRunnable (Runnable run) {
        abortRunnable = run;
    }

    public void setTimeoutRunnable (Runnable run) {
        timeoutRunnable = run;
    }

    public void setEndCriteriaRunnable (Runnable run) {
        endCriteriaRunnable = run;
    }

}
