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

    static boolean pausedAll = false;

    // internal variables
    String name;
    runState state = runState.STOPPED;
    boolean running = false;
    boolean paused = false;
    boolean done = false;
    boolean success = true;
    boolean autoReset = false;
    boolean abortOnTimeout = false;
    boolean returnStatus = false;
    boolean tempNoStop = false;

    ArrayList<String> stopGroup = new ArrayList<>();
    ArrayList<String> memberGroup = new ArrayList<>();

    int currentStep = -1;
    long overallTimeLimit = 0;  // this will be weird if paused

    long overallStartTime;
    long stepStartTime;
    long pausedStepTime;
    long pausedOverallTime;

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
        this.name = name;
    }

    public static void runLoop() {
        for (StateMachine machine : list ) {
            // do the state machine stuff
            if (!machine.running) continue;
            if (timeout(machine.overallStartTime, machine.overallTimeLimit)) {    // whole machine timed out
                machine.changeRunMode(runModeChange.TIMEOUT);
                continue;
            }
            if (machine.endCriteria != null && machine.endCriteria.get()) {              // hit the end criteria which suppose is a success?
                machine.changeRunMode(runModeChange.END);
                continue;
            }
            if (machine.currentStep == -1) machine.nextStep();
            boolean doLoop;
            do {
                doLoop = false;
                if (machine.stepRunnable != null) machine.stepRunnable.run();   // run the runnable
                // The runnable might have caused the machine to stop or pause, so have to account for that in the code below!
                // Todo: Should paused advance to the next step if appropriate?
                if (!machine.running || machine.paused) {
                    continue;
                }
                if (machine.stepEnd.get() || timeout(machine.stepStartTime, machine.stepTimeLimit)) {   // meets end criteria or exceeds time
                    if (machine.abortOnTimeout && timeout(machine.stepStartTime, machine.stepTimeLimit)) {  // if abortOnTimeout, any timeout is a failure
                        machine.changeRunMode(runModeChange.ABORT);
                        continue;
                    }
                    if (machine.nextStep()) {            // if there's another step, do it
                        doLoop = true;
                    }
                    else {                               // no more steps = success
                        if (machine.autoReset) {
                            machine.tempNoStop = true;   // todo: what behavior is desired?
                            machine.changeRunMode(runModeChange.RESTART);
                        }
                        else {
                            machine.changeRunMode(runModeChange.COMPLETE);
                        }
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

    // todo: Add static ways to: stop all machines, pause all machines, stop/pause machines in groups, etc.

    public static void stopAll() {
        for (StateMachine machine : list) {
            machine.stop();
        }
    }

    public static void pauseAll() {
        for (StateMachine machine : list) {
            machine.pause();
            pausedAll = true;
        }
    }

    public static void unPauseAll() {
        if (!pausedAll) return;
        for (StateMachine machine : list) {
            machine.unPause();
            pausedAll = false;
        }
    }

    public static void stopGroups(String... names) {
        for (String sName : names) {
            for (StateMachine machine : list) {
                if (machine.memberGroup.isEmpty()) continue;
                for (String gName : machine.memberGroup) {
                    if (gName.equals(sName)) {
                        machine.stop();
                    }
                }
            }
        }
    }

    public static void pauseGroups(String... names) {
        // should probably only be one running (potentially each group)
        for (String sName : names) {
            for (StateMachine machine : list) {
                if (machine.memberGroup.isEmpty()) continue;
                for (String gName : machine.memberGroup) {
                    if (gName.equals(sName)) {
                        machine.pause();
                    }
                }
            }
        }
    }

    public static void unPauseGroups(String... names) {
        // should probably only be one paused (potentially each group)
        for (String sName : names) {
            for (StateMachine machine : list) {
                if (machine.memberGroup.isEmpty()) continue;
                for (String gName : machine.memberGroup) {
                    if (gName.equals(sName)) {
                        machine.unPause();
                    }
                }
            }
        }
    }

    public static void addTelemetry() {
        TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, "===== State Machines =====");
        for (StateMachine machine : list ) {
            TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, machine.name, machine.getStatus());
        }
        TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, "========================");
    }

    private void deconflict() {
        if (stopGroup.isEmpty()) return;                      // if stopgroup is empty, nothing to do
//        for (String sName : stopGroup) {
//            for (StateMachine machine : list) {
//                if (machine == this) continue;                // don't want to stop ourself
//                if (machine.memberGroup.isEmpty()) continue;  // if membergroup is empty, nothing to do
//                for (String gName : machine.memberGroup) {
//                    if (gName.equals(sName)) {
//                        machine.stop();
//                    }
//                }
//            }
//        }
        stopGroups(stopGroup.toArray(new String[0]));   // this will stop "this" machine as well, but then we restart so it's OK
    }

    static enum runState {
        RUNNING,
        PAUSED,
        STOPPED,
        COMPLETED,
        TIMEOUT,
        FAILED;
    }

    static enum runModeChange {
        TIMEOUT,
        START,
        RESTART,
        STOP,
        PAUSE,
        UNPAUSE,
        COMPLETE,
        ABORT,
        END;
    }

    public boolean stop() {
        return changeRunMode(runModeChange.STOP);
    }

    public boolean start() {
        return changeRunMode(runModeChange.START);
    }
//    public boolean start(boolean stop) {
//        tempNoStop = !stop;
//        return changeRunMode(runModeChange.START);
//    }
    public boolean startNoStop() {
        tempNoStop = true;
        return changeRunMode(runModeChange.START);
    }

    public boolean restart() {
        return changeRunMode(runModeChange.RESTART);
    }
//    public boolean restart(boolean stop) {  // this is useful when one task will call and wait for another
//        tempNoStop = !stop;
//        return changeRunMode(runModeChange.RESTART);
//    }
    public boolean restartNoStop() {
        tempNoStop = true;
        return changeRunMode(runModeChange.RESTART);
    }

    public boolean pause() {
        return changeRunMode(runModeChange.PAUSE);
    }

    public boolean unPause() {
        return changeRunMode(runModeChange.UNPAUSE);
    }

    private boolean changeRunMode (runModeChange mode) {
        switch (mode) {
            case START:
                if (paused) {
                    unPause();
                    return false;
                }
                if (running) {
                    return false;
                }
                //restart();
                //return true;
                // Continue on into case RESTART
            case RESTART:
                if (!tempNoStop) deconflict();
                tempNoStop = false;
                state = runState.RUNNING;
                running = true;
                paused = false;
                done = false;
                success = false;
                currentStep = -1;  //what about unpausing?
                overallStartTime = System.currentTimeMillis();
                stepStartTime = overallStartTime;
                return true;
            case STOP:
                returnStatus = false;
                if (running || paused) {
                    success = false;
                    if (stopRunnable != null) stopRunnable.run();  // do we have to check for null?
                    returnStatus = true;
                }
                if (!success) state = runState.STOPPED;  // leave as completed?
                running = false;
                paused = false;
                done = true; //???
                currentStep = -1;
                return returnStatus;
            case PAUSE:
                if (!paused && running) {
                    state = runState.PAUSED;
                    running = false;
                    paused = true;
                    pausedOverallTime = System.currentTimeMillis() - overallStartTime;
                    pausedStepTime = System.currentTimeMillis() - stepStartTime;
                    return true;
                }
                return false;
            case UNPAUSE:
                if (paused) {
                    state = runState.RUNNING;
                    running = true;
                    paused = false;
                    // todo: reconsider what to do about the timers. Should the timer statuses be saved upon pausing?
                    //// Here, we just reset them all the way
                    //overallStartTime = System.currentTimeMillis();
                    //stepStartTime = overallStartTime;
                    overallStartTime = System.currentTimeMillis() - pausedOverallTime;
                    stepStartTime = System.currentTimeMillis() - pausedStepTime;
                    return true;
                }
                return false;
            case TIMEOUT:  // whole machine
                state = runState.TIMEOUT;
                running = false;
                done = true; //???
                success = false;
                currentStep = -1;
                if (timeoutRunnable != null) timeoutRunnable.run();
                return true;
            case ABORT:   // single runnable
                state = runState.FAILED;  // or TIMEOUT?
                running = false;
                done = true;
                success = false;
                currentStep = -1;
                if (abortRunnable != null) abortRunnable.run();
                return true;
            case END:   // whole machine end criteria
                state = runState.COMPLETED;
                running = false;
                done = true;
                success = true;
                currentStep = -1;
                if (endCriteriaRunnable != null) endCriteriaRunnable.run();
                return true;
            case COMPLETE:  // all steps complete
                state = runState.COMPLETED;
                running = false;
                done = true;
                success = true;
                currentStep = -1;
                return true;
            default:
                return false;
        }
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

    public int getCurrentStep() {
        return currentStep;
    }

    public String getStatus() {
        return currentStep + ", " + state.toString();
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

    // add a step that runs once (no end state, no time limit); for cases where the Runnable can be confused with a Supplier
    public void addRunn(Runnable step) {
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
        stopGroup = new ArrayList<>(Arrays.asList(names));
    }

    public void setMemberGroups (String... names) {
        memberGroup = new ArrayList<>(Arrays.asList(names));
    }

    public void setStopGroups (String... names) {
        stopGroup = new ArrayList<>(Arrays.asList(names));
    }

    public void addGroup (String name) {
        memberGroup.add(name);
        stopGroup.add(name);
    }

    public void addMemberGroup (String name) {
        memberGroup.add(name);
    }

    public void addStopGroup (String name) {
        stopGroup.add(name);
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
