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
    String className;
    runState state = runState.STOPPED;
    boolean running = false;
    boolean paused = false;
    boolean done = false;
    boolean success = true;
    boolean autoReset = false;
    boolean abortOnTimeout = false;
    boolean returnStatus = false;
    boolean tempNoStop = false;
    boolean noBulkStop = false;  // this makes the task invincible except when stopped directly

    ArrayList<String> stopGroup = new ArrayList<>();
    ArrayList<String> memberGroup = new ArrayList<>();

    int currentStep = -1;
    long overallTimeLimit = 0;
    long overallStartTime;
    long stepStartTime;
    long pausedStepTime;
    long pausedOverallTime;

    Runnable stepRunnable;
    Supplier<Boolean> stepEnd;
    Long stepTimeLimit;
    Boolean stepAbort;

    Supplier<Boolean> endCriteria;
    ArrayList<Runnable> steps = new ArrayList<>();
    ArrayList<Supplier<Boolean>> ends = new ArrayList<>();
    ArrayList<Long> times = new ArrayList<>();
    ArrayList<Boolean> aborts = new ArrayList<>();

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
        this.className = getCallingClass();
    }

    public static void Reset() {
        list = new ArrayList<StateMachine>();
        map = new HashMap<>();
        pausedAll = false;
    }

    /*==================*/
    /*  Static Methods  */
    /*==================*/

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
                    if ((machine.abortOnTimeout || machine.stepAbort) && timeout(machine.stepStartTime, machine.stepTimeLimit)) {  // if abortOnTimeout, any timeout is a failure
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

    private static String getCallingClass() {
        String myName = StateMachine.class.getName();
        //RobotLog.vv("SM", "My Name: "+myName);
        String eName = "n/a";
        boolean foundMe = false;
        StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
        //int count = 0;
        for (StackTraceElement element : stackTrace) {
            eName = element.getClassName();
            //RobotLog.vv("SM", count+" "+eName);
            //count++;
            if (eName.equals(myName)) foundMe = true;
            if (foundMe && !eName.equals(myName)) break;
        }
        return shortName(eName);
        //return Thread.currentThread().getStackTrace()[4].getClassName();
    }

    private static String shortName(String fullName) {
        return fullName.substring(fullName.lastIndexOf(".") + 1);
    }

    public static void addTelemetry() {
        TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, "============ State Machines ============");
        for (StateMachine machine : list ) {
            TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, machine.name, machine.getStatus()+" ("+machine.className+")");
        }
        TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, "======================================");
    }

    /*=================*/
    /* Static Controls */
    /*=================*/

    public static void stopAll() {
        for (StateMachine machine : list) {
            if (!machine.noBulkStop) machine.stop();
        }
    }

    public static void pauseAll() {
        for (StateMachine machine : list) {
            if (!machine.noBulkStop) machine.pause();
        }
        pausedAll = true;
    }

    public static void unPauseAll() {
        if (!pausedAll) return;
        for (StateMachine machine : list) {
            machine.unPause();
        }
        pausedAll = false;
    }

    public static void stopGroups(String... names) {
        for (String sName : names) {
            for (StateMachine machine : list) {
                if (machine.memberGroup.isEmpty()) continue;
                for (String gName : machine.memberGroup) {
                    if (gName.equals(sName)) {
                        if (!machine.noBulkStop) machine.stop();
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
                        if (!machine.noBulkStop) machine.pause();
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

    public static void stopClass() {
        String cName = getCallingClass();
        for (StateMachine machine : list) {
            if (machine.className.equals(cName)) {
                if (!machine.noBulkStop) machine.stop();
            }
        }
    }

    public static void pauseClass() {
        String cName = getCallingClass();
        for (StateMachine machine : list) {
            if (machine.className.equals(cName)) {
                if (!machine.noBulkStop) machine.pause();
            }
        }
    }

    public static void unPauseClass() {
        String cName = getCallingClass();
        for (StateMachine machine : list) {
            if (machine.className.equals(cName)) {
                machine.unPause();
            }
        }
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

    /*==================*/
    /* Internal Methods */
    /*==================*/

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
                if (!tempNoStop) deConflict();
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

    private boolean nextStep() {
        currentStep++;
        if (currentStep >= steps.size()) return false;
        stepRunnable = steps.get(currentStep);
        stepEnd = ends.get(currentStep);
        stepTimeLimit = times.get(currentStep);
        stepAbort = aborts.get(currentStep);
        stepStartTime = System.currentTimeMillis();
        return true;
    }

    private void deConflict() {
        if (stopGroup.isEmpty()) return;                      // if stopgroup is empty, nothing to do
        stopGroups(stopGroup.toArray(new String[0]));   // this will stop "this" machine as well, but then we restart so it's OK
    }

    /*==============*/
    /*   Controls   */
    /*==============*/

    public boolean stop() {
        return changeRunMode(runModeChange.STOP);
    }

    public boolean start() {
        return changeRunMode(runModeChange.START);
    }

    public boolean startNoStop() {
        tempNoStop = true;
        return changeRunMode(runModeChange.START);
    }

    public boolean restart() {
        return changeRunMode(runModeChange.RESTART);
    }

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

    /*==============*/
    /*    Status    */
    /*==============*/

    public int getCurrentStep() {
        return currentStep;
    }

    public String getStatus() {
        return currentStep + ", " + state.toString();
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

    /*==============*/
    /*   Setters    */
    /*==============*/

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

    public void setNoBulkStop(boolean state) {
        noBulkStop = state;
    }

    /*==============*/
    /* Step Setting */
    /*==============*/

    // todo: rebuild this with aborts?
    public void addSteps(TaskStep... taskSteps) {
        for (TaskStep taskStep : taskSteps) {
            addStep(taskStep.step, taskStep.end, taskStep.time);
        }
    }

    public void addRunStep(Runnable step) {
        addStep(step, () -> true, 0, false);
    }
    public void addRunStep(Runnable step, Supplier<Boolean> end) {
        addStep(step, end, 0, false);
    }
    public void addRunStep(Runnable step, Supplier<Boolean> end, long time) {
        addStep(step, end, time, false);
    }
    public void addRunStep(Runnable step, Supplier<Boolean> end, long time, boolean abortOnTimeout) {
        addStep(step, end, time, abortOnTimeout);
    }
    public void addRunStep(Runnable step, long time) {
        addStep(step, () -> false, time, false);
    }

    public void addWaitFor(Supplier<Boolean> end) {
        addStep( () -> {}, end, 0, false);
    }
    public void addWaitFor(Supplier<Boolean> end, long time) {
        addStep( () -> {}, end, time, false);
    }
    public void addWaitFor(Supplier<Boolean> end, long time, boolean abortOnTimeout) {
        addStep( () -> {}, end, time, abortOnTimeout);
    }
    public void addWaitFor(long time) {
        addStep( () -> {}, () -> false, time, false);
    }

    public void addDelayOf(long time) {
        addStep( () -> {}, () -> false, time, false);
    }


    // add a step with all possible parameters
    public void addStep(Runnable step, Supplier<Boolean> end, long time, boolean abortOnTimeout) {
        if (step == null || end == null) return;
        if (time < 0) time = 0;
        if (time == 0) abortOnTimeout = false;
        steps.add(step);
        ends.add(end);
        times.add(time);
        aborts.add(abortOnTimeout);
    }

    // add a step with end state and time limit
    public void addStep(Runnable step, Supplier<Boolean> end, long time) {
        addStep(step, end, time, false);
    }

    // add a step with end state, no time limit
    public void addStep(Runnable step, Supplier<Boolean> end) {
        addStep(step, end, 0, false);
    }

    // add a step with only end state
    public void addStep(Supplier<Boolean> end) {
        addStep( () -> {}, end, 0, false);
    }

    // add a step end state and time limit
    public void addStep(Supplier<Boolean> end, long time) {
        addStep( () -> {}, end, time, false);
    }

    // add a step that runs once (no end state, no time limit)
    public void addStep(Runnable step) {
        addStep(step, () -> true, 0, false);
    }

    // add a step that runs once (no end state, no time limit); for cases where the Runnable can be confused with a Supplier
    public void addRunn(Runnable step) {
        addStep(step, () -> true, 0, false);
    }

    // add a step that is just a delay
    public void addStep(long time) {
        addStep( () -> {}, () -> false, time, false);
    }
    public void addDelay(long time) {
        addStep( () -> {}, () -> false, time, false);
    }

    // add a step that repeats for time
    public void addStep(Runnable step, long time) {
        addStep(step, () -> false, time, false);
    }

    // add a step with end state and time limit that aborts the task on timeout
    public void addAbort(Supplier<Boolean> end, long time) {
        addStep( () -> {}, end, time, true);
    }

    /*===============*/
    /* Group Setting */
    /*===============*/

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

    public void addGroups (String... names) {
        memberGroup.addAll(Arrays.asList(names));
        stopGroup.addAll(Arrays.asList(names));
    }

    public void addMemberGroups (String... names) {
        memberGroup.addAll(Arrays.asList(names));
    }

    public void addStopGroups (String... names) {
        stopGroup.addAll(Arrays.asList(names));
    }

    /**
     * Add @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
     * @param name
     */
    public void addStopGroup (String name) {
        stopGroup.add(name);
    }

    /*===================*/
    /* Special Runnables */
    /*===================*/

    /**
     * Specifies a runnable to be used when the running task is stopped.
     * @param run The runnable to run
     */
    public void setStopRunnable (Runnable run) {
        stopRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the task aborts due to step timeout.
     * Either abortOnTimeout must be set (for entire task) or individual step abort set with addAbort().
     * @param run The runnable to run
     */
    public void setAbortRunnable (Runnable run) {
        abortRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the overall task timeout occurs.
     * @param run The runnable to run
     */
    public void setTimeoutRunnable (Runnable run) {
        timeoutRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the overall task end criteria is met.
     * @param run The runnable to run
     */
    public void setEndCriteriaRunnable (Runnable run) {
        endCriteriaRunnable = run;
    }

}
