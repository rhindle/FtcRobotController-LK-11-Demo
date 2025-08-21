package org.firstinspires.ftc.teamcode.RobotParts.Common;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// todo: Add overall documentation for this class

/*

    Requirements

    Initialization - Before instantiating any StateMachine objects, reset() the class. This
    re-initializes the static tracking array, clearing out any references to previously created
    machines.
        StateMachine.reset();

    Periodic - Every iteration of your main LinearOpMode "run loop", runLoop() is required to
    process all of the StateMachine objects.
        StateMachine.runLoop();

    Telemetry - If desired, some basic debugging information can be added to the telemetry.
    Add this once to your periodic loop where you want the telemetry to be inserted.
    Pass your opMode.telemetry as the argument so it has access to the telemetry methods.
        StateMachine.addTelemetry([Telemetry telemetry]);

 */

// NOTE: Any Runnables must not be blocking! (Including any methods they call.)
//       By design, they are not started in separate threads,
//       so any delay will delay execution of the entire program.

public class StateMachine {

    /* Instance tracking */
    static ArrayList<StateMachine> list = new ArrayList<>();
    static boolean pausedAll = false;

    /* Internal variables */
    String name;
    String className;
    runState state = runState.UNUSED;
    boolean running = false;
    boolean paused = false;
    boolean done = false;
    boolean success = true;
    boolean autoRestart = false;     // this makes the machine self restart at the last step
    boolean abortOnTimeout = false;  // this makes any step timeout a failure of the machine
    boolean returnStatus = false;
    boolean tempNoStop = false;      // this makes Restart skip deConflict()
    boolean noBulkStop = false;      // this makes the task invincible except when stopped directly
    boolean lastStepTimeout = false; // status for potential use in the next step
    boolean forceNext = false;       // this makes the machine advance to the next step

    int currentStep = -1;
    long overallStartTime;
    long stepStartTime;
    long pausedStepTime;
    long pausedOverallTime;

    // Arrays to hold the step-related data
    ArrayList<Runnable> steps = new ArrayList<>();
    ArrayList<Supplier<Boolean>> ends = new ArrayList<>();
    ArrayList<Long> times = new ArrayList<>();
    ArrayList<Boolean> aborts = new ArrayList<>();
    ArrayList<Runnable> timeoutSteps = new ArrayList<>();

    // Settable machine data
    ArrayList<String> stopGroup = new ArrayList<>();
    ArrayList<String> memberGroup = new ArrayList<>();
    long overallTimeLimit = 0;
    Supplier<Boolean> endCriteria;
    Runnable stopRunnable;
    Runnable abortRunnable;
    Runnable timeoutRunnable;
    Runnable endCriteriaRunnable;
    Runnable pauseRunnable;
    Runnable unPauseRunnable;
    // should the runnables be packed into an array based on runModeChange?

    // Stored data for current step being processed
    Runnable stepRunnable;
    Supplier<Boolean> stepEnd;
    Long stepTimeLimit;
    Boolean stepAbort;
    Runnable stepTimeoutRunnable;

    // Future use? Public for access where the runnables are created.
    public boolean bool1, bool2, bool3;
    public long long1, long2, long3;
    public double dub1, dub2, dub3;

    /**
     * Construct a new state machine.
     * @param name The name of the machine (used primarily for telemetry debugging)
     */
    public StateMachine(String name) {
        list.add(this);
        this.name = name;
        this.className = getCallingClass();
    }

    /*==================*/
    /*  Static Methods  */
    /*==================*/

    /**
     * Reset the static variables (otherwise the lists keep growing).
     * Only to be called near the beginning of the opMode before the new machines are instantiated.
     */
    public static void reset() {
        list = new ArrayList<StateMachine>();
        pausedAll = false;
    }

    /**
     * The main loop must be called periodically each loop of the OpMode.
     * This processes all of the state machines.
     */
    public static void runLoop() {

        // Loop through all of the state machines.
        for (StateMachine machine : list ) {

            // If the state machine isn't running, no further processing necessary.
            if (!machine.running) continue;

            // Check if the overall machine timeout (if set) has been exceeded.
            if (timeout(machine.overallStartTime, machine.overallTimeLimit)) {
                machine.changeRunMode(runModeChange.TIMEOUT);
                continue;
            }

            // This loop if for iterating within a single machine, allowing several steps to
            // be performed in quick succession (if there isn't a timeout or end criteria
            // being waited for) rather than waiting for the next full loop.
            boolean doLoop;
            do {
                doLoop = false;

                // If the machine is restarting, load the first step into the step "buffer" variables.
                if (machine.currentStep == -1) {
                    machine.lastStepTimeout = false;
                    machine.loadNextStep();
                }

                // Check if the machine ending criteria (if set) has been met.
                // Depending on the use, this could be a success or a failure.
                if (machine.endCriteria != null && machine.endCriteria.get()) {
                    machine.changeRunMode(runModeChange.ENDCONDITION);
                    continue;
                }

                // Run the runnable associated with the step.
                // (The runnable might cause the machine to stop or pause.)
                if (machine.stepRunnable != null) machine.stepRunnable.run();
                if (!machine.running || machine.paused) continue;

                boolean stepTimedOut = timeout(machine.stepStartTime, machine.stepTimeLimit);

                // Check for abort associated with step timeout.
                // Note that abortOnTimeout, if true, makes any timeout a failure.
                if ((machine.abortOnTimeout || machine.stepAbort) && stepTimedOut) {
                    machine.changeRunMode(runModeChange.ABORT);
                    continue;
                }

                // Advance step if end criteria met or step timeout.
                if (machine.stepEnd.get() || stepTimedOut || machine.forceNext) {

                    // Store whether the step timed out.
                    machine.lastStepTimeout = stepTimedOut;

                    // if the step timed out, run the step timeout runnable if it exists.  (Not the same as an abort!)
                    if (stepTimedOut && machine.stepTimeoutRunnable != null) machine.stepTimeoutRunnable.run();

                    // If there's another step, load it and loop again.
                    if (machine.loadNextStep()) {
                        doLoop = true;
                    }
                    // If there isn't a next step, restart or enter FINISHED state.
                    else {
                        if (machine.autoRestart) {
                            machine.tempNoStop = true;   // todo: what behavior is desired?
                            machine.changeRunMode(runModeChange.RESTART);
                            doLoop = true;
                        }
                        else {
                            machine.changeRunMode(runModeChange.FINISH);
                        }
                    }
                }
            } while (doLoop);
        }
    }

    /**
     * Calculate if a timeout has occurred based on the start time and limit supplied.
     * @param start The start time for calculating if the timeout has occurred.
     * @param limit The time limit (ms)
     * @return True if the timeout has occurred.
     */
    private static boolean timeout (long start, long limit) {
        if (limit == 0) return false;
        return System.currentTimeMillis() - start >= limit;
    }

    /**
     * Get the name of the calling class.
     * Used for control (stop, pause, unpause) of machines belonging to the same class.
     * Also used for telemetry debugging.
     * For internal use.
     * @return The full name of the calling class.
     */
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

    /**
     * Get the short name of a class for use in telemetry. For internal use.
     * @param fullName The full name of the class (separated by dots).
     * @return The short name of the class (last part of the name after the final dot).
     */
    private static String shortName(String fullName) {
        return fullName.substring(fullName.lastIndexOf(".") + 1);
    }

    /**
     * Add state machine status to telemetry for debugging.
     */
    public static void addTelemetry() {
        TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, "============ State Machines ============");
        for (StateMachine machine : list ) {
            TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, machine.name, machine.getStatus()+" ("+machine.className+")");
        }
        TelemetryMgr.message(TelemetryMgr.Category.TASK_EXT, "======================================");
    }
    public static void addTelemetry(LinearOpMode opMode) {
        opMode.telemetry.addData("=========== State Machines ===========", "");
        for (StateMachine machine : list ) {
            opMode.telemetry.addData(machine.name, machine.getStatus()+" ("+machine.className+")");
        }
        opMode.telemetry.addData("====================================", "");
    }
    public static void addTelemetry(Telemetry telemetry) {
        telemetry.addData("=========== State Machines ===========", "");
        for (StateMachine machine : list ) {
            telemetry.addData(machine.name, machine.getStatus()+" ("+machine.className+")");
        }
        telemetry.addData("====================================", "");
    }

    /*=================*/
    /* Static Controls */
    /*=================*/

    /**
     * Stop all machines that are currently running.
     * Machines that have noBulkStop set to True will not be stopped.
     */
    public static void stopAll() {
        for (StateMachine machine : list) {
            if (!machine.noBulkStop) machine.stop();
        }
    }

    /**
     * Pause all machines that are currently running.
     * Machines that have noBulkStop set to True will not be paused.
     */
    public static void pauseAll() {
        for (StateMachine machine : list) {
            if (!machine.noBulkStop) machine.pause();
        }
        pausedAll = true;
    }

    /**
     * Unpause all machines that are currently paused.
     * The machines must previously have been paused with the pauseAll() method, as tracked by the pausedAll variable.
     */
    public static void unPauseAll() {
        if (!pausedAll) return;
        for (StateMachine machine : list) {
            machine.unPause();
        }
        pausedAll = false;
    }

    /**
     * Stop all machines that are members of the specified groups.
     * @param names The names of the groups to stop.
     */
    public static void stopGroups(String... names) {
        controlGroups(runModeChange.STOP, names, null);
    }

    /**
     * Pause all machines that are members of the specified groups.
     * @param names The names of the groups to pause.
     */
    public static void pauseGroups(String... names) {
        // should probably only be one running (potentially each group)
        controlGroups(runModeChange.PAUSE, names, null);
    }

    /**
     * Unpause all machines that are members of the specified groups.
     * @param names The names of the groups to unpause.
     */
    public static void unPauseGroups(String... names) {
        // should probably only be one paused (potentially each group)
        controlGroups(runModeChange.UNPAUSE, names, null);
    }

    /**
     * For internal use to avoid code duplication. Controls groups of state machines.
     * @param mode The mode to process (STOP, PAUSE, or UNPAUSE)
     * @param names Array of group names to control
     * @param caller The calling state machine (not static), for use only with deConflict()
     */
    private static void controlGroups(runModeChange mode, String[] names, StateMachine caller) {
        if (mode != runModeChange.STOP
                && mode != runModeChange.PAUSE
                && mode != runModeChange.UNPAUSE) return;
        for (String sName : names) {
            for (StateMachine machine : list) {
                if (caller != null && machine == caller) continue; // Don't want to control calling machine using deConflict() because it will trigger stopRunnable (if set)
                if (machine.memberGroup.isEmpty()) continue;
                for (String gName : machine.memberGroup) {
                    if (gName.equals(sName)) {
                        switch (mode) {
                            case STOP:
                                if (!machine.noBulkStop) machine.stop();
                                break;
                            case PAUSE:
                                if (!machine.noBulkStop) machine.pause();
                                break;
                            case UNPAUSE:
                                machine.unPause();
                                break;
                        }
                    }
                }
            }
        }
    }

    /**
     * Stop all machines belonging to the calling class.
     */
    public static void stopByClass() {
        String cName = getCallingClass();
        for (StateMachine machine : list) {
            if (machine.className.equals(cName)) {
                if (!machine.noBulkStop) machine.stop();
            }
        }
    }

    /**
     * Pause all machines belonging to the calling class.
     */
    public static void pauseByClass() {
        String cName = getCallingClass();
        for (StateMachine machine : list) {
            if (machine.className.equals(cName)) {
                if (!machine.noBulkStop) machine.pause();
            }
        }
    }

    /**
     * Unpause all machines belonging to the calling class.
     */
    public static void unPauseByClass() {
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
        FINISHED,
        COMPLETED,
        TIMEOUT,
        ENDED,
        FAILED,
        UNUSED;
    }

    static enum runModeChange {
        TIMEOUT,
        START,
        RESTART,
        STOP,
        PAUSE,
        UNPAUSE,
        FINISH,
        ABORT,
        END,
        ENDCONDITION;
    }

    /*==================*/
    /* Internal Methods */
    /*==================*/

    /**
     * Update the "Run Mode" of the machine, setting variables as necessary.
     * For internal use.
     * @param mode The run mode to change to.
     */
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
                    if (pauseRunnable != null) pauseRunnable.run();
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
                    if (unPauseRunnable != null) unPauseRunnable.run();
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
            case END:   // self-ended
                state = runState.ENDED;
                running = false;
                done = true;
                success = true;
                currentStep = -1;
                return true;
            case ENDCONDITION:   // whole machine end criteria
                state = runState.COMPLETED;
                running = false;
                done = true;
                success = true;
                currentStep = -1;
                if (endCriteriaRunnable != null) endCriteriaRunnable.run();
                return true;
            case FINISH:  // all steps complete
                state = runState.FINISHED;
                running = false;
                done = true;
                success = true;
                currentStep = -1;
                return true;
            default:
                return false;
        }
    }

    /**
     * Advance to the next step.
     * Set current "buffer" variables for the step runnable, end condition, time limit, and abort state.
     * For internal use.
     * @return True if there is another step, False if there are no more steps.
     */
    private boolean loadNextStep() {
        currentStep++;
        forceNext = false;
        if (currentStep >= steps.size()) return false;
        stepRunnable = steps.get(currentStep);
        stepEnd = ends.get(currentStep);
        stepTimeLimit = times.get(currentStep);
        stepAbort = aborts.get(currentStep);
        stepTimeoutRunnable = timeoutSteps.get(currentStep);
        stepStartTime = System.currentTimeMillis();
        return true;
    }

    /**
     * For internal use. Stops all machines in the same group.
     * Uses the same method as stopGroups(), except includes the calling machine as a parameter
     * for an additional check to prevent stopping itself and triggering its stopRunnable if set.
     */
    private void deConflict() {
        if (stopGroup.isEmpty()) return;                      // If stopgroup is empty, nothing to do
        controlGroups(runModeChange.STOP, stopGroup.toArray(new String[0]), this);
    }

    /*==============*/
    /*   Controls   */
    /*==============*/

    /**
     * Stop a machine.
     * @return True if running or paused. False if not needing to be stopped.
     */
    public boolean stop() {
        return changeRunMode(runModeChange.STOP);
    }

    /**
     * Start a machine. If not running, restart. If paused, unpause.
     * @return True if restarted, False if unpaused or already running.
     */
    public boolean start() {
        return changeRunMode(runModeChange.START);
    }

    /**
     * Start a machine, skipping the deConflict() step if restarting.
     * Useful for running another machine in the same group and waiting for it to finish.
     * Otherwise, when it restarted, it would stop the calling machine.
     */
    public boolean startNoStop() {
        tempNoStop = true;
        return changeRunMode(runModeChange.START);
    }

    /**
     * Restart the machine.
     */
    public boolean restart() {
        return changeRunMode(runModeChange.RESTART);
    }

    /**
     * Restart a machine, skipping the deConflict() step.
     * Useful for running another machine in the same group and waiting for it to finish.
     * Otherwise, when it restarted, it would stop the calling machine.
     */
    public boolean restartNoStop() {
        tempNoStop = true;
        return changeRunMode(runModeChange.RESTART);
    }

    /**
     * Pause the machine if it is in a running state and not already paused.
     * @return True if successfully paused, false if not running or already paused.
     */
    public boolean pause() {
        return changeRunMode(runModeChange.PAUSE);
    }

    /**
     * Unpause (resume) the machine if it is in a paused state.
     * @return True if unpaused, false if not in a paused state.
     */
    public boolean unPause() {
        return changeRunMode(runModeChange.UNPAUSE);
    }

    /**
     * "End" the machine. Similar to stop(), but without triggering any runnables.
     * Primarily intended for use within a machine to end early under some condition.
     */
    public boolean end() {
        return changeRunMode(runModeChange.END);
    }

    /**
     * Cause the machine to advance to the next step. Primitive flow control for use in a
     * step runnable in certain cases where the logic may be simpler than using
     * an exit criteria.
     */
    public void nextStep() {
        forceNext = true;
    }

    /**
     * Changes the current step pointer to the specified step number.
     * This intended to allow a certain amount of flow control within a machine.
     * However, the step numbers will need to manually counted and updated.
     * @param stepNumber The step number (index 0..n) to jump to.
     */
    public void gotoStep (int stepNumber) {
        if (stepNumber < 0) return;
        if (stepNumber >= steps.size()) return;
        currentStep = stepNumber - 1;   // intent is for this to only be used within the machine; step counter will increment by one
        forceNext = true;
    }

    /**
     * Adjusts the current step pointer by the specified number.
     * This intended to allow a certain amount of flow control within a machine.
     * @param stepAdd The number to increment (or decrement if negative) the step index (currentStep) by.
     */
    public void gotoStepRelative (int stepAdd) {
        currentStep += stepAdd;
        if (currentStep < 0) currentStep = 0;
        if (currentStep > steps.size()) currentStep = steps.size();
        currentStep--;       // intent is for this to only be used within the machine; step counter will increment by one
        forceNext = true;
    }

    /*==============*/
    /*    Status    */
    /*==============*/

    /**
     * @return The current step number.
     */
    public int getCurrentStep() {
        return currentStep;
    }

    /**
     * @return The step number and current state, in String format. Used for telemetry.
     */
    public String getStatus() {
        return currentStep + ", " + state.toString();
    }

    /**
     * @return True if the machine is "done" (finished, completed, ended, aborted) and not running.
     */
    public boolean isDone() {
        return done;
        //return state == runState.FINISHED;
    }

    /**
     * @return True if the machine successfully finished all steps or completed the end condition.
     */
    public boolean isSuccess() {
        return success;
        //return state == runState.FINISHED;
    }

    /**
     * @return True if the machine is currently in a running state.
     */
    public boolean isRunning() {
        return running;
        //return state == runState.RUNNING;
    }

    /**
     * @return True if the machine has timed out.
     */
    public boolean isTimedOut() {
        return state == runState.TIMEOUT;
    }

    /**
     * @return True if the previous step timed out.
     */
    public boolean lastStepTimeout() {
        return lastStepTimeout;
    }

    /*==============*/
    /*   Setters    */
    /*==============*/

    /**
     * The machine will automatically restart (loop) when it finishes the last step.
     * @param state If true, the machine will restart when it finishes.
     * @return this, for method chaining.
     */
    public StateMachine setAutoRestart(boolean state) {
        autoRestart = state;
        return this;
    }

    /**
     * The machine will stop when the time limit is reached for ANY step and will have a state of ABORT.
     * @param state If true, the machine will abort upon timeout of any step.
     * @return this, for method chaining.
     */
    public StateMachine setAbortOnTimeout(boolean state) {
        abortOnTimeout = state;
        return this;
    }

    /**
     * Sets the time limit for the entire machine, zeroed at restart.
     * The machine will stop when this time limit is reached and will have a state of TIMEOUT.
     * @param timeLimit The time limit (ms) for the entire machine.
     * @return this, for method chaining.
     */
    public StateMachine setTimeLimit (long timeLimit) {
        // sanitize input?
        overallTimeLimit = timeLimit;
        return this;
    }

    /**
     * Sets the end condition for the machine. The machine will stop when this condition is met and will have a state of COMPLETED.
     * @param endCriteria The end condition for the entire machine.
     */
    public void setEndCriteria(Supplier<Boolean> endCriteria) {
        this.endCriteria = endCriteria;
    }

    /**
     * Sets the bulk stop state. If true, the machine will only be stopped individually with the stop() method.
     * It will not be stopped as part of a group or class. This is intended for machines that are "safe" and/or
     * need to run continuously for any reason.
     * @param state If true, the machine will not be stopped by group, class, or other bulk stop.
     */
    public void setNoBulkStop(boolean state) {
        noBulkStop = state;
    }

    /*==============*/
    /* Step Setting */
    /*==============*/

    /**
     * Add multiple steps at once as TaskStep objects.
     * A potential use for this is preparing a set of steps that can be reused in several state machines.
     * @param taskSteps The TaskStep objects to add as steps to the state machine.
     */
    public void addSteps(TaskStep... taskSteps) {
        for (TaskStep taskStep : taskSteps) {
            addStep(taskStep.step, taskStep.end, taskStep.time, taskStep.abort);
        }
    }

    /**
     * Add a step that runs a single time.
      * @param runnable The runnable to run.
     */
    public void addRunOnce(Runnable runnable) {
        addStep(runnable, () -> true, 0, false);
    }

    /**
     * Add a step that runs repeatedly until the end condition is met.
     * @param runnable The runnable to run.
     * @param endCondition The end condition to advance to the next step.
     */
    public void addRunPlus(Runnable runnable, Supplier<Boolean> endCondition) {
        addStep(runnable, endCondition, 0, false);
    }

    /**
     * Add a step that runs repeatedly until the end condition is met or a certain amount of time passes.
     * @param runnable The runnable to run.
     * @param endCondition The end condition to advance to the next step.
     * @param timeOut The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     */
    public void addRunPlus(Runnable runnable, Supplier<Boolean> endCondition, long timeOut) {
        addStep(runnable, endCondition, timeOut, false);
    }

    /**
     * Add a step that runs repeatedly until the end condition is met or a certain amount of time passes,
     * plus a runnable to run if the timeout is reached before the end condition is met.
     * @param runnable The runnable to run.
     * @param endCondition The end condition to advance to the next step.
     * @param timeOut The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     * @param timeoutStep The runnable to run if the timeout is reached.
     */
    public void addRunPlus(Runnable runnable, Supplier<Boolean> endCondition, long timeOut, Runnable timeoutStep) {
        addStep(runnable, endCondition, timeOut, false, timeoutStep);
    }

    /**
     * Add a step that runs repeatedly until the end condition is met or a certain amount of time passes.
     * If abortOnTimeout is true, the machine will abort upon timeout.
     * @param runnable The runnable to run.
     * @param endCondition The end condition to advance to the next step.
     * @param timeOut The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     * @param abortOnTimeout If true, the machine will "abort" (stop) if the timeout is reached.
     */
    public void addRunPlus(Runnable runnable, Supplier<Boolean> endCondition, long timeOut, boolean abortOnTimeout) {
        addStep(runnable, endCondition, timeOut, abortOnTimeout);
    }

    /**
     * Add a step that runs repeatedly for a certain amount of time.
     * @param runnable The runnable to run.
     * @param time The amount of time (ms) to wait before advancing to the next step.
     */
    public void addRunPlus(Runnable runnable, long time) {
        addStep(runnable, () -> false, time, false);
    }

    /**
     * Add a step that waits for an end condition to be met (true).
     * @param endCondition The end condition to advance to the next step.
     */
    public void addWaitFor(Supplier<Boolean> endCondition) {
        addStep( () -> {}, endCondition, 0, false);
    }

    /**
     * Add a step that waits for an end condition to be met (true) or a certain amount of time to pass.
     * @param endCondition The end condition to advance to the next step.
     * @param timeOut The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     */
    public void addWaitFor(Supplier<Boolean> endCondition, long timeOut) {
        addStep( () -> {}, endCondition, timeOut, false);
    }

    /**
     * Add a step that waits for an end condition to be met (true) or a certain amount of time to pass,
     * plus a runnable to run if the timeout is reached before the end condition is met.
     * @param endCondition The end condition to advance to the next step.
     * @param timeOut The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     * @param timeoutStep The runnable to run if the timeout is reached.
     */
    public void addWaitFor(Supplier<Boolean> endCondition, long timeOut, Runnable timeoutStep) {
        addStep( () -> {}, endCondition, timeOut, false, timeoutStep);
    }

    /**
     * Add a step that waits for an end condition to be met (true) or a certain amount of time to pass.
     * If abortOnTimeout is true, the machine will abort upon timeout.
     * @param endCondition The end condition to advance to the next step.
     * @param timeOut The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     * @param abortOnTimeout If true, the machine will "abort" (stop) if the timeout is reached.
     */
    public void addWaitFor(Supplier<Boolean> endCondition, long timeOut, boolean abortOnTimeout) {
        addStep( () -> {}, endCondition, timeOut, abortOnTimeout);
    }

    /**
     * Add a step that waits for a certain amount of time to pass.
     * @param time The amount of time (ms) to wait.
     */
    public void addWaitFor(long time) {
        addStep( () -> {}, () -> false, time, false);
    }

    /**
     * Add a step that waits for a certain amount of time to pass.
     * @param time The amount of time (ms) to wait.
     */
    public void addDelayOf(long time) {
        addStep( () -> {}, () -> false, time, false);
    }

    /**
     * Add a step with all possible parameters. Primarily for internal use.
     * @param step The runnable to run.
     * @param end The end condition to advance to the next step.
     * @param time The amount of time (ms) to wait before advancing to the next step (without meeting the end condition).
     * @param abortOnTimeout If true, the machine will "abort" (stop) if the timeout is reached.
     * @param timeoutStep The runnable to run if the timeout is reached, but abort is not set.
     */
    public void addStep(Runnable step, Supplier<Boolean> end, long time, boolean abortOnTimeout, Runnable timeoutStep) {
        if (step == null || end == null) return;
        if (time < 0) time = 0;
        if (time == 0) abortOnTimeout = false;
        steps.add(step);
        ends.add(end);
        times.add(time);
        aborts.add(abortOnTimeout);
        timeoutSteps.add(timeoutStep);
    }

    public void addStep(Runnable step, Supplier<Boolean> end, long time, boolean abortOnTimeout) {
        addStep(step, end, time, abortOnTimeout, () -> {});
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

    /**
     * Sets names of groups that this machine belongs to.
     * They will be stopped when this machine is restarted,
     * and this machine will be stopped when they are restarted.
     * @param names The names of the groups this machine belongs to.
     */
    public void setGroups (String... names) {
        memberGroup = new ArrayList<>(Arrays.asList(names));
        stopGroup = new ArrayList<>(Arrays.asList(names));
    }

    /**
     * Sets names of groups to that stop this machine when they are restarted.
     * @param names The names of the groups to be stopped by.
     */
    public void setMemberGroups (String... names) {
        memberGroup = new ArrayList<>(Arrays.asList(names));
    }

    /**
     * Sets the names of groups to stop when this machine is restarted.
     * @param names The names of the groups to stop.
     */
    public void setStopGroups (String... names) {
        stopGroup = new ArrayList<>(Arrays.asList(names));
    }

    /**
     * Adds names of groups that this machine belongs to.
     * They will be stopped when this machine is restarted,
     * and this machine will be stopped when they are restarted.
     * @param names The names of the groups this machine belongs to.
     */
    public void addGroups (String... names) {
        memberGroup.addAll(Arrays.asList(names));
        stopGroup.addAll(Arrays.asList(names));
    }

    /**
     * Adds names of groups to that stop this machine when they are restarted.
     * @param names The names of the groups to be stopped by.
     */
    public void addMemberGroups (String... names) {
        memberGroup.addAll(Arrays.asList(names));
    }

    /**
     * Adds names of groups to stop when this machine is restarted.
     * @param names The names of the groups to stop.
     */
    public void addStopGroups (String... names) {
        stopGroup.addAll(Arrays.asList(names));
    }

    /*=======================*/
    /* Mode Change Runnables */
    /*=======================*/

    /**
     * Specifies a runnable to be used when the running task is stopped.
     * @param run The runnable to run.
     */
    public void setStopRunnable (Runnable run) {
        stopRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the task aborts due to step timeout.
     * Either abortOnTimeout must be set (for entire task) or
     * individual step created with with abortOnTimeout parameter true.
     * @param run The runnable to run.
     */
    public void setAbortRunnable (Runnable run) {
        abortRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the overall task timeout occurs.
     * @param run The runnable to run.
     */
    public void setTimeoutRunnable (Runnable run) {
        timeoutRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the overall task end criteria is met.
     * @param run The runnable to run.
     */
    public void setEndCriteriaRunnable (Runnable run) {
        endCriteriaRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the running task is paused.
     * @param run The runnable to run.
     */
    public void setPauseRunnable (Runnable run) {
        pauseRunnable = run;
    }

    /**
     * Specifies a runnable to be used when the running task is unpaused.
     * @param run The runnable to run.
     */
    public void setUnPauseRunnable (Runnable run) {
        unPauseRunnable = run;
    }

    /*================*/
    /* TaskStep class */
    /*================*/

    /**
     * Class for creating steps that can be added to a state machine with addSteps().
     */
    public static class TaskStep {

        public Runnable step;
        public Supplier<Boolean> end;
        public long time;
        public boolean abort = false;
        public Runnable timeoutStep;

        public TaskStep() {
            this.step = () -> {};
            this.end = () -> true;
            this.time = 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step with all properties
        public TaskStep(Runnable step, Supplier<Boolean> end, long time, boolean abort, Runnable timeoutStep) {
            this.step = (step != null) ? step : () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = abort;
            this.timeoutStep = (timeoutStep != null) ? timeoutStep : () -> {};
        }

        // create a step with end state, time limit, and abort state
        public TaskStep(Runnable step, Supplier<Boolean> end, long time, boolean abort) {
            this.step = (step != null) ? step : () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = abort;
            this.timeoutStep = () -> {};
        }

        // create a step with end state and time limit
        public TaskStep(Runnable step, Supplier<Boolean> end, long time) {
            this.step = (step != null) ? step : () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step with end state, time limit, and timeout step
        public TaskStep(Runnable step, Supplier<Boolean> end, long time, Runnable timeoutStep) {
            this.step = (step != null) ? step : () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = false;
            this.timeoutStep = (timeoutStep != null) ? timeoutStep : () -> {};
        }

        // create a step with end state, no time limit
        public TaskStep(Runnable step, Supplier<Boolean> end) {
            this.step = (step != null) ? step : () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step with only end state
        public TaskStep(Supplier<Boolean> end) {
            this.step = () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step with end state and time limit
        public TaskStep(Supplier<Boolean> end, long time) {
            this.step = () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step with end state, time limit, and timeout step
        public TaskStep(Supplier<Boolean> end, long time, Runnable timeoutStep) {
            this.step = () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = false;
            this.timeoutStep = (timeoutStep != null) ? timeoutStep : () -> {};
        }

        // create a step with end state, time limit, and abort state
        public TaskStep(Supplier<Boolean> end, long time, boolean abort) {
            this.step = () -> {};
            this.end = (end != null) ? end : () -> true;
            this.time = (time >= 0) ? time : 0;
            this.abort = abort;
            this.timeoutStep = () -> {};
        }

        // create a step that runs once (no end state, no time limit)
        public TaskStep(Runnable step) {
            this.step = (step != null) ? step : () -> {};
            this.end = () -> true;
            this.time = 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step that is just a delay
        public TaskStep(long time) {
            this.step = () -> {};
            this.end = () -> false;
            this.time = (time >= 0) ? time : 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        // create a step that repeats for time
        public TaskStep(Runnable step, long time) {
            this.step = (step != null) ? step : () -> {};
            this.end = () -> false;
            this.time = (time >= 0) ? time : 0;
            this.abort = false;
            this.timeoutStep = () -> {};
        }

        @NonNull
        public TaskStep clone() {
            return new TaskStep(step, end, time, abort, timeoutStep);
        }

    }

}