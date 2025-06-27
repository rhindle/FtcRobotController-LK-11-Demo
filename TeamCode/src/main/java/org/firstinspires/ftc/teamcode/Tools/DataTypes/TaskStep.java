package org.firstinspires.ftc.teamcode.Tools.DataTypes;

import java.util.function.Supplier;

import androidx.annotation.NonNull;

public class TaskStep {

    public Runnable step;
    public Supplier<Boolean> end;
    public long time;
    public boolean abort = false;

    public TaskStep() {
        this.step = () -> {};
        this.end = () -> true;
        this.time = 0;
        this.abort = false;
    }

    // create a step with end state, time limit, and abort state
    public TaskStep(Runnable step, Supplier<Boolean> end, long time, boolean abort) {
        this.step = (step != null) ? step : () -> {};
        this.end = (end != null) ? end : () -> true;
        this.time = (time >= 0) ? time : 0;
        this.abort = abort;
    }

    // create a step with end state and time limit
    public TaskStep(Runnable step, Supplier<Boolean> end, long time) {
        this.step = (step != null) ? step : () -> {};
        this.end = (end != null) ? end : () -> true;
        this.time = (time >= 0) ? time : 0;
        this.abort = false;
    }

    // create a step with end state, no time limit
    public TaskStep(Runnable step, Supplier<Boolean> end) {
        this.step = (step != null) ? step : () -> {};
        this.end = (end != null) ? end : () -> true;
        this.time = 0;
        this.abort = false;
    }

    // create a step with only end state
    public TaskStep(Supplier<Boolean> end) {
        this.step = () -> {};
        this.end = (end != null) ? end : () -> true;
        this.time = 0;
        this.abort = false;
    }

    // create a step with end state and time limit
    public TaskStep(Supplier<Boolean> end, long time) {
        this.step = () -> {};
        this.end = (end != null) ? end : () -> true;
        this.time = (time >= 0) ? time : 0;
        this.abort = false;
    }

    // create a step with end state, time limit, and abort state
    public TaskStep(Supplier<Boolean> end, long time, boolean abort) {
        this.step = () -> {};
        this.end = (end != null) ? end : () -> true;
        this.time = (time >= 0) ? time : 0;
        this.abort = abort;
    }

    // create a step that runs once (no end state, no time limit)
    public TaskStep(Runnable step) {
        this.step = (step != null) ? step : () -> {};
        this.end = () -> true;
        this.time = 0;
        this.abort = false;
    }

    // create a step that is just a delay
    public TaskStep(long time) {
        this.step = () -> {};
        this.end = () -> false;
        this.time = (time >= 0) ? time : 0;
        this.abort = false;
    }

    // create a step that repeats for time
    public TaskStep(Runnable step, long time) {
        this.step = (step != null) ? step : () -> {};
        this.end = () -> false;
        this.time = (time >= 0) ? time : 0;
        this.abort = false;
    }

    @NonNull
    public TaskStep clone() {
        return new TaskStep(step, end, time, abort);
    }

}
