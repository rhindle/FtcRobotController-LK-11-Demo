package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoWrapper implements Servo {

    private Servo servo;
    private double offset = 0.0;
    private boolean enabled = false;
    private int sweepTime = 1500;
    private long timer = System.currentTimeMillis();
    private boolean eStopped = false;

    public ServoWrapper(Servo servo) {
        this.servo = servo;
    }

    // settings

     /**
     * Sets an offset value for the servo that will be subtracted with setting a position with setPosition()
     * @param offset the offset to subtracted
     * @return this for method chaining
     */
    public ServoWrapper setOffset(double offset) {
        this.offset = offset;
        return this;
    }

    /**
     * Sets the servo sweep time (full time to traverse from minimum to maximum position; e.g., -150° to 150°).
     * The value supplied should account for how the servo is loaded.
     * @param sweepTime the time in ms
     * @return this for method chaining
     */
    public ServoWrapper setSweepTime(int sweepTime) {
        this.sweepTime = sweepTime;
        return this;
    }

    /**
     * Sets the servo Pwm range to the maximum; i.e., 500-2500 μs
     * @return this for method chaining
     */
    public ServoWrapper setFullPwmRange() {
        ((ServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        return this;
    }

    /**
     * Sets the servo Pwm range
     * @param low the low end of the Pwm range in μs
     * @param high the high end of the Pwn range in μs
     * @return this for method chaining
     */
    public ServoWrapper setPwmRange(double low, double high) {
        // could use some checking
        ((ServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(low, high));
        return this;
    }

    // enabling and disabling pwm

    /**
     * Emergency Stop: Disables the Pwm signal for the servo such that the position is assumed to be lost/unknown.
     */
    public void eStop() {
        disable();
        eStopped = true;  // we no longer know where the servo is, so need to time accordingly next move
    }

    /**
     * Disables the Pwm signal for the servo. Servo behavior may vary; goBilda servos will power down.
     * (Will automatically re-enable when another position is set.)
     */
    public void disable() {
        //((ServoControllerEx) getController()).setServoPwmDisable(getPortNumber());
        ((ServoImplEx) servo).setPwmDisable();
        this.enabled = false;
    }

    /**
     * Enables the Pwm signal for the servo. (Will automatically re-enable when another position is set.)
     */
    public void enable() {
        //((ServoControllerEx) getController()).setServoPwmEnable(getPortNumber());
        ((ServoImplEx) servo).setPwmEnable();
        this.enabled = true;
    }

    // status responders

    /**
     * Gets the stored offset value
     * @return the offset that is subtracted when setting position
     */
    public double getOffset() {
        return this.offset;
    }

    /**
     * Gets the servo position last set, accounting for the offset
     * @return the servo position + offset
     */
    public double getPositionWithOffset() {
        return getPosition() + offset;
    }

    /**
     * Determine if the timer associated with the servo movement is complete (i.e., servo is expected to be finished moving)
     * @return TRUE if the time is complete
     */
    public boolean isTimerDone() {
        return System.currentTimeMillis() >= this.timer;
    }

    /**
     * Determine if the servo is set to a certain position and the associated timer is complete
     * @param comparePosition the position to check against the actual set position
     * @return TRUE if both the servo is set to that position and the time is complete
     */
    public boolean isAtPositionWithTime(double comparePosition) {
        return isAtPosition(comparePosition) && isTimerDone();
    }

    /**
     * Determine if the servo is set to a certain position (not accounting for the timer)
     * @param comparePosition the position to check against the actual set position
     * @return TRUE if the servo is set to that position
     */
    public boolean isAtPosition(double comparePosition) {
        return(Math.round(getPositionWithOffset()*100.0) == Math.round(comparePosition*100.0));  // deals with rounding error
    }

    /**
     * Determine if the Pwm signal is enabled for the servo (as tracked internally by the wrapper)
     * @return TRUE if the servo Pwm is enabled
     */
    public boolean isEnabled() {
        return this.enabled;
    }

    /**
     * Determine if the Pwm signal is disabled for the servo (as tracked internally by the wrapper)
     * @return TRUE if the servo Pwm is disabled
     */
    public boolean isDisabled() {
        return !this.enabled;
    }

    // Servo overrides

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        this.servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return this.servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        this.enabled = true;  // setting a position re-enables
        if (this.eStopped) {
            this.timer = System.currentTimeMillis() + this.sweepTime;  //allow full sweep time
        }
        else {
            if (isAtPosition(position)) return;  // has already been set (but not necessarily done moving), no need to update timer or position
            this.timer = calcSweepTimerValue(position);
        }
        this.servo.setPosition(position - offset);
    }

    @Override
    public double getPosition() {
        return this.servo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        this.servo.scaleRange(min, max);
    }

    // internal methods

    private double calcSweepChange(double newPosition) {
        return Math.abs(getPositionWithOffset()-newPosition);
    }

    private long calcSweepTimerValue(double newPosition) {
//        // this assumes the previous move was completed
//        return System.currentTimeMillis() + (long)(calcSweepChange(newPosition) * (long)sweepTime);
        // would it be safer to add to the remaining time?
        if (isTimerDone()) {
            return System.currentTimeMillis() + (long)(calcSweepChange(newPosition) * (long)sweepTime);
        }
        else {
            return (long)sweepTime + (long)(calcSweepChange(newPosition) * (long)sweepTime);
        }
    }
}
