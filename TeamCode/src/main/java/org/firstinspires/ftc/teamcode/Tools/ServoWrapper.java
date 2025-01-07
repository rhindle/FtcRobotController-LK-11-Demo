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

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getOffset() {
        return this.offset;
    }

    public void setSweepTime(int sweepTime) { this.sweepTime = sweepTime; }

    public double calcSweepChange(double newPosition) {
        return Math.abs(getPositionWithOffset()-newPosition);
    }

    public long calcSweepTimerValue(double newPosition) {
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

    public double getPositionWithOffset() {
        return getPosition() + offset;
    }

    public boolean isTimerDone() { return System.currentTimeMillis() >= this.timer; }

    public boolean isAtPositionWithTime(double comparePosition) {
        return isAtPosition(comparePosition) && isTimerDone();
    }

    public boolean isAtPosition(double comparePosition) {
        return(Math.round(getPositionWithOffset()*100.0) == Math.round(comparePosition*100.0));  // deals with rounding error
    }

    public void eStop() {
        disable();
        eStopped = true;  // we no longer know where the servo is, so need to time accordingly next move
    }

    public void disable() {
        //((ServoControllerEx) getController()).setServoPwmDisable(getPortNumber());
        ((ServoImplEx) servo).setPwmDisable();
        this.enabled = false;
    }

    public void enable() {
        //((ServoControllerEx) getController()).setServoPwmEnable(getPortNumber());
        ((ServoImplEx) servo).setPwmEnable();
        this.enabled = true;
    }

    public boolean isEnabled() {
        return this.enabled;
    }

    public boolean isDisabled() {
        return !this.enabled;
    }

    public void setFullPwmRange() {
        ((ServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setPwmRange(double low, double high) {
        // could use some checking
        ((ServoImplEx) this.servo).setPwmRange(new PwmControl.PwmRange(low, high));
    }

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
}
