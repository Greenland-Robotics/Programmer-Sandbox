package gcsrobotics.framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@SuppressWarnings("unused")
public class DcMotorEnhanced {
    private final DcMotorEx motor;
    private double DEFAULT_SPEED = 1;

    public DcMotorEnhanced(DcMotor motor) {
        // Try to cast to DcMotorEx for bonus features
        this.motor = (DcMotorEx) motor;
    }

    public void setPosAndWait(int targetPosition){
        setPosAndWait(targetPosition,DEFAULT_SPEED);
    }
    public void setPosAndWait(int targetPosition, double speed){
        setPosition(targetPosition,speed);
        while(getCurrentPosition() != targetPosition){
            Thread.yield();
        }
    }

    /// Sets the given motor to go to a certain position, at full speed.
    /// If you want to vary the speed, add another parameter with the speed you want
    public void setPosition(int targetPosition){
        this.setPosition(targetPosition,DEFAULT_SPEED);
    }
    /// Sets the given motor to go to a certain position at a given speed
    public void setPosition(int targetPosition,double speed){
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }

    public void setDefaultSpeed(double DEFAULT_SPEED){
        this.DEFAULT_SPEED = DEFAULT_SPEED;
    }
    public double getDefaultSpeed(){return DEFAULT_SPEED;}

    // === Forward motor control with minimal effort ===
    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setVelocity(double velocity) {
        motor.setVelocity(velocity);
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return motor.getMode();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection(){
        return motor.getDirection();
    }

    // --- Plus, you can still access motor directly if needed ---
    public DcMotorEx getBaseMotor() {
        return motor;
    }
}
