package gcsrobotics.framework;

import static gcsrobotics.framework.Constants.KdDrive;
import static gcsrobotics.framework.Constants.KpDrive;
import static gcsrobotics.framework.Constants.KpTurn;
import static gcsrobotics.framework.Constants.autoMaxPower;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Supplier;

import gcsrobotics.framework.hardware.DcMotorEnhanced;

/** The base for all autonomous opmodes. This class extends OpModeBase, so you have access to all
 * hardware, global variables, and more.
 * <br>
 * This provides a centralized and modular approach to autonomous opmodes.<br>
 * This class has pathing methods,simple drive methods, and lots of utility methods
 * @author Josh Kelley
 **/
@SuppressWarnings("all")
public abstract class AutoBase extends OpModeBase {

    protected enum Axis{
        X,
        Y,
        NONE;
    }


    /// Optional method to define when you want to run code in init
    protected void initSequence(){}

    /// The code that runs during start
    protected abstract void runSequence();

    @Override
    protected void runInit(){
        initSequence();
    }

    @Override
    protected void run() {
        runSequence();
    }

    private final ElapsedTime stuckTimer = new ElapsedTime();
    private double lastDistanceToTarget = Double.MAX_VALUE;
    private double targetAngle = 0;


    /// Used for making small, corrective movements when you need simple directional movement
    /// @param direction the direction you want to move
    /// @param power the power to set the motors to
    /// @param time the time to wait in milliseconds
    protected void simpleDrive(Axis direction, double power, int time){
        switch(direction){
            case X:
                setPowers(power);
                wait(time);
                stopMotors();
                break;
            case Y:
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(power);
                br.setPower(-power);
                wait(time);
                stopMotors();
                break;
            default:
                throw new IllegalArgumentException("Invalid direction for simpleDrive: NONE" +
                        "is not a valid direction");
        }


    }


    /// Set all motor powers to a certain power
    /// @param power the power to set the motors to
    protected void setPowers(double power) {
        for (DcMotorEnhanced motor : new DcMotorEnhanced[]{fl, fr, bl, br}) {
            motor.setPower(power);
        }
    }

    /// Waits for a set amount of time, similar to sleep, but better for a few reasons
    /// @param milliseconds the amount of time to wait in milliseconds
    protected void wait(int milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            telemetry.addLine("Waiting for " + (milliseconds - timer.milliseconds()));
            telemetry.update();
            odo.update();
            sleep(50);
        }
    }


    /// <strong>Precise positioning</strong><br>
    /// Accurate movement to any specified coordinate you want.
    /// If you need to be accurate in your positioning, use this method.
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    protected void path(int targetX, int targetY) {
        path(targetX, targetY, Axis.NONE);
    }


    /// <strong>Precise positioning</strong><br>
    /// Accurate movement to any specified coordinate you want.
    /// If you need to be accurate in your positioning, use this method.
    /// This method has a forgiveAxis, so if you don't want a particular axis to affect
    /// end behavior, you can specify it here
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    /// @param forgiveAxis the axis you want to not consider in the end behavior
    protected void path(int targetX, int targetY, Axis forgiveAxis) {
        ElapsedTime endTimer = new ElapsedTime();
        boolean endSession = false;

        while (opModeIsActive() && notStuck(targetX,targetY)) {
            double xError = targetX - getX();
            double yError = targetY - getY();

            boolean atTarget = Math.abs(xError) < 20 && Math.abs(yError) < 20;
            if (forgiveAxis == Axis.X) atTarget = Math.abs(yError) < 20;
            else if (forgiveAxis == Axis.Y) atTarget = Math.abs(xError) < 20;

            if (atTarget && !endSession) {
                endSession = true;
                endTimer.reset();
            } else if (!atTarget) {
                endSession = false;
            }

            if (endSession && endTimer.milliseconds() > 100) break;

            double xPower = pidDrivePower(xError, true);
            double yPower = pidDrivePower(yError, false);
            double headingCorrection = Range.clip(0.03 * getAngle() - this.targetAngle, -0.3, 0.3);

            setMotorPowers(xPower, yPower, headingCorrection);
            sendTelemetry("PATH", xError, yError, xPower, yPower, headingCorrection);
        }

        stopMotors();
    }

    /// <strong>Fast, but not as accurate</strong><br>
    /// Movement to any specified coordinates you want
    /// If you want to be fast, but don't need it to be very accurate, use this.
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    protected void chain(int targetX, int targetY) {
        chain(targetX, targetY, Axis.NONE);
    }

    /// <strong>Fast, but not as accurate</strong><br>
    /// Movement to any specified coordinates you want
    /// If you want to be fast, but don't need it to be very accurate, use this.
    /// This method has a forgiveAxis, so if you don't want a particular axis to affect
    /// end behavior, you can specify it here
    /// @param targetX the x coordinate you want to go to
    /// @param targetY the y coordinate you want to go to
    /// @param forgiveAxis the axis you want to not consider in the end behavior
    protected void chain(int targetX, int targetY, Axis forgiveAxis) {
        while (opModeIsActive() && notStuck(targetX,targetY)) {
            double xError = targetX - getX();
            double yError = targetY - getY();

            boolean atTarget = Math.abs(xError) < 40 && Math.abs(yError) < 40;
            if (forgiveAxis == Axis.X) atTarget = Math.abs(yError) < 40;
            else if (forgiveAxis == Axis.Y) atTarget = Math.abs(xError) < 40;

            if (atTarget) break;

            double xPower = pidDrivePower(xError, true);
            double yPower = pidDrivePower(yError, false);
            double headingCorrection = Range.clip(0.03 * getAngle() - this.targetAngle, -0.3, 0.3);

            setMotorPowers(xPower, yPower, headingCorrection);
            sendTelemetry("CHAIN", xError, yError, xPower, yPower, headingCorrection);
        }
        stopMotors();
    }

    /**
     * Turns the robot to a specific heading (in degrees).
     * Positive angles are counterclockwise, negative are clockwise.
     * Uses a proportional controller for heading.
     * @param targetAngle the angle to turn to, in degrees (0 = field forward, CCW+)
     */
    protected void turn(double targetAngle) {
        final double tolerance = 1.5; // Degrees within target to finish
        final long settleTimeMillis = 150; // How long to be within tolerance (ms)

        this.targetAngle = targetAngle;

        targetAngle = normalizeAngle(targetAngle);

        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;

        while (opModeIsActive()) {
            double currentAngle = getAngle();
            double error = normalizeAngle(targetAngle - currentAngle);

            // Check if within tolerance to start settling
            if (Math.abs(error) <= tolerance) {
                if (!settling) {
                    settling = true;
                    settleTimer.reset();
                }
                if (settleTimer.milliseconds() >= settleTimeMillis) {
                    break; // Done turning
                }
            } else {
                settling = false; // Not settled, reset timer next time in tolerance
            }

            // Proportional control for turn power
            double power = KpTurn * error;

            // Only heading correction, no X/Y drive
            setMotorPowers(0, 0, power);

            telemetry.addLine("Turning");
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error",error);
            telemetry.addData("Turn Power", power);
            telemetry.update();
        }
        stopMotors();
    }

    private double normalizeAngle(double angle){
        return Math.IEEEremainder(angle, 360.0);
    }

    //Global variables specific to pidDrivePower
    private double lastXError = 0;
    private double lastYError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    /// Calculates drive power for the pathing methods using PID control
    private double pidDrivePower(double error, boolean isX) {
        double kp = isX ? KpDrive : KpDrive + 0.006;

        // Get the time delta for derivative calculation
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Calculate derivative (rate of change of error)
        double lastError = isX ? lastXError : lastYError;
        double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;

        // Update last error for next iteration
        if (isX) {
            lastXError = error;
        } else {
            lastYError = error;
        }

        // Calculate PID output
        double proportional = kp * error;
        double derivativeTerm = KdDrive * derivative;

        return proportional + derivativeTerm;
    }


    /// Sets the motor powers according to the calculated powers for pathing methods
    private void setMotorPowers(double xPower, double yPower, double headingCorrection) {
        // Compensate for robot heading (field-centric control)
        double headingRad = Math.toRadians(getAngle());

        // Apply field-centric transformation
        double forwardPower = xPower * Math.cos(headingRad) - yPower * Math.sin(headingRad);
        double strafePower = xPower * Math.sin(headingRad) + yPower * Math.cos(headingRad);

        // Calculate mecanum motor powers
        // Forward = all motors same direction, Strafe = diagonal pattern
        double flPower = forwardPower + strafePower + headingCorrection;
        double frPower = forwardPower - strafePower - headingCorrection;
        double blPower = forwardPower - strafePower + headingCorrection;
        double brPower = forwardPower + strafePower - headingCorrection;

        // Find the largest motor power magnitude
        double maxMotorPower = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        );

        // Only scale if any motor power exceeds the limit
        if (maxMotorPower > autoMaxPower) {
            double scaleFactor = autoMaxPower / maxMotorPower;
            fl.setPower(flPower * scaleFactor);
            fr.setPower(frPower * scaleFactor);
            bl.setPower(blPower * scaleFactor);
            br.setPower(brPower * scaleFactor);
        } else {
            // No scaling needed, powers are already within limits
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }
    }



    /// Sends any telemetry
    private void sendTelemetry(String label, double xErr, double yErr, double xPow, double yPow, double headingCorr) {
        telemetry.addLine("Following a " + label);
        telemetry.addData("X Coord", getX());
        telemetry.addData("Y Coord", getY());
        telemetry.addData("Heading", getAngle());
        telemetry.addData("X Error", xErr);
        telemetry.addData("Y Error", yErr);
        telemetry.addData("X Power", xPow);
        telemetry.addData("Y Power", yPow);
        telemetry.addData("Heading Corr", headingCorr);
        telemetry.update();
    }


    /// Sets all powers to 0
    private void stopMotors() {
        setPowers(0);
    }

    /// Checks if the robot is not moving
    private boolean notStuck(double targetX, double targetY) {
        double currentDistance = Math.sqrt(Math.pow(targetX - getX(), 2) + Math.pow(targetY - getY(), 2));

        if (Math.abs(currentDistance - lastDistanceToTarget) > 5) {
            lastDistanceToTarget = currentDistance;
            stuckTimer.reset();
            return true;
        }

        return stuckTimer.seconds() < 3.0;
    }


    protected void waitUntil(@NonNull Supplier<Boolean> condition) {
        while (!condition.get()) {
            sleep(10);
        }
    }


}