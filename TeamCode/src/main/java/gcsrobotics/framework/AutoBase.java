package gcsrobotics.framework;

import static gcsrobotics.framework.Constants.KdDrive;
import static gcsrobotics.framework.Constants.KpDrive;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@SuppressWarnings("unused")
public abstract class AutoBase extends FrameBase {


    /// Optional method to define when you want to run code in init
    public void initSequence(){}

    /// The code that runs during start
    public abstract void runSequence();

    @Override
    public void runInit(){
        initSequence();
    }

    @Override
    public void run() {
        runSequence();
    }


    /// Used for making small, corrective movements when you need simple directional movement
    public void simpleDrive(String direction, double power, int time){
        if(direction.equals("vertical")){
            setPowers(power);
            wait(time);
            stopMotors();
        }else if(direction.equals("horizontal")){
            fl.setPower(-1);
            fr.setPower(1);
            bl.setPower(1);
            br.setPower(-1);
            wait(time);
            stopMotors();
        }
    }


    /// Set all motor powers to a certain power
    public void setPowers(double power) {
        for (DcMotorEnhanced motor : new DcMotorEnhanced[]{fl, fr, bl, br}) {
            motor.setPower(power);
        }
    }

    /// Waits for a set amount of time, similar to sleep, but better for a few reasons
    public void wait(int milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            telemetry.addLine("Waiting for " + (milliseconds - timer.milliseconds()));
            telemetry.update();
            odo.update();
            sleep(50);
        }
    }


    /// Accurate movement to any specified coordinate you want.
    /// If you need to be accurate in your positioning, use this method.
    public void path(int targetX, int targetY) {
        path(targetX, targetY, ' ');
    }


    /// Accurate movement to any specified coordinate you want.
    /// If you need to be accurate in your positioning, use this method.
    /// Also has a forgiveAxis, so if you don't want a particular axis to affect
    /// end behavior, you can specify it here
    public void path(int targetX, int targetY, char forgiveAxis) {
        ElapsedTime endTimer = new ElapsedTime();
        boolean endSession = false;

        while (opModeIsActive() && notStuck()) {
            double xError = targetX - getX();
            double yError = targetY - getY();

            boolean atTarget = Math.abs(xError) < 20 && Math.abs(yError) < 20;
            if (forgiveAxis == 'x' || forgiveAxis == 'X') atTarget = Math.abs(yError) < 20;
            else if (forgiveAxis == 'y' || forgiveAxis == 'Y') atTarget = Math.abs(xError) < 20;

            if (atTarget && !endSession) {
                endSession = true;
                endTimer.reset();
            } else if (!atTarget) {
                endSession = false;
            }

            if (endSession && endTimer.milliseconds() > 100) break;

            double xPower = pidDrivePower(xError, true);
            double yPower = pidDrivePower(yError, false);
            double headingCorrection = Range.clip(0.03 * getHeading(), -0.3, 0.3);

            setMotorPowers(xPower, yPower, headingCorrection);
            sendTelemetry("PATH", xError, yError, xPower, yPower, headingCorrection);
        }

        stopMotors();
    }

    /// Movement to any specified coordinates you want
    /// If you want to be fast, but don't need it to be very accurate, use this.
    public void chain(int targetX, int targetY) {
        chain(targetX, targetY, ' ');
    }

    /// Movement to any specified coordinates you want
    /// If you want to be fast, but don't need it to be very accurate, use this.
    /// Also has a forgiveAxis, so if you don't want a particular axis to affect
    /// end behavior, you can specify it here
    public void chain(int targetX, int targetY, char forgiveAxis) {
        while (opModeIsActive() && notStuck()) {
            double xError = targetX - getX();
            double yError = targetY - getY();

            boolean atTarget = Math.abs(xError) < 30 && Math.abs(yError) < 30;
            if (forgiveAxis == 'x' || forgiveAxis == 'X') atTarget = Math.abs(yError) < 30;
            else if (forgiveAxis == 'y' || forgiveAxis == 'Y') atTarget = Math.abs(xError) < 30;

            if (atTarget) break;

            double xPower = pidDrivePower(xError, true);
            double yPower = pidDrivePower(yError, false);
            double headingCorrection = Range.clip(0.03 * getHeading(), -0.3, 0.3);

            setMotorPowers(xPower, yPower, headingCorrection);
            sendTelemetry("CHAIN", xError, yError, xPower, yPower, headingCorrection);
        }
        stopMotors();
    }

    ///  Calculates drive power for the pathing methods
    private double pidDrivePower(double error, boolean isX) {
        double kp = isX ? KpDrive : KpDrive + 0.006;
        return kp * error + KdDrive * error;
    }


    /// Sets the motor powers according to the calculated powers for pathing methods
    private void setMotorPowers(double xPower, double yPower, double headingCorrection) {
        double flPower = xPower - yPower + headingCorrection;
        double frPower = xPower + yPower - headingCorrection;
        double blPower = xPower + yPower + headingCorrection;
        double brPower = xPower - yPower - headingCorrection;

        // Find the largest magnitude
        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(flPower), Math.abs(frPower)),
                        Math.max(Math.abs(blPower), Math.abs(brPower))
                )
        );

        // Scale all powers if needed
        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);
    }



    /// Sends any telemetry
    private void sendTelemetry(String label, double xErr, double yErr, double xPow, double yPow, double headingCorr) {
        telemetry.addLine("Following a " + label);
        telemetry.addData("X Coord", getX());
        telemetry.addData("Y Coord", getY());
        telemetry.addData("Heading", getHeading());
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
    private boolean notStuck() {
        double x = getX(), y = getY();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        if(timer.seconds() < 1) {
            return true;
        }else{
            return Math.abs(getX() - x) > 10 || Math.abs(getY() - y) > 10;
        }
    }

    /// Getter method for the x coordinate
    private double getX() {
        odo.update();
        return odo.getPosition().getX(DistanceUnit.INCH);
    }


    /// Getter method for the y coordinate
    private double getY() {
        odo.update();
        return odo.getPosition().getY(DistanceUnit.INCH);
    }


    ///  Getter method for the heading
    private double getHeading() {
        odo.update();
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public void waitUntil(Supplier<Boolean> condition) {
        while (!condition.get()) {
            idle();
        }
    }


}