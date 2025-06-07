package gcsrobotics.framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import gcsrobotics.framework.hardware.DcMotorEnhanced;

///@author Josh Kelley
@SuppressWarnings("unused")
public abstract class TeleOpBase extends OpModeBase {

    private double speed = 0.7;
    private boolean tempButton = false;
    protected boolean fieldCentric = true;

    @Override
    protected void runInit(){
        // Run without encoders for TeleOp
        for (DcMotorEnhanced motor : new DcMotorEnhanced[]{fl, fr, bl, br}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        inInit();
    }

    @Override
    protected void run(){
        while(opModeIsActive()){
            runLoop();
        }
    }

    ///Code to be run when you press start
    protected abstract void runLoop();

    ///Code to be run when you press init
    protected abstract void inInit();

    /// Sets the drive speed of the chassis
    protected void setSpeed(double speed){
        this.speed = speed;
    }

    /// Calling this method implements the entire mecanum drive logic in one line, use this unless you need a different
    /// system for driving
    protected void implementDriveLogic(boolean fieldCentric) {
        double horizontal;
        // Horizontal Lock
        if (gamepad1.right_bumper) {
            horizontal = 0;
        } else {
            // Joystick and trigger combined horizontal control
            double rightTriggerDeadZone = Math.abs(gamepad1.right_trigger) > 0.1 ? gamepad1.right_trigger : 0;
            double leftTriggerDeadZone = Math.abs(gamepad1.left_trigger) > 0.1 ? gamepad1.left_trigger : 0;
            double stickDeadZone = Math.abs(gamepad1.left_stick_x) > 0.1 ? gamepad1.left_stick_x : 0;
            horizontal = -stickDeadZone - rightTriggerDeadZone + leftTriggerDeadZone
                    - (gamepad2.right_trigger * 0.35) + (gamepad2.left_trigger * 0.35);
        }

        double pivot = gamepad1.right_stick_x;

        double driveX = -gamepad1.left_stick_y;
        double driveY = horizontal;

        if (fieldCentric) {
            // --- Field-centric compensation ---
            double headingRad = Math.toRadians(odo.getAngle()); // getAngle() from odometry/IMU
            double tempX = driveX * Math.cos(headingRad) - driveY * Math.sin(headingRad);
            double tempY = driveX * Math.sin(headingRad) + driveY * Math.cos(headingRad);
            driveX = tempX;
            driveY = tempY;
        }

        // Set powers and limit speed (using either robot- or field-centric values)
        fl.setPower(speed * (pivot - driveX - driveY));
        fr.setPower(speed * (pivot + driveX - driveY));
        bl.setPower(speed * (pivot - driveX + driveY));
        br.setPower(speed * (pivot + driveX + driveY));
    }

    protected void toggleFieldCentric(boolean button){
        if(button && !tempButton){
            fieldCentric = !fieldCentric;
            tempButton = true;
        }else tempButton = false;
    }

}
