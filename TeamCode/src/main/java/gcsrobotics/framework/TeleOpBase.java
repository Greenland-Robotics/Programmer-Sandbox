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
    protected void implementDriveLogic() {
        double horizontal;
        // Horizontal Lock
        if (gamepad1.right_bumper) {
            horizontal = 0;
        } else {
            // Joystick and trigger combined horizontal control
            double rightTriggerDeadZone = Math.abs(gamepad1.right_trigger) > 0.1 ? gamepad1.right_trigger : 0;
            double leftTriggerDeadZone = Math.abs(gamepad1.left_trigger) > 0.1 ? gamepad1.left_trigger : 0;
            double stickDeadZone = Math.abs(gamepad1.left_stick_x) > 0.1 ? gamepad1.left_stick_x : 0;
            horizontal = -stickDeadZone - rightTriggerDeadZone + leftTriggerDeadZone;
        }

        double pivot = gamepad1.right_stick_x > 0.1 || gamepad1.right_stick_x < -0.1 ? gamepad1.right_stick_x : 0;

        // Use consistent naming: forward = X axis, strafe = Y axis (GoBILDA convention)
        double forward = -gamepad1.left_stick_y;  // Forward/back (X in GoBILDA)
        double strafe = horizontal;               // Left/right (Y in GoBILDA)

        if (fieldCentric) {
            // Field-centric compensation using GoBILDA convention
            double headingRad = Math.toRadians(odo.getAngle());
            double tempForward = forward * Math.cos(headingRad) - strafe * Math.sin(headingRad);
            double tempStrafe = forward * Math.sin(headingRad) + strafe * Math.cos(headingRad);
            forward = tempForward;
            strafe = tempStrafe;
        }

        // Mecanum drive motor calculations
        // Forward: all motors same direction, Strafe: diagonal pattern
        fl.setPower(speed * (pivot + forward + strafe));
        fr.setPower(speed * (-pivot + forward - strafe));
        bl.setPower(speed * (pivot + forward - strafe));
        br.setPower(speed * (-pivot + forward + strafe));
    }

    protected void toggleFieldCentric(boolean button){
        if(button && !tempButton){
            fieldCentric = !fieldCentric;
            tempButton = true;
        }else tempButton = false;
    }

}
