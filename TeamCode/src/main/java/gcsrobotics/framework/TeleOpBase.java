package gcsrobotics.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("unused")
public abstract class TeleOpBase extends OpMode {

    //Declare motors here. Add any actuator motors you need
    public DcMotor fl,fr,bl,br;//,arm,slide,etc.

    //Declare any servos you have
    public Servo claw;// wrist, rotate, etc.
    private double vertical, horizontal, pivot, speed = 0.7;


    public ElapsedTime runTimer = new ElapsedTime();

    @Override
    public void init(){
        // Map to config. These must match what is in the config. Add maps to extra motors
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");
      //arm = hardwareMap.get(DcMotor.class,"arm"), for example

        claw = hardwareMap.get(Servo.class,"claw");
      //wrist = hardwareMap.get(Servo.class,"wrist"), for example


        //Reverse motors when necessary, typically will be the right motors, but you will have to check
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run without encoders for TeleOp
        for (DcMotor motor : new DcMotor[]{fl, fr, bl, br}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        runInit();
        resetRuntime();
    }

    @Override
    public void loop(){
        runLoop();
    }

    public abstract void runLoop();
    public abstract void runInit();

    /// Sets the drive speed of the chassis
    public void setSpeed(double speed){
        this.speed = speed;
    }

    /// Calling this method implements the entire mecanum drive logic in one line, use this unless you need a different
    /// system for driving
    public void implementDriveLogic(){

        //Horizontal Lock
        if (gamepad1.right_bumper) {
            horizontal = 0;
        } else {
            // Joystick and trigger combined horizontal control
            double rightTriggerDeadZone = Math.abs(gamepad1.right_trigger) > 0.1 ? gamepad1.right_trigger : 0;
            double leftTriggerDeadZone = Math.abs(gamepad1.left_trigger) > 0.1 ? gamepad1.left_trigger : 0;
            double stickDeadZone = Math.abs(gamepad1.left_stick_x) > 0.1 ? gamepad1.left_stick_x : 0;
            horizontal = -stickDeadZone - rightTriggerDeadZone + leftTriggerDeadZone - (gamepad2.right_trigger * 0.35) + (gamepad2.left_trigger * 0.35);
        }

        pivot = gamepad1.right_stick_x;
        vertical = gamepad1.left_stick_y;

        //Set powers and limit speed
        fl.setPower(speed * (pivot - vertical - horizontal));
        fr.setPower(speed * (pivot + vertical - horizontal));
        bl.setPower(speed * (pivot - vertical + horizontal));
        br.setPower(speed * (pivot + vertical + horizontal));
    }

    /// Sets the given motor to go to a certain position, at full speed.
    /// If you want to vary the speed, add another parameter with the speed you want
    public void setMotorPosition(DcMotor motor, int targetPosition){
        setMotorPosition(motor,targetPosition,1);
    }
    /// Sets the given motor to go to a certain position at a given speed
    public void setMotorPosition(DcMotor motor, int targetPosition,double speed){
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
    }

    /// Resets the runTimer of the opMode
    public void resetRunTimer(){
        runTimer.reset();
    }

    /// Returns the current time of the opMode
    public double getRunTime(){
        return runTimer.milliseconds();
    }


}
