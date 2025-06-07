package gcsrobotics.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import gcsrobotics.framework.OpModeBase;
import gcsrobotics.framework.hardware.Claw;

///This class reads out the current position of a servo and allows you to adjust it,
/// meaning it is very useful for finding servo limits.
/// You can set these limits with Servo.scaleRange(double min, double max).
/// You can also set those limits with Claw.setLimits(double min, double max).
/// @see Servo#scaleRange(double min, double max)
/// @see Claw#setLimits(double min,double max)
@TeleOp(name="Servo Tuner")
public class ServoTuner extends OpModeBase {//Replace this with the name that you want to tune

    private double servoPos = 0.5;
    private boolean canTick = false;
    protected void runInit(){
        telemetry.addData("Status","Ready");
        telemetry.update();
    }

    protected void run(){
        while(opModeIsActive()) {
            if (gamepad1.right_bumper && canTick) {
                servoPos += servoPos >= 1 ? 0 : 0.01;

                canTick = false;

            } else if (gamepad1.left_bumper && canTick) {
                servoPos -= servoPos <= 0 ? 0 : 0.01;
                canTick = false;

            } else canTick = true;

            telemetry.addData("Servo Position",servoPos);
            telemetry.update();

            //Replace this with the servo you want to tune
            servo.setPosition(servoPos);

        }
    }
}
