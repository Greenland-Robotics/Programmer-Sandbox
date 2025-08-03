package gcsrobotics.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;// You don't need to include this
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.TeleOpBase;

@TeleOp(name="Example TeleOp")
@Disabled
public class TeleOpExample extends TeleOpBase {
    @Override
    public void inInit(){
        claw.setPosition(0);
    }

    @Override
    public void runLoop(){


        /*
            Implements all drive logic necessary
            By default, fieldCentric is true.
            If you want to turn it off permanently, add this line:
            fieldCentric = false; //right before the call to implementDriveLogic()
        */
        implementDriveLogic();
        // This toggles field-centric drive when the left bumper is pressed
        toggleFieldCentric(gamepad1.left_bumper);

        //Example usage of the setMotorPosition() Method
        if(gamepad2.b){
            arm.setPosition(100);
        }


        //Speed control
        if(gamepad1.a){
            setSpeed(0.3);
        } else if(gamepad1.b){
            setSpeed(0.5);
        } else if(gamepad1.x){
            setSpeed(0.7);
        } else if(gamepad1.y){
            setSpeed(1);
        }

    }
}
