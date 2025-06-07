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


        /* Implements all drive logic necessary
         * The fieldCentric is toggled by the toggleFieldCentric() method
         * If you don't want the toggle, change fieldCentric to false like so:
         * implementDriveLogic(false)
        */
        implementDriveLogic(fieldCentric);
        // This toggles field-centric drive when the right bumper is pressed
        toggleFieldCentric(gamepad1.right_bumper);

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
