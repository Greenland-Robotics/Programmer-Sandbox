package gcsrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.TeleOpBase;

@TeleOp(name="Example TeleOp")
@Disabled
public class TeleOpExample extends TeleOpBase {
    public void runInit(){
        claw.setPosition(0);
    }

    public void runLoop(){

        implementDriveLogic();

        if(gamepad2.b){
            setMotorPosition(fl,100);
        }

        if(gamepad1.a){
            setSpeed(0.3);
        }
        else if(gamepad1.b){
            setSpeed(0.5);
        }if(gamepad1.x){
            setSpeed(0.7);
        }
        else if(gamepad1.y){
            setSpeed(1);
        }


    }
}
