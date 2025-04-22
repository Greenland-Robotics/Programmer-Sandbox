package gcsrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import gcsrobotics.framework.AutoBase;

@Autonomous(name="Auto Example")
@Disabled
public class AutoExample extends AutoBase{

    @Override
    public void initSequence(){
        claw.setPosition(0);
    }

    @Override
    public void run(){
        setMotorPosition(fl,2000);
        wait(1000);

        path(100,100);
        wait(1000);

        claw.setPosition(1);
        wait(1000);
    }


}
