package gcsrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import gcsrobotics.framework.AutoBase;

@Autonomous(name="Auto Example")
@Disabled
public class AutoExample extends AutoBase {

    @Override
    public void initSequence() {
        claw.setPosition(0);
    }

    @Override
    public void runSequence() {

        // Example usage of setMotorPosition
        //Example usage of the wait(int milliseconds) method
        wait(1000);


        // Example usage of the prebuilt path and chain methods
        // Specify any coordinate, and it will go there.
        path(100, 100);
        wait(1000);


        claw.setPosition(1);
        wait(1000);
    }

}