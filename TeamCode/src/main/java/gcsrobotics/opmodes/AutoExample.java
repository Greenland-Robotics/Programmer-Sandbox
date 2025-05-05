package gcsrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static gcsrobotics.framework.Constants.*;
import gcsrobotics.framework.AutoBase;

@Autonomous(name="Auto Example")
@Disabled
public class AutoExample extends AutoBase {

    @Override
    public void initSequence() {
        claw.setPosition(clawClose);
    }

    @Override
    public void runSequence() {

        // Example usage of setPosAndWait()
        arm.setPosAndWait(armUp,this);

        // Example usage of the prebuilt path and chain methods
        // Specify any coordinate, and it will go there.
        path(100, 100);
        wait(200);


        claw.setPosition(clawClose);
        wait(100);


        arm.setPosition(armDown);
        //Example usage of wait until, it looks different from the other methods,
        //but nothing is different. Just include the () -> and then your boolean value
        waitUntil(() -> arm.getCurrentPosition() != armDown);


        chain(100,200);
        arm.setPosAndWait(armUp,0.9,this);
    }

}
