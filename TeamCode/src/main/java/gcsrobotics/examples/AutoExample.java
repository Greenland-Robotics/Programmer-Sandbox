package gcsrobotics.examples;

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
        // Be sure to include the this parameter after your desired position
        arm.setPosAndWait(armUp,this);

        // Example usage of the prebuilt path and chain methods
        // Specify any coordinate, and it will go there.
        path(100, 100);
        wait(200);

        //Center a servo
        servo.setPosition(0.5);
        wait(100);

        //Example usage of opening the claw according to Constants.clawOpen
        claw.close();
        arm.setPosition(armDown);
        //Example usage of wait until, it looks different from the other methods,
        //but nothing is different. Just include the () -> and then your boolean value
        waitUntil(() -> arm.getCurrentPosition() == armDown);

        //Example usage of opening the claw according to Constants.clawOpen
        claw.open();
        chain(100,200);

        // Move forward for 1 second
        simpleDrive(Axis.X,0.5,1000);
        arm.setPosAndWait(armUp,0.9,this);
    }

}
