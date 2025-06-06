package gcsrobotics.examples; // Yours will say gcsrobotics.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;// You will need to include this
import com.qualcomm.robotcore.eventloop.opmode.Disabled; // You DON'T need to include this

import static gcsrobotics.framework.Constants.*; // This imports all of the constants that you need from the Constants file
import gcsrobotics.framework.AutoBase;// This imports the AutoBase, you need this

@Autonomous(name="Auto Example")// This tells the program what this opmode is called, in this case, "Auto Example"
@Disabled // This says to not include it in the driver hub, but if you want to actually run this, don't include it
public class AutoExample extends AutoBase /*This means it can access stuff from AutoBase */ {

    // Put all of your init logic here.
    // Include the @Override
    @Override
    protected void initSequence() {
        claw.setPosition(clawClose);
    }

    //Put all of your auto logic right in this method, runSequence()
    //Make sure you include the protected void and @Override
    @Override
    protected void runSequence() {

        // Example usage of setPosAndWait()
        // Be sure to include the this parameter after your desired position
        arm.setPosAndWait(armUp,this);

        // Example usage of the prebuilt path and chain methods
        // Specify any coordinate, and it will go there.
        path(100, 100);
        wait(200);

        //Set the claw position to closed
        claw.setPosition(clawOpen);
        // Holds the execution for 100. Similar to sleep
        wait(100);


        arm.setPosition(armDown);
        //Example usage of wait until, it looks different from the other methods,
        //but nothing is different. Just include the () -> and then your boolean value
        waitUntil(() -> arm.getCurrentPosition() == armDown);


        chain(100,200);

        // Move forward for 1 second. Make the power negative to go backwards
        simpleDrive(Axis.X,0.5,1000);
        arm.setPosAndWait(armUp,0.9,this);
        // Move right for 1 second. Make the power negative to go left
        simpleDrive(Axis.Y, 0.5, 1000);
    }

}
