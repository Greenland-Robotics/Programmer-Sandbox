package gcsrobotics.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.AutoBase;

/**This class allows you to test distances in the x and y directions,
 * so you can tune the drive constants, Constants.KpDrive and Constants.KdDrive live with
 * FTC Dashboard
 * @see ServoTuner
 * @see TurnTuner
 * @see OdoTuner
 */
@TeleOp(name="Drive Tuner")
//@Disabled
public class DriveTuner extends AutoBase {

    protected void runSequence(){
        while(opModeIsActive()){

            //Test vertically with a long distance
            if(gamepad1.y){
                path(400,0);
                resetPosition();
            }

            //Test vertically with a short distance
            if(gamepad1.a){
                path(200,0);
                resetPosition();
            }


            //Test horizontally with a long distance
            if(gamepad1.x){
                path(0,400);
                resetPosition();
            }

            //Test horizontally with a short distance
            if(gamepad1.b){
                path(0,200);
                resetPosition();
            }

        }
    }

}
