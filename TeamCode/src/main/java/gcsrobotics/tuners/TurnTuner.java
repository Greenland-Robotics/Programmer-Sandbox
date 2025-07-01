package gcsrobotics.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.AutoBase;

/**This class allows you to test turning,
 * so you can tune the turning constants: Constants.KpTurn live with FTC Dashboard
 * @see ServoTuner
 * @see DriveTuner
 * @see OdoTuner
 */
@TeleOp(name="Turn Tuner")
//@Disabled
public class TurnTuner extends AutoBase {

    protected void runSequence() {
        while (opModeIsActive()) {
            if (gamepad1.b) {
                turn(90);
                resetPosition();
            }

            if (gamepad1.x) {
                turn(180);
                resetPosition();
            }

        }
    }
}