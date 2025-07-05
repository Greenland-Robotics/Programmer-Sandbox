package gcsrobotics.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.TeleOpBase;

/**
 * This opmode tunes the Odometry system on your robot. Follow the step by step instructions on the
 * telemetry to tune correctly.
 * This is only a basic tuner. For any troubleshooting issues or if you have any questions refer
 * to the <a href="https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf?srsltid=AfmBOoqu3tdALGfSoZl9luumLbiPfkdN_fDECC5KgIIDm8Ch6J7xZ0Us">full documentation</a>
 * or, for an easier alternative, look at PinpointDocumentation.md
 */
@TeleOp(name="Odo Tuner")
public class OdoTuner extends TeleOpBase {


    @Override
    protected void inInit() {
        resetPosition();
    }

    @Override
    protected void runLoop() {
        telemetry.addLine("The X value should uptick when moving forward");
        telemetry.addLine("The Y value should uptick when moving left");
        telemetry.addData("X", getX());
        telemetry.addData("Y", getY());
        telemetry.addData("Heading/Angle of Robot", getAngle());
        telemetry.addLine("Make your changes in OpModeBase, there is a section in initHardware() that sets the right directions");
        telemetry.addLine("When finished, press A to reset position");
        telemetry.update();
        if (gamepad1.a) resetPosition(); sleep(10);

    }
}
