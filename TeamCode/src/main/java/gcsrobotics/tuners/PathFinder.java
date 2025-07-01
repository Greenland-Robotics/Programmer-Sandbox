package gcsrobotics.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.TeleOpBase;

@TeleOp(name="Path Finder")
public class PathFinder extends TeleOpBase {

    @Override
    protected void inInit() {
        resetPosition();
        telemetry.addLine("Use this to find positions on the field");

    }

    @Override
    protected void runLoop() {
        telemetry.addData("X",getX());
        telemetry.addData("Y",getY());
        telemetry.addData("Angle",getAngle());
        telemetry.update();

        if(gamepad1.a){
            resetPosition();
        }


    }
}
