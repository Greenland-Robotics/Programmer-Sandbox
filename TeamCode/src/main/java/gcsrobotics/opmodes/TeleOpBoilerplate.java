package gcsrobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;//You won't need this
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.framework.TeleOpBase;

@TeleOp(name="TeleOp Boilerplate")
@Disabled//Do NOT include this. This is to ensure this program doesn't show up on the driver hub, but
//you will want your programs to show up
public class TeleOpBoilerplate extends TeleOpBase {

    @Override
    protected void runLoop() {

    }

    @Override
    protected void inInit() {

    }
}
