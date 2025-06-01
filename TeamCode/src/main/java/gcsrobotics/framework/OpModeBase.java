package gcsrobotics.framework;

import static gcsrobotics.framework.Constants.xPodDirection;
import static gcsrobotics.framework.Constants.yPodDirection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


public abstract class OpModeBase extends LinearOpMode {
    protected DcMotorEnhanced fl,fr,bl,br,arm;
    protected Servo claw;
    protected GoBildaPinpointDriver odo;


    protected abstract void runInit();
    protected abstract void run();

    private void initHardware(){
        //TODO: Update this config for your robot
        fl = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"fl"));
        fr = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"fr"));
        bl = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"bl"));
        br = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"br"));
        arm = new DcMotorEnhanced(hardwareMap.get(DcMotor.class,"arm"));

        claw = hardwareMap.get(Servo.class,"claw");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(xPodDirection, yPodDirection);
        odo.resetPosAndIMU();

        //TODO: Set motor directions. Some motors will be reversed, so you must change that here
        //Note: Typically the right side is reversed, but change it as you need
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void runOpMode(){
        // Initialize telemetry for dashboard and Driver Hub
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize hardware
        initHardware();

        //Run the init sequence
        runInit();

        //Wait for the start button to be pressed
        waitForStart();

        //Run the main logic when the start button is pressed
        run();

    }
}
