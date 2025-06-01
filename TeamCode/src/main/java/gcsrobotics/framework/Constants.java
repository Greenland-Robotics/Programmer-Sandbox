package gcsrobotics.framework;

import com.acmerobotics.dashboard.config.Config;

/// <strong>A class that holds all of the robot constants</strong>
@Config("Robot Constants") //This Config tag allows for these values to be tuned in real time using the dashboard
public class Constants {
    public static double clawClose;
    public static double clawOpen;
    public static double KpDrive;
    public static double KdDrive;
    public static double KpTurn;
    public static double KdTurn;
    public static int armUp;
    public static int armMiddle;
    public static int armDown;
    public static double wristUp;
    public static double wristDown;
    public static int ENCODER_TOLERANCE;
    public static double autoMaxPower;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection;

    static{


        /* Examples of constants
        --- Use this to set things like servo and motor positions ---
        This allows you to change numbers in only one spot if you need
        to make an adjustment
         */

        clawClose = 0;
        clawOpen = 1;

        KpDrive = 0.01;
        KdDrive = 0.001;
        KpTurn = 0.1;
        KdTurn = 0.01;


        //More examples here
        armUp = 500;
        armDown = 0;
        armMiddle = 250;

        wristUp = 0.8;
        wristDown = 0;

        //This must be between 0 and 1
        autoMaxPower = 0.6;


        ENCODER_TOLERANCE = 10;

        //Reverse this if going forward makes the x coordinate go down
        xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        //Reverse this if going left makes the y coordinate go down
        yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;


    }


}
