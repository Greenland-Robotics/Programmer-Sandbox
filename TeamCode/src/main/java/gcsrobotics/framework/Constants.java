package gcsrobotics.framework;

/// A class that houses all of the constants
public class Constants {
    public static double clawClose;
    public static double clawOpen;
    public static double KpDrive;
    public static double KdDrive;
    public static double KpTurn;
    public static double KdTurn;

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
//        final double armUp = 500;
//        final double armDown = 0;
//        final double armMiddle = 250;
//
//        final double wristUp = 0.8;
//        final double wristDown = 0;

    }


}
