package pandaPathing.robot;

import com.acmerobotics.dashboard.config.Config;

@Config

public class RobotConstants {

    // Servo positions
    public static double

    pitchGrab = 0.12,

    pitchSpecimen = 0.9,

    pitchDown = 0.6,

    pitchDeposit = 0.9,


    railROut = 1,

    railLOut = 0,

    railRIn = 0,

    railLIn = 1,

    v4bExtend = 0.84,

    v4bScoreSpec = 0.83,

    v4bGrab = 0.9,

    v4bUp = 0.6,

    v4bDeposit = 0.5,

    v4bGrabSpec = 0.377,

    clawOpen = 0.9,

    clawClose = 0,

    claw0 = 0.8,
    //y

    claw45 = 0.66,
    //a

    claw90 = 0.5,
    //x

    claw45_2 = 0.92,
    //b

    claw180 = 0.2,

    yaw45_2 = 0.75,

    yaw0 = 0.2,

    yaw45 = 0.845;


    // Slide constants
    public static int

    slideScoreHighBasket = 1450,
    slideHighBasket = slideScoreHighBasket - 50,

    slideScoreLowBasket = 1450,
    slideLowBasket = slideScoreHighBasket - 50,

    slideHighChamber = 607,

    slidePark = 490,

    slideZero = -10;


    // PID Constants
    public static double p = 0.036, d = 0.9, f = 0.15, l = 0;

    public static double p1 = 0.02, d1 = 0.7, f1 = 0.15, l1 = 0, homingConstant1 = 0;
}