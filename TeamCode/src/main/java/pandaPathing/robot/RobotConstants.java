package pandaPathing.robot;

import com.acmerobotics.dashboard.config.Config;

@Config

public class RobotConstants {

    // Servo positions
    public static double

    pitchFDown = 0.15,

    pitchFOut = 0.382,

    pitchMUp = 0.6,

    pitchBOut = 0.97,

    //pitchBDown = 0.2,

    pitchBUp = 0.3,

    railRMax = 1,

    railLMax = 0,

    railRMin = 0,

    railLMin = 1,

    v4bFUp = 0.87,

    v4bFOut = 0.85,

    v4bFDown = 0.96,

    v4bMUp = 0.7,

    v4bBUp = 0.57,

    v4bBDown = 0.53,

    v4bspec = 0.377,

    clawOpen = 0.53,

    clawClose = 0.05,

    claw0 = 0.8,
    //y

    claw45 = 0.7,
    //a

    claw90 = 0.5,
    //x

    claw45_2 = 0.38,
    //b

    claw180 = 0.2,

    yaw45_2 = 0.75,

    yaw0 = 0.2,

    yaw45 = 0.845;


    // Slide constants
    public static int

    slideMax = 1450,

    slideMaxSpec = 613,

    slideMaxSpecTele = 613,

    slideSampleAuto = 490,

    slideMin = -10,

    CVSmoothing = 1;


    // PID Constants
    public static double p = 0.034, d = 0.9, f = 0.2, l = 0, homingConstant = 0;

    public static double p1 = 0.02, d1 = 0.7, f1 = 0.15, l1 = 0, homingConstant1 = 0;
}