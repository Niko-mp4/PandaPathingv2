package pandaPathing.robot;

import com.acmerobotics.dashboard.config.Config;

@Config

public class RobotConstants {

    // Servo positions
    public static double

    pitchFDown = 0.593,

    pitchFOut = 1,

    pitchMUp = 0,

    pitchBOut = 0.316,

    //pitchBDown = 0.2,

    pitchBUp = 0.3,

    railRMax = 0.74,

    railLMax = 0.28,

    railRMin = 0.1,

    railLMin = 0.9,

    railRSpec = 0.6,

    railLSpec = 0.4,

    v4bFUp = 0.59,

    v4bFOut = 0.64,

    v4bFDown = 0.68,

    v4bMUp = 0.39,

    v4bBUp = 0.31,

    v4bBDown = 0.23,

    v4bspec = 0.377,

    clawOpen = 0.5,

    clawClose = 0.1,

    claw0 = 0.09,

    claw45 = 0.19,

    claw90 = 0.37,

    claw45_2 = 0.5,

    claw180 = 0.66,

    yaw45_2 = 0.75,

    yaw0 = 0.6,

    yaw45 = 0.845;


    // Slide constants
    public static int

    slideMax = 1450,

    slideMaxSpec = 570,

    slideMin = 0;


    // PID Constants
    public static double p = 0.035, d = 0.7, f = 0.15, l = 0, homingConstant = 0;

    public static double p1 = 0.02, d1 = 0.7, f1 = 0.15, l1 = 0, homingConstant1 = 0;
}