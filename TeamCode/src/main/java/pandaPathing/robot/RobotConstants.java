package pandaPathing.robot;

import com.acmerobotics.dashboard.config.Config;

@Config

public class RobotConstants {

    // Servo positions
    public static double

            pitchFDown = 0.571,

            pitchFOut = 1,

            pitchBOut = 0.24,

            pitchBDown = 0.2,

            railRMax = 0.74,

            railLMax = 0.28,

            railRMin = 0.1,

            railLMin = 0.9,

            v4bOutUp = 0.31,

            v4bOutDown = 0.39,

            v4bBackUp = 0.16,

            v4bBackDown = 0.065,

            clawOpen = 0.5,

            clawClose = 0.1,

            claw45_2 = 0.788,

            claw0 = 0.515,

            claw45 = 0.320,

            claw90 = 0.005,

            yaw45_2 = 0.334,

            yaw0 = 0.6,

            yaw45 = 0.845;


    // Slide constants
    public static int

            slideMax = 1450,
            slideMaxSpec = 1000,
            slideMin = 0;


    // PID Constants
    public static double p = 0.035, d = 0.7, f = 0.15, l = 0, homingConstant = 0;
}