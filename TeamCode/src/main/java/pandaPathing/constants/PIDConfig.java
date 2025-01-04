package pandaPathing.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConfig {

    // Translational PID
    public static double translationalP = 0.1;
    public static double translationalI = 0.0;
    public static double translationalD = 0.01;
    public static double translationalF = 0.0;

    // Heading PID
    public static double headingP = 2.0;
    public static double headingI = 0.0;
    public static double headingD = 0.1;
    public static double headingF = 0.0;

    // Drive PID
    public static double driveP = 0.001;
    public static double driveI = 0.0;
    public static double driveD = 0.0;
    public static double driveF = 0.00006;

    // Optional Secondary PID coefficients
    public static boolean useSecondaryTranslationalPID = true;
    public static boolean useSecondaryHeadingPID = true;
    public static boolean useSecondaryDrivePID = true;
}