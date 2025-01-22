package pandaPathing.constants;

import com.acmerobotics.dashboard.config.Config;


@Config
public class PIDConfig {

    // Translational PID
    public static double translationalP = 0.1;
    public static double translationalI = 0.0;
    public static double translationalD = 0.01;
    public static double translationalF = 0.0;

    public static double secondaryTranslationalP = 0.1;
    public static double secondaryTranslationalI = 0.0;
    public static double secondaryTranslationalD = 0.01;
    public static double secondaryTranslationalF = 0.0;

    // Heading PID
    public static double headingP = 2.0;
    public static double headingI = 0.0;
    public static double headingD = 0.1;
    public static double headingF = 0.0;

    public static double secondaryHeadingP = 2.0;
    public static double secondaryHeadingI = 0.0;
    public static double secondaryHeadingD = 0.1;
    public static double secondaryHeadingF = 0.0;

    // Drive PID
    public static double driveP = 0.02;
    public static double driveI = 0.0;
    public static double driveD = 0.0;
    public static double driveF = 0.00006;

    public static double secondaryDriveP = 0.02;
    public static double secondaryDriveI = 0.0;
    public static double secondaryDriveD = 0.0;
    public static double secondaryDriveF = 0.0006;

    // Optional Secondary PID coefficients
    public static boolean useSecondaryTranslationalPID = true;
    public static boolean useSecondaryHeadingPID = true;
    public static boolean useSecondaryDrivePID = true;
}