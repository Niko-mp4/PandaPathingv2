package pandaPathing.constants;

import static pandaPathing.constants.PIDConfig.driveD;
import static pandaPathing.constants.PIDConfig.driveF;
import static pandaPathing.constants.PIDConfig.driveI;
import static pandaPathing.constants.PIDConfig.driveP;
import static pandaPathing.constants.PIDConfig.headingD;
import static pandaPathing.constants.PIDConfig.headingF;
import static pandaPathing.constants.PIDConfig.headingI;
import static pandaPathing.constants.PIDConfig.headingP;
import static pandaPathing.constants.PIDConfig.secondaryDriveD;
import static pandaPathing.constants.PIDConfig.secondaryDriveF;
import static pandaPathing.constants.PIDConfig.secondaryDriveI;
import static pandaPathing.constants.PIDConfig.secondaryDriveP;
import static pandaPathing.constants.PIDConfig.secondaryHeadingD;
import static pandaPathing.constants.PIDConfig.secondaryHeadingF;
import static pandaPathing.constants.PIDConfig.secondaryHeadingI;
import static pandaPathing.constants.PIDConfig.secondaryHeadingP;
import static pandaPathing.constants.PIDConfig.secondaryTranslationalD;
import static pandaPathing.constants.PIDConfig.secondaryTranslationalF;
import static pandaPathing.constants.PIDConfig.secondaryTranslationalI;
import static pandaPathing.constants.PIDConfig.secondaryTranslationalP;
import static pandaPathing.constants.PIDConfig.translationalD;
import static pandaPathing.constants.PIDConfig.translationalF;
import static pandaPathing.constants.PIDConfig.translationalI;
import static pandaPathing.constants.PIDConfig.translationalP;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {

    static {
        initializeLocalizer();
        initializeMotorSetup();
        initializeRobotSettings();
        // Initialize PID coefficients once, and update them in real-time later
        initializePIDCoefficients();
        initializePathConstraints();
    }

    // Localizer setup
    private static void initializeLocalizer() {
        FollowerConstants.localizers = Localizers.PINPOINT;
    }

    // Motor setup
    private static void initializeMotorSetup() {
        FollowerConstants.leftFrontMotorName = "em3";
        FollowerConstants.leftRearMotorName = "cm2";
        FollowerConstants.rightFrontMotorName = "em2";
        FollowerConstants.rightRearMotorName = "cm3";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }

    // Robot-specific settings
    private static void initializeRobotSettings() {
        FollowerConstants.mass = 13.9;
        FollowerConstants.xMovement = 72.6541;
        FollowerConstants.yMovement = 60.696;
        FollowerConstants.forwardZeroPowerAcceleration = -27.0836;
        FollowerConstants.lateralZeroPowerAcceleration = -63.2158;
        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;
    }

    // Initialize PID coefficients for translational, heading, and drive
    private static void initializePIDCoefficients() {
        // Translational PID
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(translationalP, translationalI, translationalD, translationalF);
        if (PIDConfig.useSecondaryTranslationalPID) {
            FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(secondaryTranslationalP, secondaryTranslationalI, secondaryTranslationalD, secondaryTranslationalF);
        }

        // Heading PID
        FollowerConstants.headingPIDFCoefficients.setCoefficients(headingP, headingI, headingD, headingF);
        if (PIDConfig.useSecondaryHeadingPID) {
            FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(secondaryHeadingP, secondaryHeadingI, secondaryHeadingD, secondaryHeadingF);
        }

        // Drive PID
        FollowerConstants.drivePIDFCoefficients.setCoefficients(driveP, driveI, driveD, driveF, 0);
        if (PIDConfig.useSecondaryDrivePID) {
            FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF, 0);
        }
    }

    // Initialize path constraints (timeouts, velocity, etc.)
    private static void initializePathConstraints() {
        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.01;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
