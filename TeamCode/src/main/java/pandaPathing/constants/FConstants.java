package pandaPathing.constants;

import static pandaPathing.constants.PIDConfig.driveD;
import static pandaPathing.constants.PIDConfig.driveF;
import static pandaPathing.constants.PIDConfig.driveI;
import static pandaPathing.constants.PIDConfig.driveP;
import static pandaPathing.constants.PIDConfig.headingD;
import static pandaPathing.constants.PIDConfig.headingF;
import static pandaPathing.constants.PIDConfig.headingI;
import static pandaPathing.constants.PIDConfig.headingP;
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
            FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(translationalP, translationalI, translationalD, translationalF);
                if (PIDConfig.useSecondaryTranslationalPID) {
            FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(translationalP, translationalI, translationalD, translationalF);
        }

        // Heading PID
            FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(headingP, headingI, headingD, headingF);
                if (PIDConfig.useSecondaryHeadingPID) {
            FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(headingP, headingI, headingD, headingF);
        }

        // Drive PID
            FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(driveP, driveI, driveD, driveF, 0);
                if (PIDConfig.useSecondaryDrivePID) {
            FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(driveP, driveI, driveD, driveF, 0);
        }
    }

    // Initialize path constraints (timeouts, velocity, etc.)
    private static void initializePathConstraints() {
        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
