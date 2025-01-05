package pandaPathing.tuners_tests.pid;

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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.Constants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.constants.PIDConfig;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Straight Back And Forth", group = "PIDF Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        // Update PID coefficients during the loop if needed

        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(translationalP, translationalI, translationalD, translationalF);
        if (PIDConfig.useSecondaryTranslationalPID) {
            FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(secondaryTranslationalP, secondaryTranslationalI, secondaryTranslationalD, secondaryTranslationalF);
        }

        // Heading PID
        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(headingP, headingI, headingD, headingF);
        if (PIDConfig.useSecondaryHeadingPID) {
            FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(secondaryHeadingP, secondaryHeadingI, secondaryHeadingD, secondaryHeadingF);
        }

        // Drive PID
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(driveP, driveI, driveD, driveF, 0);
        if (PIDConfig.useSecondaryDrivePID) {
            FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(secondaryDriveP, secondaryDriveI, secondaryDriveD, secondaryDriveF, 0);
        }

        // Update the follower
        follower.update();

        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        telemetry.addData("drivePID", PIDConfig.driveP);
        follower.telemetryDebug(telemetryA);
        telemetry.update();
    }
}