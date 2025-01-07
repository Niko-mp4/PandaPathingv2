package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw45_2;
import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.clawOpen;
import static pandaPathing.robot.RobotConstants.f;
import static pandaPathing.robot.RobotConstants.pitchBOut;
import static pandaPathing.robot.RobotConstants.pitchFDown;
import static pandaPathing.robot.RobotConstants.railLMax;
import static pandaPathing.robot.RobotConstants.railLMin;
import static pandaPathing.robot.RobotConstants.railRMax;
import static pandaPathing.robot.RobotConstants.railRMin;
import static pandaPathing.robot.RobotConstants.slideMax;
import static pandaPathing.robot.RobotConstants.v4bBackDown;
import static pandaPathing.robot.RobotConstants.v4bBackUp;
import static pandaPathing.robot.RobotConstants.v4bOutDown;
import static pandaPathing.robot.RobotConstants.v4bOutUp;
import static pandaPathing.robot.RobotConstants.yaw0;
import static pandaPathing.robot.RobotConstants.yaw45;
import static pandaPathing.robot.RobotConstants.yaw45_2;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.robot.Hardware;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "automUs", group = "Opmode")
public class Automus extends OpMode {

    private Follower follower;
    private Hardware robot;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private static double target = 0;
    private boolean stopped = true;
    private double lastX = 0;
    private double lastY = 0;
    private double lastH = 0;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private String pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */


    // as x goes to 0 it goes to the submersable
    // as y goes to 0 it gets closer to thr basket
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(136.5, 31.5, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePreloadPose = new Pose(134, 22, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(131, 17, Math.toRadians(120));

    private final Pose scorePose2 = new Pose(129, 16, Math.toRadians(120));

    private final Pose scorePose3 = new Pose(131, 17, Math.toRadians(120));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(127, 22.5, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(127, 15, Math.toRadians(180));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(122, 15, Math.toRadians(210));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(72, 48, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(72, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePreloadPose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePreloadPose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose1.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose3.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose3), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());
    }


    public Path addPath(double targetX, double targetY, double targetH) {
        Point startPoint = new Point(lastX, lastY, Point.CARTESIAN);
        Point endPoint = new Point(targetX, targetY, Point.CARTESIAN);
        Path path = new Path(new BezierLine(startPoint, endPoint));
        if(lastX == targetX && lastY == targetY) path = new Path(new BezierPoint(endPoint));
        path.setLinearHeadingInterpolation(Math.toRadians(lastH), Math.toRadians(targetH));
        lastX = targetX;
        lastY = targetY;
        lastH = targetH;
        return path;
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        double time = pathTimer.getElapsedTimeSeconds();
        double slidePos = robot.rightSlides.getCurrentPosition();
        switch (pathState) {
            case "0.0": //preload & set max power
                follower.setMaxPower(1);
                robot.v4b.setPosition(v4bBackDown);
                robot.pitch.setPosition(pitchBOut);
                robot.lilJarret.setPosition(clawClose);
                robot.railL.setPosition(railLMin);
                robot.railR.setPosition(railRMin);
                robot.roll.setPosition(claw0);
                robot.yaw.setPosition(yaw0);
                setPathState("0.1");
                break;
            case "0.1": //run robo to bucket and lift slides 0
                if(stopped){
                    follower.followPath(scorePreload);
                    stopped = false;
                }
                target = slideMax;
                if(slidePos >= target - 10){
                    setPathState("0.2");
                    stopped = true;
                }
                break;
            case "0.2": // score timer 0
                if(time > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState("1.0");
                }
                else if(time > 0.25){
                    robot.yaw.setPosition(yaw45_2);
                    robot.v4b.setPosition(v4bBackDown);
                }
                break;
            case "1.0": // grab 1
                if(stopped){
                    follower.followPath(grabPickup1);
                    stopped = false;
                }

                if(time > 0.5){
                    // Rails out at 1 sec
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.v4b.setPosition(v4bOutUp);
                    if(slidePos <= target+50){
                        setPathState("1.1");
                        stopped = true;
                    }
                } else if(time > 0.25) {
                    // Slides down and servos in at 0.5 sec
                    target = 0;
                    robot.v4b.setPosition(v4bBackUp);
                    robot.lilJarret.setPosition(clawClose);
                    robot.yaw.setPosition(yaw0);
                    robot.pitch.setPosition(pitchFDown);
                }
                break;
            case "1.1": //grab 1
                if(time > 0.75) {
                    robot.v4b.setPosition(v4bBackUp);
                    robot.pitch.setPosition(pitchBOut);
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                } else if(time > 0.25){
                    robot.lilJarret.setPosition(clawClose);
                } else{
                    robot.v4b.setPosition(v4bOutDown);
                }
                if(robot.railR.getPosition() == railRMin) setPathState("1.2");
                break;
            case "1.2": // score 1
                if(stopped){
                    follower.followPath(scorePickup1);
                    stopped = false;
                }
                target = slideMax;
                if(time > 0.5 && slidePos >= target-10) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState("2.0");
                    stopped = true;
                }
                else if(time > 0.25){
                    robot.yaw.setPosition(yaw45);
                    robot.v4b.setPosition(v4bBackDown);
                }
                break;
            case "2.0": // grab 2
                if(stopped){
                    follower.followPath(grabPickup2);
                    stopped = false;
                }

                if(time > 0.5){
                    // Rails out at 0.5 sec
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.v4b.setPosition(v4bOutUp);
                    if(slidePos <= target+50){
                        setPathState("2.1");
                        stopped = true;
                    }
                } else if(time > 0.25) {
                    // Slides down and servos in at 0.25 sec
                    target = 0;
                    robot.v4b.setPosition(v4bBackUp);
                    robot.lilJarret.setPosition(clawClose);
                    robot.yaw.setPosition(yaw0);
                    robot.pitch.setPosition(pitchFDown);
                }
                break;
            case "2.1": //grab 1
                if(time > 0.75) {
                    robot.v4b.setPosition(v4bBackUp);
                    robot.pitch.setPosition(pitchBOut);
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                } else if(time > 0.25){
                    robot.lilJarret.setPosition(clawClose);
                } else{
                    robot.v4b.setPosition(v4bOutDown);
                }
                if(robot.railR.getPosition() == railRMin) setPathState("2.2");
                break;
            case "2.2": // score 1
                if(stopped){
                    follower.followPath(scorePickup2);
                    stopped = false;
                }
                target = slideMax;
                if(time > 0.5 && slidePos >= target-10) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState("3.0");
                    stopped = true;
                }
                else if(time > 0.25){
                    robot.yaw.setPosition(yaw45);
                    robot.v4b.setPosition(v4bBackDown);
                }
                break;
            case "3.0": // grab 1
                if(stopped){
                    follower.followPath(grabPickup3);
                    stopped = false;
                }

                if(time > 1){
                    // Rails out at 1 sec
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.v4b.setPosition(v4bOutUp);
                    if(slidePos <= target+50){
                        setPathState("3.1");
                        stopped = true;
                    }
                } else if(time > 0.5) {
                    // Slides down and servos in at 0.5 sec
                    target = 0;
                    robot.v4b.setPosition(v4bBackUp);
                    robot.lilJarret.setPosition(clawClose);
                    robot.yaw.setPosition(yaw0);
                    robot.pitch.setPosition(pitchFDown);
                }
                break;
            case "3.1": //grab 1
                if(time > 0.75) {
                    robot.v4b.setPosition(v4bBackUp);
                    robot.pitch.setPosition(pitchBOut);
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                } else if(time > 0.25){
                    robot.lilJarret.setPosition(clawClose);
                } else{
                    robot.v4b.setPosition(v4bOutDown);
                }
                if(robot.railR.getPosition() == railRMin) setPathState("3.2");
                break;
            case "3.2": // score 1
                if(stopped){
                    follower.followPath(scorePickup3);
                    stopped = false;
                }
                target = slideMax;
                if(time > 0.5 && slidePos >= target-10) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState("4.0");
                    stopped = true;
                }
                else if(time > 0.25){
                    robot.yaw.setPosition(yaw45);
                    robot.v4b.setPosition(v4bBackDown);
                    target = 0;
                    follower.followPath(park);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(String pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Constantly run slides to their target
        robot.rightSlides.setPower(robot.slidePidPow(target));
        robot.leftSlides.setPower(robot.slidePidPow(target));

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        robot = new Hardware(hardwareMap);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("0.0");
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

