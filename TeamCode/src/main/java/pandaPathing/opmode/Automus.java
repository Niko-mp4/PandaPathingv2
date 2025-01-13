package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw45;
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
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBackDown;
import static pandaPathing.robot.RobotConstants.v4bBackUp;
import static pandaPathing.robot.RobotConstants.v4bOutDown;
import static pandaPathing.robot.RobotConstants.v4bOutUp;
import static pandaPathing.robot.RobotConstants.yaw0;
import static pandaPathing.robot.RobotConstants.yaw45;
import static pandaPathing.robot.RobotConstants.yaw45_2;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Config
@Autonomous(name = "automUs", group = "Opmode")
public class Automus extends OpMode {

    private Follower follower;
    private Hardware robot;
    private Timer opmodeTimer;
    private static double target = 0;
    private boolean stopped = true;
    private boolean slidesUp, slidesDown;
    private double lastX = 0;
    private double lastY = 0;
    private double lastH = 0;
    private double startTime = System.currentTimeMillis();

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
    private final Pose scorePreloadPose = new Pose(135.5, 20.4, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(128, 11, Math.toRadians(180));

    private final Pose scorePose2 = new Pose(126.5, 11, Math.toRadians(180));

    private final Pose scorePose3 = new Pose(125.6, 11, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(127.1, 23.1, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(125.9, 13.65, Math.toRadians(180));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(122.4, 14.8, Math.toRadians(210));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(72, 48, Math.toRadians(270));

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
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose3), /* Control Point */ new Point(parkPose)));
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
        double time = (System.currentTimeMillis() - startTime) / 1000.0;

        double slidePos = robot.rightSlides.getCurrentPosition();
        if(slidePos >= slideMax - 20 && !slidesUp) slidesUp = true;
        else if(slidePos < slideMax - 20) slidesUp = false;
        if(slidePos <= slideMin + 50 && !slidesDown) slidesDown = true;
        else if(slidePos < slideMin - 20) slidesDown = false;


        switch (pathState) {
            case "0.0": //preload & set max power
                follower.setMaxPower(1);
                robot.v4b.setPosition(v4bBackDown);
                robot.pitch.setPosition(pitchFDown);
                robot.lilJarret.setPosition(clawClose);
                robot.railL.setPosition(railLMin);
                robot.railR.setPosition(railRMin);
                robot.roll.setPosition(claw0);
                robot.yaw.setPosition(yaw0);
                setPathState("0.1");
                startTime = System.currentTimeMillis();
                break;
            case "0.1": //run robo to bucket and lift slides 0
                if (slidePos > 500 && stopped) {
                    follower.followPath(scorePreload);
                    stopped = false;
                }
                target = slideMax;
                if (slidesUp && !follower.isBusy()){
                    setPathState("0.2");
                } else if(time > 0.75){
                    robot.pitch.setPosition(pitchBOut);
                    robot.v4b.setPosition(v4bBackDown-0.065);
                }
                break;
            case "0.2": // score timer 0
                if(time > 1.5) setPathState("1.0");
                else if(time > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                } else if(time > 0.25){
                    robot.yaw.setPosition(yaw45_2);
                }
                break;
            case "1.0": // Go to grab position 1 and extend
                if (stopped) {
                    follower.followPath(grabPickup1);
                    stopped = false;
                }
                if (slidesDown && time > 1 && !follower.isBusy()) setPathState("1.1");
                else if(time > 0.5){
                    robot.pitch.setPosition(pitchFDown);
                }
                else if (time > 0.25) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawClose);
                    robot.yaw.setPosition(yaw0);
                }
                break;
            case "1.1": // Grab 1 and retract
                if (time > 2.5) {
                    if (robot.railR.getPosition() == railRMin) setPathState("1.2");
                    robot.v4b.setPosition(v4bBackUp);
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                } else if (time > 2.25) {
                    robot.lilJarret.setPosition(clawClose);
                } else if (time > 2){
                    robot.v4b.setPosition(v4bOutDown + 0.04);
                } else if(time > 1) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.v4b.setPosition(v4bOutUp);
                }
                break;
            case "1.2": // Go to score position 1 and lift slides
                if (slidePos > 500 && stopped) {
                    follower.followPath(scorePickup1);
                    stopped = false;
                }
                target = slideMax;
                if (slidesUp && !follower.isBusy()){
                    setPathState("1.3");
                } else if(time > 0.75){
                    robot.pitch.setPosition(pitchBOut);
                    robot.v4b.setPosition(v4bBackDown-0.065);
                }
                break;
            case "1.3": // Deposit 1
                if(time > 0.75) setPathState("2.0");
                else if(time > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                } else if(time > 0.25) {
                    robot.yaw.setPosition(yaw45);
                }
                break;
            case "2.0": // Go to grab position 1 and extend
                if (stopped) {
                    follower.followPath(grabPickup2);
                    stopped = false;
                }
                if (slidesDown && time > 1 && !follower.isBusy()) setPathState("2.1");
                else if(time > 0.5){
                    robot.pitch.setPosition(pitchFDown);
                }
                else if (time > 0.25) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawClose);
                    robot.yaw.setPosition(yaw0);
                }
                break;
            case "2.1": // Grab 1 and retract
                if (time > 2.5) {
                    if (robot.railR.getPosition() == railRMin) setPathState("2.2");
                    robot.v4b.setPosition(v4bBackUp);
                    robot.roll.setPosition(claw0);
                    robot.yaw.setPosition(yaw0);
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                } else if (time > 2.25) {
                    robot.lilJarret.setPosition(clawClose);
                } else if (time > 2){
                    robot.v4b.setPosition(v4bOutDown + 0.04);
                } else if(time > 1.5){
                    robot.yaw.setPosition(yaw0 + 0.06);
                }else if(time > 1) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.roll.setPosition(claw0-0.05);
                    robot.v4b.setPosition(v4bOutUp);
                }
                break;
            case "2.2": // Go to score position 1 and lift slides
                if (slidePos > 750 && stopped) {
                    follower.followPath(scorePickup2);
                    stopped = false;
                }
                target = slideMax;
                if (slidesUp && !follower.isBusy()){
                    setPathState("2.3");
                } else if(time > 0.75){
                    robot.pitch.setPosition(pitchBOut);
                    robot.v4b.setPosition(v4bBackDown-0.065);
                }
                break;
            case "2.3": // Deposit 1
                if(time > 0.75) setPathState("3.0");
                else if(time > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                } else if(time > 0.25) {
                    robot.yaw.setPosition(yaw45);
                }
                break;
            case "3.0": // Go to grab position 1 and extend
                if (stopped) {
                    follower.followPath(grabPickup3);
                    stopped = false;
                }
                if (slidesDown && time > 1 && !follower.isBusy()) setPathState("3.1");
                else if(time > 0.5){
                    robot.pitch.setPosition(pitchFDown);
                }
                else if (time > 0.25) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawClose);
                    robot.yaw.setPosition(yaw0);
                }
                break;
            case "3.1": // Grab 1 and retract
                if (time > 2.5) {
                    if (robot.railR.getPosition() == railRMin) setPathState("3.2");
                    robot.v4b.setPosition(v4bBackUp);
                    robot.roll.setPosition(claw0);
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                } else if (time > 2.25) {
                    robot.lilJarret.setPosition(clawClose);
                } else if (time > 2){
                    robot.v4b.setPosition(v4bOutDown + 0.04);
                } else if(time > 1) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.roll.setPosition(claw45_2);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.v4b.setPosition(v4bOutUp);
                }
                break;
            case "3.2": // Go to score position 1 and lift slides
                if (slidePos > 750 && stopped) {
                    follower.followPath(scorePickup3);
                    stopped = false;
                }
                target = slideMax;
                if (slidesUp && !follower.isBusy()){
                    setPathState("3.3");
                } else if(time > 0.75){
                    robot.pitch.setPosition(pitchBOut);
                    robot.v4b.setPosition(v4bBackDown-0.065);
                }
                break;
            case "3.3": // Deposit 1
                if(time > 0.75) setPathState("4.0");
                else if(time > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                } else if(time > 0.25) {
                    robot.yaw.setPosition(yaw45);
                }
                break;
            case "4.0":
                if (stopped) {
                    follower.followPath(park);
                    stopped = false;
                }
                if(time > 4){
                    robot.hangerR.setTargetPosition(5100);
                    robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.hangerR.setPower(1);
                    robot.hangerL.setTargetPosition(5100);
                    robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.hangerL.setPower(1);
                } else if(time > 1)
                    target = slideMin;

                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(String pState) {
        pathState = pState;
        startTime = System.currentTimeMillis();
        stopped = true;
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
        telemetry.addData("x", follower.getXOffset());
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
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
