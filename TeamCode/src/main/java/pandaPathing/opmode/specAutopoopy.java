package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw45_2;
import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.clawOpen;
import static pandaPathing.robot.RobotConstants.pitchBOut;
import static pandaPathing.robot.RobotConstants.pitchFDown;
import static pandaPathing.robot.RobotConstants.pitchMUp;
import static pandaPathing.robot.RobotConstants.railLMax;
import static pandaPathing.robot.RobotConstants.railLMin;
import static pandaPathing.robot.RobotConstants.railRMax;
import static pandaPathing.robot.RobotConstants.railRMin;
import static pandaPathing.robot.RobotConstants.slideMax;
import static pandaPathing.robot.RobotConstants.slideMaxSpec;
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBUp;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFUp;
import static pandaPathing.robot.RobotConstants.yaw0;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.robot.Hardware;


@Config
@Disabled
@Autonomous(name = "PoopShoot", group = "Opmode")
public class specAutopoopy extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Hardware robot;
    private boolean slidesUp, slidesDown, rails;
    private int target;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose scorePreloadPose = new Pose(24, 0, Math.toRadians(0));

    private final Pose moveForwardPose = new Pose(28, 0, Math.toRadians(0));

    private final Pose grabPosition1Pose = new Pose(15, -39, Math.toRadians(0));

    private final Pose grabPosition2Pose = new Pose(15, -49.5, Math.toRadians(0));

    private final Pose grabPosition3Pose = new Pose(20, -45, Math.toRadians(-45));

    private final Pose bringBack1Pose = new Pose(12, -39, Math.toRadians(0));

    private final Pose bringBack2Pose = new Pose(12, -49.5, Math.toRadians(0));

    private final Pose bringBack3Pose = new Pose(12, -45, Math.toRadians(0));

    private PathChain hangPreload, moveForward, grabPosition1, grabPosition2, grabPosition3, bringBack1, bringBack2, bringBack3;

    public void buildPaths() {


        hangPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .build();

        moveForward = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(moveForwardPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabPosition1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(moveForwardPose), new Point(grabPosition1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(500)
                .build();

        bringBack1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grabPosition1Pose), new Point(bringBack1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabPosition2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bringBack1Pose), new Point(grabPosition2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(500)
                .build();

        bringBack2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grabPosition2Pose), new Point(bringBack2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grabPosition3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bringBack2Pose), new Point(grabPosition3Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .setPathEndTimeoutConstraint(500)
                .build();

        bringBack3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grabPosition3Pose), new Point(bringBack3Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {

        double slidePos = robot.rightSlides.getCurrentPosition();
        if(slidePos >= slideMaxSpec - 20 && !slidesUp) slidesUp = true;
        else if(slidePos < slideMax - 20) slidesUp = false;
        if(slidePos <= slideMin + 50 && !slidesDown) slidesDown = true;
        else if(slidePos < slideMin - 20) slidesDown = false;

        switch (pathState) {
            case 00: //preload & set max power
                follower.setMaxPower(1);
                robot.v4b.setPosition(v4bFUp);
                robot.pitch.setPosition(pitchMUp);
                robot.lilJarret.setPosition(clawClose);
                robot.railL.setPosition(railLMin);
                robot.railR.setPosition(railRMin);
                robot.roll.setPosition(claw0);
                robot.yaw.setPosition(yaw0);
                rails = false;
                setPathState(01);
                break;

            case 1:
                follower.setMaxPower(1);
                follower.followPath(hangPreload, true);
                setPathState(2);
                break;

            case 2:
                if (follower.getCurrentTValue() > 0.5) target = slideMaxSpec;
                if (slidesUp) {
                    setPathState(3);
                }
                break;

            case 3:
                if (follower.atParametricEnd()) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    rails = true;
                    setPathState(4);
                }
                break;

            case 4:
                follower.setMaxPower(1);
                follower.followPath(moveForward, true);
                setPathState(6);
                break;


            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    rails = false;
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPosition1, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = slideMin;
                    setPathState(10);
                }
                break;



            case 10:
                if (follower.atParametricEnd()) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    rails = true;
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bBUp);
                    robot.pitch.setPosition(pitchBOut);
                    rails = false;
                    setPathState(14);
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(bringBack1, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (follower.atParametricEnd()) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(20);
                }
                break;



            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPosition2, true);
                    setPathState(21);
                }
                break;

            case 21:
                if (follower.atParametricEnd()) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    rails = true;
                    setPathState(22);
                }
                break;

            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(23);
                }
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(24);
                }
                break;

            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bBUp);
                    robot.pitch.setPosition(pitchBOut);
                    rails = false;
                    setPathState(25);
                }
                break;

            case 25:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(bringBack2, true);
                    setPathState(26);
                }
                break;

            case 26:
                if (follower.atParametricEnd()) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(30);
                }
                break;



            case 30:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(grabPosition3, true);
                    setPathState(31);
                }
                break;

            case 31:
                if (follower.atParametricEnd()) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    robot.roll.setPosition(claw45_2);
                    rails = true;
                    setPathState(32);
                }
                break;

            case 32:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(33);
                }
                break;

            case 33:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(34);
                }
                break;

            case 34:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bBUp);
                    robot.pitch.setPosition(pitchBOut);
                    robot.roll.setPosition(claw0);
                    rails = false;
                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(bringBack3, true);
                    setPathState(36);
                }
                break;

            case 36:
                if (follower.atParametricEnd()) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(40);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        robot = new Hardware(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        robot.rightSlides.setPower(robot.slidePidPow(target));
        robot.leftSlides.setPower(robot.slidePidPow(target));

        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("T-Value", follower.getCurrentTValue());
        telemetry.addData("pitch Position", robot.pitch.getPosition());
        telemetry.addData("v4b Position", robot.v4b.getPosition());
        telemetry.update();
    }
}