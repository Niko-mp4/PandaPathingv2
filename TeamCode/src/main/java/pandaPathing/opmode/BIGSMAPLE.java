package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw45_2;
import static pandaPathing.robot.RobotConstants.claw90;
import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.clawOpen;
import static pandaPathing.robot.RobotConstants.pitchBOut;
import static pandaPathing.robot.RobotConstants.pitchFDown;
import static pandaPathing.robot.RobotConstants.railLMax;
import static pandaPathing.robot.RobotConstants.railLMin;
import static pandaPathing.robot.RobotConstants.railRMax;
import static pandaPathing.robot.RobotConstants.railRMin;
import static pandaPathing.robot.RobotConstants.slideMax;
import static pandaPathing.robot.RobotConstants.slideMaxSpec;
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBDown;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFUp;
import static pandaPathing.robot.RobotConstants.v4bMUp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.robot.Hardware;


@Config
@Autonomous(name = "BIGSMAPLE", group = "Opmode")
public class BIGSMAPLE extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Hardware robot;
    private boolean slidesUp, slidesDown, rails;
    private int target;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(-90));

    private final Pose scorePose = new Pose(5, 12, Math.toRadians(-45));

    private final Pose humanPlayerSamplePose = new Pose(7, -65, Math.toRadians(-90));

    private final Pose firstSamplePose = new Pose(13, 11.5, Math.toRadians(0));

    private final Pose firstScorePose = new Pose(11, 14, Math.toRadians(-45));

    private final Pose secondSamplePose = new Pose(13, 16, Math.toRadians(0));

    private final Pose secondScorePose = new Pose(10, 16, Math.toRadians(-45));

    private final Pose thirdSamplePose = new Pose(15, 18.5, Math.toRadians(20));

    private final Pose thirdScorePose = new Pose(10, 16, Math.toRadians(-45));

    private final Pose parkPose = new Pose(45, -5, Math.toRadians(0));

    private PathChain preload, humanPlayerSample, firstSample, firstScore, secondSample, secondScore, thirdSample, thirdScore, park;

    public void buildPaths() {


        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        humanPlayerSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(humanPlayerSamplePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), humanPlayerSamplePose.getHeading())
                .build();

        firstSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstScorePose), new Point(firstSamplePose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), firstSamplePose.getHeading())
                .build();

        firstScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(firstScorePose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstScorePose.getHeading())
                .build();

        secondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstScorePose), new Point(secondSamplePose)))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), secondSamplePose.getHeading())
                .build();

        secondScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSamplePose), new Point(secondScorePose)))
                .setLinearHeadingInterpolation(secondSamplePose.getHeading(), secondScorePose.getHeading())
                .build();

        thirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondScorePose), new Point(thirdSamplePose)))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), thirdSamplePose.getHeading())
                .build();

        thirdScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSamplePose), new Point(thirdScorePose)))
                .setLinearHeadingInterpolation(thirdSamplePose.getHeading(), thirdScorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdScorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        double slidePos = robot.rightSlides.getCurrentPosition();
        if(slidePos >= slideMaxSpec - 20 && !slidesUp) slidesUp = true;
        else if(slidePos < slideMax - 20) slidesUp = false;
        if(slidePos <= slideMin + 50 && !slidesDown) slidesDown = true;
        else if(slidePos < slideMin - 20) slidesDown = false;

        switch (pathState) {
            case 0: //preload & set max power
                follower.setMaxPower(1);
                robot.v4b.setPosition(v4bMUp);
                robot.pitch.setPosition(pitchBOut);
                robot.lilJarret.setPosition(clawClose);
                robot.railL.setPosition(railLMin);
                robot.railR.setPosition(railRMin);
                robot.roll.setPosition(claw90);
                rails = false;
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(preload, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (follower.getCurrentTValue() > 0.01)
                    target = slideMax;
                if (slidesUp) {
                    setPathState(3);
                }
                break;

            case 3:
                if (slidePos > (slideMax - 50)) {
                    robot.v4b.setPosition(v4bBDown);
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(humanPlayerSample, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    robot.roll.setPosition(claw0);
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    rails = true;
                    setPathState(13);
                }
                break;

            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(14);
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(15);
                }
                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bMUp);
                    robot.pitch.setPosition(pitchBOut);
                    rails = false;
                    setPathState(20);
                }
                break;



            case 20:
                if (!rails) {
                    follower.followPath(firstScore, true);
                    setPathState(21);
                }
                break;

            case 21:
                if (follower.getCurrentTValue() > 0.01)
                    target = slideMax;
                if (slidesUp) {
                    setPathState(22);
                }
                break;

            case 22:
                if (slidePos > (slideMax - 50)) {
                    robot.v4b.setPosition(v4bBDown);
                    robot.roll.setPosition(claw90);
                    setPathState(23);
                }
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(24);
                }
                break;

            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(firstSample, true);
                    setPathState(25);
                }
                break;

            case 25:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(26);
                }
                break;

            case 26:
                if (!follower.isBusy()) {
                    robot.roll.setPosition(claw0);
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    rails = true;
                    setPathState(27);
                }
                break;

            case 27:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(28);
                }
                break;

            case 28:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(29);
                }
                break;

            case 29:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bMUp);
                    robot.pitch.setPosition(pitchBOut);
                    rails = false;
                    setPathState(30);
                }
                break;



            case 30:
                if (!rails) {
                    follower.followPath(firstScore, true);
                    setPathState(31);
                }
                break;

            case 31:
                if (follower.getCurrentTValue() > 0.01)
                    target = slideMax;
                if (slidesUp) {
                    setPathState(32);
                }
                break;

            case 32:
                if (slidePos > (slideMax - 50)) {
                    robot.v4b.setPosition(v4bBDown);
                    robot.roll.setPosition(claw90);
                    setPathState(33);
                }
                break;

            case 33:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(34);
                }
                break;

            case 34:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(secondSample, true);
                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(36);
                }
                break;

            case 36:
                if (!follower.isBusy()) {
                    robot.roll.setPosition(claw0);
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    rails = true;
                    setPathState(37);
                }
                break;

            case 37:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(38);
                }
                break;

            case 38:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(39);
                }
                break;

            case 39:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bMUp);
                    robot.pitch.setPosition(pitchBOut);
                    rails = false;
                    setPathState(40);
                }
                break;



            case 40:
                if (!rails) {
                    follower.followPath(secondScore, true);
                    setPathState(41);
                }
                break;

            case 41:
                if (follower.getCurrentTValue() > 0.01)
                    target = slideMax;
                if (slidesUp) {
                    setPathState(42);
                }
                break;

            case 42:
                if (slidePos > (slideMax - 50)) {
                    robot.v4b.setPosition(v4bBDown);
                    robot.roll.setPosition(claw90);
                    setPathState(43);
                }
                break;

            case 43:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(44);
                }
                break;

            case 44:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(thirdSample, true);
                    setPathState(45);
                }
                break;

            case 45:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = slideMin;
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(46);
                }
                break;

            case 46:
                if (!follower.isBusy()) {
                    robot.roll.setPosition(claw0);
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    robot.v4b.setPosition(v4bFUp);
                    robot.pitch.setPosition(pitchFDown);
                    robot.roll.setPosition(claw45_2);
                    rails = true;
                    setPathState(47);
                }
                break;

            case 47:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    robot.v4b.setPosition(v4bFDown);
                    setPathState(48);
                }
                break;

            case 48:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawClose);
                    setPathState(49);
                }
                break;

            case 49:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    robot.v4b.setPosition(v4bMUp);
                    robot.pitch.setPosition(pitchBOut);
                    robot.roll.setPosition(claw0);
                    rails = false;
                    setPathState(50);
                }
                break;



            case 50:
                if (!rails) {
                    follower.followPath(thirdScore, true);
                    setPathState(51);
                }
                break;

            case 51:
                if (follower.getCurrentTValue() > 0.01)
                    target = slideMax;
                if (slidesUp) {
                    setPathState(52);
                }
                break;

            case 52:
                if (slidePos > (slideMax - 50)) {
                    robot.v4b.setPosition(v4bBDown);
                    robot.roll.setPosition(claw90);
                    setPathState(53);
                }
                break;

            case 53:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.lilJarret.setPosition(clawOpen);
                    setPathState(54);
                }
                break;

            case 54:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(park, true);
                    setPathState(55);
                }
                break;

            case 55:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    target = slideMin;
                    robot.v4b.setPosition(v4bMUp);
                    robot.pitch.setPosition(pitchBOut);
                    robot.lilJarret.setPosition(clawOpen);
                    robot.railL.setPosition(railLMin);
                    robot.railR.setPosition(railRMin);
                    robot.roll.setPosition(claw0);
                    setPathState(56);
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
        telemetry.addData("target", target);
        telemetry.addData("slide position", robot.rightSlides.getCurrentPosition());
        telemetry.update();
    }
}