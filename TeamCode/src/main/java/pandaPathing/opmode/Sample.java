package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw180;
import static pandaPathing.robot.RobotConstants.claw45;
import static pandaPathing.robot.RobotConstants.claw45_2;
import static pandaPathing.robot.RobotConstants.claw90;
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
import static pandaPathing.robot.RobotConstants.v4bBDown;
import static pandaPathing.robot.RobotConstants.v4bBUp;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFUp;
import static pandaPathing.robot.RobotConstants.v4bMUp;
import static pandaPathing.robot.RobotConstants.yaw0;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.robot.Hardware;


@Config
@Autonomous(name = "Sample Boiii", group = "Opmode")
public class Sample extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Hardware robot;
    private boolean slidesUp, slidesDown, rails;
    private int target;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(-90));

    private final Pose preloadPose = new Pose(5, 12, Math.toRadians(-45));

    private final Pose firstSamplePose = new Pose(13, 9, Math.toRadians(0));

    private PathChain preload, firstSample;

    public void buildPaths() {


        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .build();

        firstSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(firstSamplePose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), firstSamplePose.getHeading())
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
                robot.v4b.setPosition(v4bMUp);
                robot.pitch.setPosition(pitchBOut);
                robot.lilJarret.setPosition(clawClose);
                robot.railL.setPosition(railLMin);
                robot.railR.setPosition(railRMin);
                robot.roll.setPosition(claw90);
                robot.yaw.setPosition(yaw0);
                rails = false;
                setPathState(01);
                break;

            case 1:
                follower.setMaxPower(1);
                follower.followPath(preload, true);
                setPathState(2);
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
                    follower.followPath(firstSample, true);
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
                    setPathState(16);
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