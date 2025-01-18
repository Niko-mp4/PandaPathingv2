package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.pitchFDown;
import static pandaPathing.robot.RobotConstants.pitchMUp;
import static pandaPathing.robot.RobotConstants.railLMax;
import static pandaPathing.robot.RobotConstants.railLMin;
import static pandaPathing.robot.RobotConstants.railRMax;
import static pandaPathing.robot.RobotConstants.railRMin;
import static pandaPathing.robot.RobotConstants.slideMax;
import static pandaPathing.robot.RobotConstants.slideMaxSpec;
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFUp;
import static pandaPathing.robot.RobotConstants.v4bMUp;
import static pandaPathing.robot.RobotConstants.yaw0;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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
@Autonomous(name = "AmungAuto", group = "Opmode")
public class betterAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Hardware robot;
    private boolean slidesUp, slidesDown;
    private int target;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose scorePreloadPose = new Pose(30, 0, Math.toRadians(0));

    private final Pose moveRightPose = new Pose(25, -20, Math.toRadians(0));

    private final Pose pushPositionPose1 = new Pose(55, -39, Math.toRadians(0));

    private final Pose pushPositionPose2 = new Pose(55, -45.5, Math.toRadians(0));

    private final Pose pushPositionPose3 = new Pose(55, -54.25, Math.toRadians(0));

    private final Pose pushPose1 = new Pose(8, -39, Math.toRadians(0));

    private final Pose pushPose2 = new Pose(8, -50, Math.toRadians(0));

    private final Pose pushPose3 = new Pose(8, -56.5, Math.toRadians(0));

    private final Pose grabPose = new Pose(6, -40, Math.toRadians(0));

    private final Pose hangPose1 = new Pose(25, 5, Math.toRadians(0));

    private final Pose hangPose2 = new Pose(25, 5, Math.toRadians(0));

    private final Pose hangPose3 = new Pose(25, 5, Math.toRadians(0));

    private final Pose hangPose4 = new Pose(25, 5, Math.toRadians(0));

    private final Pose hangPose5 = new Pose(25, 5, Math.toRadians(0));

    private final Pose parkPose = new Pose(72, 48, Math.toRadians(270));


    private PathChain hangPreload, moveRight, pushPosition1, pushPosition2, pushPosition3, push1, push2, push3, hang1, grab2, hang2, grab3, hang3, grab4, hang4, parkAtEnd;

    public void buildPaths() {


        hangPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                .build();

        moveRight = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(moveRightPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pushPosition1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveRightPose), new Point(pushPositionPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        pushPosition2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose1), new Point(pushPositionPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();


        pushPosition3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose2), new Point(pushPositionPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        push1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPositionPose1), new Point(pushPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        push2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPositionPose2), new Point(pushPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        push3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPositionPose3), new Point(pushPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hang1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose3), new Point(hangPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose1), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hang2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(hangPose2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose2), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hang3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(hangPose3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        grab4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose3), new Point(grabPose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        hang4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPose), new Point(hangPose4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        parkAtEnd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose4), new Point(grabPose)))
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
                setPathState(01);
                break;

            case 01:
                follower.setMaxPower(1);
                    follower.followPath(hangPreload, true);
                    setPathState(02);
                    break;

            case 02:
                if(follower.getCurrentTValue() > 0.5) target = slideMaxSpec;
                if(slidesUp){
                    setPathState(03);
                }
                break;

            case 03:
                if(follower.atParametricEnd()) {
                    robot.railR.setPosition(railRMax);
                    robot.railL.setPosition(railLMax);
                    setPathState(04);
                }
                break;

            case 04:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    robot.railR.setPosition(railRMin);
                    robot.railL.setPosition(railLMin);
                    setPathState(10);
                }
                break;


            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    target = slideMin;
                    setPathState(11);
                }
                break;

            case 11:
                if (target == slideMin) {
                    follower.followPath(pushPosition1, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (follower.atParametricEnd()) {
                    follower.followPath(push1, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (follower.atParametricEnd()) {
                    follower.followPath(pushPosition2, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (follower.atParametricEnd()) {
                    follower.followPath(push2, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (follower.atParametricEnd()) {
                    follower.followPath(pushPosition3, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (follower.atParametricEnd()) {
                    follower.followPath(push3, true);
                    setPathState(20);
                }
                break;

            case 20:
                if (follower.atParametricEnd()) {
                    follower.followPath(hang1, true);
                    setPathState(21);
                }
                break;

            case 21:
                if (follower.atParametricEnd()) {
                    follower.followPath(grab2, true);
                    setPathState(22);
                }
                break;

            case 22:
                if (follower.atParametricEnd()) {
                    follower.followPath(hang2, true);
                    setPathState(23);
                }
                break;

            case 23:
                if (follower.atParametricEnd()) {
                    follower.followPath(grab3, true);
                    setPathState(24);
                }
                break;

            case 24:
                if (follower.atParametricEnd()) {
                    follower.followPath(hang3, true);
                    setPathState(25);
                }
                break;

            case 25:
                if (follower.atParametricEnd()) {
                    follower.followPath(grab4, true);
                    setPathState(26);
                }
                break;

            case 26:
                if (follower.atParametricEnd()) {
                    follower.followPath(hang4, true);
                    setPathState(27);
                }
                break;

            case 27:
                if (follower.atParametricEnd()) {
                    follower.followPath(parkAtEnd, true);
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
        telemetry.update();
    }
}