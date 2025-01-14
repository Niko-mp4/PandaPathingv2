package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.pitchFDown;
import static pandaPathing.robot.RobotConstants.railLMin;
import static pandaPathing.robot.RobotConstants.railRMin;
import static pandaPathing.robot.RobotConstants.slideMax;
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBackDown;
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

    private final Pose startPose = new Pose(136.5, 31.5, Math.toRadians(90));

    private final Pose scorePreloadPose = new Pose(135.5, 20.4, Math.toRadians(90));

    private final Pose scorePose1 = new Pose(128, 11, Math.toRadians(180));

    private final Pose scorePose2 = new Pose(126.5, 11, Math.toRadians(180));

    private final Pose scorePose3 = new Pose(125.6, 11, Math.toRadians(180));

    private final Pose pickup1Pose = new Pose(127.1, 23.1, Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(125.9, 13.65, Math.toRadians(180));

    private final Pose pickup3Pose = new Pose(122.4, 14.8, Math.toRadians(210));

    private final Pose parkPose = new Pose(72, 48, Math.toRadians(270));

    private final Pose parkControlPose = new Pose(72, 98, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {


        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePreloadPose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickup2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
                .build();

        park = new Path(new BezierCurve(new Point(scorePose3), /* Control Point */ new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {

        double slidePos = robot.rightSlides.getCurrentPosition();
        if(slidePos >= slideMax - 20 && !slidesUp) slidesUp = true;
        else if(slidePos < slideMax - 20) slidesUp = false;
        if(slidePos <= slideMin + 50 && !slidesDown) slidesDown = true;
        else if(slidePos < slideMin - 20) slidesDown = false;

        switch (pathState) {
            case 0: //preload & set max power
                follower.setMaxPower(1);
                robot.v4b.setPosition(v4bBackDown);
                robot.pitch.setPosition(pitchFDown);
                robot.lilJarret.setPosition(clawClose);
                robot.railL.setPosition(railLMin);
                robot.railR.setPosition(railRMin);
                robot.roll.setPosition(claw0);
                robot.yaw.setPosition(yaw0);
                setPathState(1);
                break;

            case 1:
                    follower.followPath(scorePreload);
                    setPathState(20);
                    break;

            case 20: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(21);
                }
                break;

            case 21: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy() /*follower.getCurrentTValue() > .95 --- makes it so after the last 95% complete it moves onto this path. Change .95 with whatever percentage value.*/) {
                    follower.followPath(scorePickup1, true);
                    setPathState(30);
                }
                break;

            case 30: // Wait until the robot is near the first sample pickup position
                if (follower.atParametricEnd()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(31);
                }
                break;

            case 31: // Wait until the robot is near the first sample pickup position
                if (follower.atParametricEnd()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(6);
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
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }
}