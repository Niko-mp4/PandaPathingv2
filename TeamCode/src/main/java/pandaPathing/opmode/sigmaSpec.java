//package pandaPathing.opmode;
//
//import static pandaPathing.robot.RobotConstants.claw0;
//import static pandaPathing.robot.RobotConstants.claw180;
//import static pandaPathing.robot.RobotConstants.clawClose;
//import static pandaPathing.robot.RobotConstants.clawOpen;
//import static pandaPathing.robot.RobotConstants.pitchBOut;
//import static pandaPathing.robot.RobotConstants.pitchFOut;
//import static pandaPathing.robot.RobotConstants.pitchMUp;
//import static pandaPathing.robot.RobotConstants.railLMax;
//import static pandaPathing.robot.RobotConstants.railLMin;
//import static pandaPathing.robot.RobotConstants.railRMax;
//import static pandaPathing.robot.RobotConstants.railRMin;
//import static pandaPathing.robot.RobotConstants.slideMax;
//import static pandaPathing.robot.RobotConstants.slideMaxSpec;
//import static pandaPathing.robot.RobotConstants.slideMin;
//import static pandaPathing.robot.RobotConstants.v4bBDown;
//import static pandaPathing.robot.RobotConstants.v4bFUp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.pedropathing.util.Timer;
//import pandaPathing.constants.FConstants;
//import pandaPathing.constants.LConstants;
//import pandaPathing.robot.Hardware;
//
//
//@Config
//@Autonomous(name = "sigmaSpec", group = "Opmode")
//public class sigmaSpec extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//    private Hardware robot;
//    private boolean slidesUp, slidesDown, rails;
//    private int target;
//
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); //1
//
//    private final Pose scorePreloadPose = new Pose(22, 0, Math.toRadians(0)); //2
//
//    private final Pose pickUp6thSamplePose = new Pose(28, 0, Math.toRadians(0)); //3
//
//    private final Pose dropOff6thSamplePose = new Pose(6, -24, Math.toRadians(-45)); //4
//
//    private final Pose pickUp1stGroundSamplePose = new Pose(12, -24, Math.toRadians(45)); //5
//
//    private final Pose dropOff1stGroundSamplePose = new Pose(12, -24, Math.toRadians(135)); //6
//
//    private final Pose pickUp2ndGroundSamplePose = new Pose(12, -36, Math.toRadians(45)); //7
//
//    private final Pose dropOff2ndGroundSamplePose = new Pose(12, -36, Math.toRadians(135)); //8
//
//    private final Pose pickUp3rdGroundSamplePose = new Pose(12, -48, Math.toRadians(45)); //9
//
//    private final Pose dropOff3rdGroundSamplePose = new Pose(12, -48, Math.toRadians(135)); //10
//
//    private final Pose pickUpPose1 = new Pose(0, -40, Math.toRadians(0)); //11
//
//    private final Pose hangP1 = new Pose(22, -10, Math.toRadians(0)); //12
//
//    private final Pose pickUpPose2 = new Pose(0, -25, Math.toRadians(0)); //13, 15, 17, 19
//
//    private final Pose hangP2 = new Pose(22, -6, Math.toRadians(0)); //14
//
//    private final Pose hangP3 = new Pose(22, -2, Math.toRadians(0)); //16
//
//    private final Pose hangP4 = new Pose(22, 2, Math.toRadians(0)); //18
//
//    private final Pose hangP5 = new Pose(22, 6, Math.toRadians(0)); //20
//
//    private final Pose parking = new Pose(2, -30, Math.toRadians(0)); //21
//
//
//    private PathChain hangPreload, grab6th, drop6th, grab1G, drop1G, grab2G, drop2G, grab3G, drop3G, grab1, hang1, grab2, hang2, grab3, hang3, grab4, hang4, grab5, hang5, parkAtEnd;
//
//    public void buildPaths() {
//
//
//        hangPreload = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(scorePreloadPose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
//                .build();
//
//        grab6th = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scorePreloadPose), new Point(pickUp6thSamplePose)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        drop6th = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(pickUp6thSamplePose), new Point(dropOff6thSamplePose)))
//                .setLinearHeadingInterpolation(pickUp6thSamplePose.getHeading(), pickUp6thSamplePose.getHeading())
//                .build();
//
//        grab1G = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(dropOff6thSamplePose), new Point(pickUp1stGroundSamplePose)))
//                .setLinearHeadingInterpolation(dropOff6thSamplePose.getHeading(), pickUp1stGroundSamplePose.getHeading())
//                .setPathEndTimeoutConstraint(0)
//                .build();
//
//
//        drop1G = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUp1stGroundSamplePose), new Point(dropOff1stGroundSamplePose)))
//                .setLinearHeadingInterpolation(pickUp1stGroundSamplePose.getHeading(), dropOff1stGroundSamplePose.getHeading())
//                .setPathEndTimeoutConstraint(0)
//                .build();
//
//
//        grab2G = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(dropOff1stGroundSamplePose), new Point(pickUp2ndGroundSamplePose)))
//                .setLinearHeadingInterpolation(dropOff1stGroundSamplePose.getHeading(), pickUp2ndGroundSamplePose.getHeading())
//                .setPathEndTimeoutConstraint(0)
//                .build();
//
//        drop2G = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUp2ndGroundSamplePose), new Point(dropOff2ndGroundSamplePose)))
//                .setLinearHeadingInterpolation(pickUp2ndGroundSamplePose.getHeading(), dropOff2ndGroundSamplePose.getHeading())
//                .setPathEndTimeoutConstraint(0)
//                .build();
//
//        grab3G = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(dropOff2ndGroundSamplePose), new Point(pickUp3rdGroundSamplePose)))
//                .setLinearHeadingInterpolation(dropOff2ndGroundSamplePose.getHeading(), pickUp3rdGroundSamplePose.getHeading())
//                .setPathEndTimeoutConstraint(0)
//                .build();
//
//        drop3G = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUp3rdGroundSamplePose), new Point(dropOff3rdGroundSamplePose)))
//                .setLinearHeadingInterpolation(pickUp3rdGroundSamplePose.getHeading(), dropOff3rdGroundSamplePose.getHeading())
//                .setPathEndTimeoutConstraint(0)
//                .build();
//
//        grab1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(dropOff3rdGroundSamplePose), new Point(pickUpPose1)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        hang1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUpPose1), new Point(hangP1)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
////        strafeGrab2 = follower.pathBuilder()
////                .addPath(new BezierLine(new Point(hangP1), new Point(grab2)))
////                .setConstantHeadingInterpolation(Math.toRadians(0))
////                .build();
//
//        grab2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(hangP1), new Point(pickUpPose2)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        hang2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUpPose2), new Point(hangP2)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
////        strafeGrab3 = follower.pathBuilder()
////                .addPath(new BezierLine(new Point(hangPose2), new Point(strafeGrabPose)))
////                .setConstantHeadingInterpolation(Math.toRadians(0))
////                .build();
//
//        grab3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(hangP2), new Point(pickUpPose2)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        hang3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUpPose2), new Point(hangP3)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
////        strafeGrab4 = follower.pathBuilder()
////                .addPath(new BezierLine(new Point(hangPose3), new Point(strafeGrabPose)))
////                .setConstantHeadingInterpolation(Math.toRadians(0))
////                .build();
//
//        grab4 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(hangP3), new Point(pickUpPose2)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        hang4 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUpPose2), new Point(hangP4)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        grab5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(hangP4), new Point(pickUpPose2)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        hang5 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickUpPose2), new Point(hangP5)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        parkAtEnd = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(hangP5), new Point(parking)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//
//        double slidePos = robot.rightSlides.getCurrentPosition();
//        if(slidePos >= slideMaxSpec - 20 && !slidesUp) slidesUp = true;
//        else if(slidePos < slideMax - 20) slidesUp = false;
//        if(slidePos <= slideMin + 50 && !slidesDown) slidesDown = true;
//        else if(slidePos < slideMin - 20) slidesDown = false;
//
//        switch (pathState) {
//            case 0: //preload & set max power
//                follower.setMaxPower(1);
//                robot.v4b.setPosition(v4bFUp);
//                robot.pitch.setPosition(pitchMUp);
//                robot.lilJarret.setPosition(clawClose);
//                robot.railL.setPosition(railLMin);
//                robot.railR.setPosition(railRMin);
//                robot.roll.setPosition(claw180);
//                rails = false;
//                setPathState(1);
//                break;
//
//            case 1:
//                follower.followPath(hangPreload, true);
//                target = slideMaxSpec ;
//                setPathState(2);
//                break;
//
//            case 2:
//                if (follower.atParametricEnd()) {
//                    robot.railR.setPosition(railRMax);
//                    robot.railL.setPosition(railLMax);
//                    rails = true;
//                    setPathState(205);
//                }
//                break;
//
//            case 205:
//                robot.lilJarret.setPosition(clawOpen);
//
//            case 05:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    robot.pitch.setPosition(pitchFOut);
//                    robot.railR.setPosition(railRMin);
//                    robot.railL.setPosition(railLMin);
//                    rails = false;
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
//                    follower.followPath(grab6th, true);
//                    setPathState(11);
//                }
//                break;
//
//            case 11:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    target = slideMin;
//                    setPathState(12);
//                }
//                break;
//
//            case 12:
//                if (follower.atParametricEnd()) {
//                    robot.railR.setPosition(railRMax);
//                    robot.railL.setPosition(railLMax);
//                    rails = true;
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(pushPosition2, true);
//                    setPathState(14);
//                }
//                break;
//
//            case 14:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(push2, true);
//                    setPathState(15);
//                }
//                break;
//
//            case 15:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(pushPosition3, true);
//                    setPathState(155);
//                }
//                break;
//
//            case 155:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(finalPush, true);
//                    setPathState(16);
//                }
//                break;
//
//            case 16:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bBDown);
//                    robot.pitch.setPosition(pitchBOut);
//                    robot.roll.setPosition(claw0);
//                    setPathState(17);
//                }
//                break;
//
//            case 17:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(push3, true);
//                    setPathState(20);
//                }
//                break;
//
//
//
//            case 20:
//                if (follower.atParametricEnd()) {
//                    robot.lilJarret.setPosition(clawClose);
//                    setPathState(21);
//                }
//                break;
//
//            case 21:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    target = slideMaxSpec;
//                    follower.followPath(hang1, true);
//                    setPathState(22);
//                }
//                break;
//
//            case 22:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bFUp);
//                    robot.pitch.setPosition(pitchMUp);
//                    robot.roll.setPosition(claw180);
//                    setPathState(23);
//                }
//                break;
//
//            case 23:
//                if (follower.atParametricEnd()) {
//                    robot.railR.setPosition(railRMax);
//                    robot.railL.setPosition(railLMax);
//                    rails = true;
//                    setPathState(25);
//                }
//                break;
//
//            case 25:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
//                    robot.lilJarret.setPosition(clawOpen);
//                    setPathState(26);
//                }
//                break;
//
//            case 26:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    robot.pitch.setPosition(pitchFOut);
//                    robot.railR.setPosition(railRMin);
//                    robot.railL.setPosition(railLMin);
//                    rails = false;
//                    setPathState(27);
//                }
//                break;
//
//            case 27:
//                if (!rails) {
//                    follower.followPath(strafeGrab2, true);
//                    setPathState(28);
//                }
//                break;
//
//            case 28:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    target = slideMin;
//                    setPathState(29);
//                }
//                break;
//
//            case 29:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bBDown);
//                    robot.pitch.setPosition(pitchBOut);
//                    robot.roll.setPosition(claw0);
//                    setPathState(30);
//                }
//                break;
//
//
//
//            case 30:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(grab2, true);
//                    setPathState(31);
//                }
//                break;
//
//            case 31:
//                if (follower.atParametricEnd()) {
//                    robot.lilJarret.setPosition(clawClose);
//                    setPathState(32);
//                }
//                break;
//
//            case 32:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    target = slideMaxSpec;
//                    follower.followPath(hang2, true);
//                    setPathState(33);
//                }
//                break;
//
//            case 33:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bFUp);
//                    robot.pitch.setPosition(pitchMUp);
//                    robot.roll.setPosition(claw180);
//                    setPathState(34);
//                }
//                break;
//
//            case 34:
//                if (follower.atParametricEnd()) {
//                    robot.railR.setPosition(railRMax);
//                    robot.railL.setPosition(railLMax);
//                    rails = true;
//                    setPathState(36);
//                }
//                break;
//
//            case 36:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
//                    robot.lilJarret.setPosition(clawOpen);
//                    setPathState(37);
//                }
//                break;
//
//            case 37:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    robot.pitch.setPosition(pitchFOut);
//                    robot.railR.setPosition(railRMin);
//                    robot.railL.setPosition(railLMin);
//                    rails = false;
//                    setPathState(38);
//                }
//                break;
//
//            case 38:
//                if (!rails) {
//                    follower.followPath(strafeGrab3, true);
//                    setPathState(39);
//                }
//                break;
//
//            case 39:
//                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
//                    target = slideMin;
//                    setPathState(40);
//                }
//                break;
//
//            case 40:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bBDown);
//                    robot.pitch.setPosition(pitchBOut);
//                    robot.roll.setPosition(claw0);
//                    setPathState(50);
//                }
//                break;
//
//
//
//            case 50:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(grab3, true);
//                    setPathState(51);
//                }
//                break;
//
//            case 51:
//                if (follower.atParametricEnd()) {
//                    robot.lilJarret.setPosition(clawClose);
//                    setPathState(52);
//                }
//                break;
//
//            case 52:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    target = slideMaxSpec;
//                    follower.followPath(hang3, true);
//                    setPathState(53);
//                }
//                break;
//
//            case 53:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bFUp);
//                    robot.pitch.setPosition(pitchMUp);
//                    robot.roll.setPosition(claw180);
//                    setPathState(54);
//                }
//                break;
//
//            case 54:
//                if (follower.atParametricEnd()) {
//                    robot.railR.setPosition(railRMax);
//                    robot.railL.setPosition(railLMax);
//                    rails = true;
//                    setPathState(56);
//                }
//                break;
//
//            case 56:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
//                    robot.lilJarret.setPosition(clawOpen);
//                    setPathState(57);
//                }
//                break;
//
//            case 57:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    robot.pitch.setPosition(pitchFOut);
//                    robot.railR.setPosition(railRMin);
//                    robot.railL.setPosition(railLMin);
//                    rails = false;
//                    setPathState(58);
//                }
//                break;
//
//            case 58:
//                if (!rails) {
//                    follower.followPath(strafeGrab4, true);
//                    setPathState(59);
//                }
//                break;
//
//            case 59:
//                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
//                    target = slideMin;
//                    setPathState(60);
//                }
//                break;
//
//            case 60:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bBDown);
//                    robot.pitch.setPosition(pitchBOut);
//                    robot.roll.setPosition(claw0);
//                    setPathState(70);
//                }
//                break;
//
//
//
//            case 70:
//                if (follower.atParametricEnd()) {
//                    follower.followPath(grab4, true);
//                    setPathState(71);
//                }
//                break;
//
//            case 71:
//                if (follower.atParametricEnd()) {
//                    robot.lilJarret.setPosition(clawClose);
//                    setPathState(72);
//                }
//                break;
//
//            case 72:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    target = slideMaxSpec;
//                    follower.followPath(hang4, true);
//                    setPathState(73);
//                }
//                break;
//
//            case 73:
//                if (follower.getCurrentTValue() > 0.2) {
//                    robot.v4b.setPosition(v4bFUp);
//                    robot.pitch.setPosition(pitchMUp);
//                    robot.roll.setPosition(claw180);
//                    setPathState(74);
//                }
//                break;
//
//            case 74:
//                if (follower.atParametricEnd()) {
//                    robot.railR.setPosition(railRMax);
//                    robot.railL.setPosition(railLMax);
//                    rails = true;
//                    setPathState(76);
//                }
//                break;
//
//            case 76:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
//                    robot.lilJarret.setPosition(clawOpen);
//                    setPathState(77);
//                }
//                break;
//
//            case 77:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
//                    robot.pitch.setPosition(pitchFOut);
//                    robot.railR.setPosition(railRMin);
//                    robot.railL.setPosition(railLMin);
//                    rails = false;
//                    setPathState(78);
//                }
//                break;
//
//            case 78:
//                if (!rails) {
//                    follower.followPath(grab5, true);
//                    setPathState(79);
//                }
//                break;
//
//            case 79:
//                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
//                    target = slideMin;
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        robot = new Hardware(hardwareMap);
//        follower.setStartingPose(startPose);
//        follower.setMaxPower(1);
//        robot.v4b.setPosition(v4bFUp);
//        robot.pitch.setPosition(pitchMUp);
//        robot.lilJarret.setPosition(clawClose);
//        robot.railL.setPosition(railLMin);
//        robot.railR.setPosition(railRMin);
//        robot.roll.setPosition(claw180);
//        rails = false;
//        buildPaths();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//        robot.rightSlides.setPower(robot.slidePidPow(target));
//        robot.leftSlides.setPower(robot.slidePidPow(target));
//
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("Position", follower.getPose().toString());
//        telemetry.addData("T-Value", follower.getCurrentTValue());
//        telemetry.addData("pitch Position", robot.pitch.getPosition());
//        telemetry.addData("v4b Position", robot.v4b.getPosition());
//        telemetry.addData("R Slide", robot.rightSlides.getCurrentPosition());
//        telemetry.addData("L Slide", robot.leftSlides.getCurrentPosition());
//        telemetry.update();
//    }
//}