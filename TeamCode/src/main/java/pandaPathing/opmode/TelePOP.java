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
import static pandaPathing.robot.RobotConstants.pitchFOut;
import static pandaPathing.robot.RobotConstants.pitchMUp;
import static pandaPathing.robot.RobotConstants.railLMax;
import static pandaPathing.robot.RobotConstants.railLMin;
import static pandaPathing.robot.RobotConstants.railRMax;
import static pandaPathing.robot.RobotConstants.railRMin;
import static pandaPathing.robot.RobotConstants.slideMax;
import static pandaPathing.robot.RobotConstants.slideMaxSpec;
import static pandaPathing.robot.RobotConstants.slideMaxSpecTele;
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBDown;
import static pandaPathing.robot.RobotConstants.v4bBUp;
import static pandaPathing.robot.RobotConstants.v4bMUp;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFOut;
import static pandaPathing.robot.RobotConstants.v4bFUp;
import static pandaPathing.robot.RobotConstants.yaw0;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.robot.Hardware;

@Config
@TeleOp(name = "telelelelelellel", group = "Drive")
public class TelePOP extends OpMode {
    private Hardware robot;
    private Follower follower;
    private Path holdHeading;

    boolean dUpPressed, dDownPressed, dLeftPressed, yPressed, y1Pressed, aPressed, a1Pressed, rBumpPressed, lBumpPressed, backPressed, back2Pressed, xPressed, x1Pressed, bPressed, b1Pressed,
            clawIsOpen = false, extended = false, neckUp = true, slowMode = false, hanging = false, slidesUp = false, slidesDown = false, clawOut = false,
            depositAction = true, grabAction = false, specRetractAction = true,
            scoringSpec = false, specMode = false, firstRun = true, headingLock = true, teleRestart = false;
    double slideTarget = slideMin, extendPosR = railRMin, extendPosL = railLMin, v4bPos = v4bMUp,
            depositTime, depositStartTime, grabStartTime, grabTime, specRetractStartTime, specRetractTime,
            driveSpeed, slideSpeed, mult = 1;

    public void init() {
        robot = new Hardware(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.startTeleopDrive();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.v4b.setPosition(v4bMUp);
        robot.pitch.setPosition(pitchFDown);
        robot.lilJarret.setPosition(clawClose);
        clawIsOpen = false;
        robot.roll.setPosition(claw0);
        robot.yaw.setPosition(yaw0);
        robot.railL.setPosition(railLMin);
        robot.railR.setPosition(railRMin);
    }

    public void loop() {

        follower.setTeleOpMovementVectors(
                slideSpeed*driveSpeed*-gamepad1.left_stick_y,
                2*slideSpeed*driveSpeed * (gamepad1.left_trigger > 0 ? gamepad1.left_trigger : (gamepad1.right_trigger > 0 ? -gamepad1.right_trigger : 0)),
                slideSpeed*driveSpeed * -gamepad1.right_stick_x,
                true);
        /*if(clawOut && specMode && !headingLock){\
            Follower.useHeading = true;
            holdHeading = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(1,0, Point.CARTESIAN)));
            holdHeading.setConstantHeadingInterpolation(0);
            follower.holdPoint(new Point(follower.getXOffset(), follower.getYOffset()), Math.toRadians(0));
            teleRestart = false;
            headingLock = true;
        } else if(!clawOut){
            headingLock = false;
        }
        if(headingLock && !teleRestart){
            follower.startTeleopDrive();
            Follower.useHeading = false;
            Follower.useTranslational = false;
            Follower.useCentripetal = false;
            Follower.useDrive = false;
            teleRestart = true;
        }*/

        follower.update();

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // automatic speed control
        if (robot.railL.getPosition() == railLMax || (clawOut && specMode))
            driveSpeed = 0.2;
        else driveSpeed = mult;

        slideSpeed = Math.sqrt((robot.rightSlides.getCurrentPosition() - 1510.41666) / -1510.41666);
        if(scoringSpec) slideSpeed *= 1.2;

        // press 'left bumper' to toggle speed slow/fast (driver 1)
        if (gamepad1.left_bumper && !lBumpPressed) {
            if (!slowMode) {
                mult = 0.33;
                slowMode = true;
            } else {
                mult = 1;
                slowMode = false;
            }
            lBumpPressed = true;
        } else if (!gamepad1.left_bumper) lBumpPressed = false;

        robot.rightSlides.setPower(robot.slidePidPow(slideTarget));
        robot.leftSlides.setPower(robot.slidePidPow(slideTarget));

        // press 'dpad up/down' to set slide target to one of 4 positions (driver 2)
        if (gamepad2.dpad_up && !dUpPressed) {
            slideTarget = scoringSpec ? slideMaxSpecTele : slideMax; // top bar for spec. or top basket
            dUpPressed = true;
        } else if (!gamepad2.dpad_up) dUpPressed = false;
        if (gamepad2.dpad_down && !dDownPressed) {
            slideTarget = slideMin; // or slides down all the way
            dDownPressed = true;
        } else if (!gamepad2.dpad_down) dDownPressed = false;
        if (gamepad2.dpad_left && !dLeftPressed) {
            slideTarget = 700; // or slides down all the way
            dLeftPressed = true;
        } else if (!gamepad2.dpad_left) dLeftPressed = false;
        // automatic deposit setup when slides are up
        if (robot.rightSlides.getCurrentPosition() >= slideMax - 20 && !slidesUp) {
            robot.pitch.setPosition(pitchBOut); // set claw above basket
            v4bPos = v4bBDown;
            slidesUp = true;
        } else if (robot.rightSlides.getCurrentPosition() <= slideMax - 100) slidesUp = false;
        if (slideTarget <= slideMin)
            slideTarget = slideMin; // reset slide position if out of bounds
        if (slideTarget >= slideMax) slideTarget = slideMax;

        //if(!robot.touchSensor.getState() && !slidesDown){
        // automatically reset slide zero point when touch sensor is triggered
        //robot.rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidesDown = true;
        //} else if(robot.touchSensor.getState()) slidesDown = false;

        // press 'a' to toggle rail extension in/out (driver 2)
        robot.railR.setPosition(extendPosR);
        robot.railL.setPosition(extendPosL);
        if (gamepad2.a && !aPressed) {
            if (!extended) { // claw open & out when extend || extend into bar for spec
                railExtend();
                extended = true;
            } else { // claw close & tuck when retract
                railRetract();
                extended = false;
            }
            aPressed = true;
        } else if (!gamepad2.a) aPressed = false;
        robot.railR.resetDeviceConfigurationForOpMode();
        robot.railL.resetDeviceConfigurationForOpMode();

        // press 'right bumper' to toggle v4b grabbing up/down (driver 1)
        robot.v4b.setPosition(v4bPos);
        if (gamepad1.right_bumper && extended && !rBumpPressed) {
            if (!neckUp) {
                v4bPos = v4bFUp;
                grabStartTime = System.currentTimeMillis();
                neckUp = true;
            } else {
                v4bPos = v4bFDown;
                grabStartTime = System.currentTimeMillis();
                neckUp = false;
            }
            rBumpPressed = true;
        } else if (!gamepad1.right_bumper) rBumpPressed = false;
        grabTime = (System.currentTimeMillis() - grabStartTime) / 1000.0;
        if (grabTime > 0.12 && !grabAction) { // timer to open/close claw after v4b moves
            robot.lilJarret.setPosition(neckUp ? clawOpen : clawClose);
            clawIsOpen = neckUp;
            grabAction = true;
        } else if (grabTime < 0.12) grabAction = false;

        // press 'b' to deposit sample (driver 2)
        if (gamepad2.b && !bPressed && !extended) {
            if (!clawOut) {
                v4bPos = v4bBDown;
                robot.roll.setPosition(claw0);
                robot.pitch.setPosition(pitchBOut);
                clawOut = true;
            } else {
                robot.lilJarret.setPosition(clawClose);
                clawIsOpen = false;
                clawOut = false;
            }
            depositStartTime = System.currentTimeMillis();
            bPressed = true;
        } else if (!gamepad2.b) bPressed = false;
        depositTime = (System.currentTimeMillis() - depositStartTime) / 1000.0;
        if (clawOut) {
            if (depositTime > 0.5 && !depositAction) {
                v4bPos = v4bMUp;
                robot.pitch.setPosition(pitchFDown);
                if(firstRun) {specMode = true; firstRun = false;}
                clawOut = false;
                depositAction = true;
                scoringSpec = false;
            } else if (depositTime > 0.25 && !depositAction) {
                v4bPos = v4bBDown;
                robot.lilJarret.setPosition(clawOpen); // drop
                clawIsOpen = true;
                depositAction = specMode;
            } else if (depositTime < 0.25) depositAction = false;
        } else {
            if (depositTime > 0.75 && !depositAction) {
                scoringSpec = true;
                v4bPos = v4bFOut;
                robot.pitch.setPosition(pitchMUp);
                slideTarget = slideMin;
                depositAction = true;
            } else if (depositTime > 0.5 && !depositAction) {
                robot.roll.setPosition(claw180);
            } else if (depositTime > 0.25 && !depositAction) {
                v4bPos = v4bBDown;
                slideTarget = slideMin+200;
            } else if (depositTime < 0.25) depositAction = false;
        }
        specRetractTime = (System.currentTimeMillis() - specRetractStartTime) / 1000.0;
        if(specRetractTime > 0.5 && !specRetractAction){
            slideTarget = slideMin;
            scoringSpec = false;
            robot.lilJarret.setPosition(clawOpen);
            specRetractAction = true;
        } else if(specRetractTime < 0.5){
            specRetractAction = false;
        }

        // press 'x' to toggle claw (driver 2)
        if (gamepad2.x && !xPressed) {
            if (!clawIsOpen) {
                robot.lilJarret.setPosition(clawOpen);
                clawIsOpen = true;
            } else {
                robot.lilJarret.setPosition(clawClose);
                clawIsOpen = false;
            }
            xPressed = true;
        } else if (!gamepad2.x) xPressed = false;

        // press 'back' to toggle spec mode (driver 2)
        if (gamepad2.back && !back2Pressed) {
            specMode = !specMode;
            if(!specMode) scoringSpec = false;
            back2Pressed = true;
        } else if (!gamepad2.back) back2Pressed = false;

        // press 'x', 'y', 'a', and 'b' for roll control (driver 1)
        if (extended && gamepad1.y && !y1Pressed) {
            robot.roll.setPosition(claw0);
            y1Pressed = true;
        } else if (!gamepad1.y) y1Pressed = false;
        if (extended && gamepad1.a && !a1Pressed) {
            robot.roll.setPosition(claw90);
            a1Pressed = true;
        } else if (!gamepad1.a) a1Pressed = false;
        if (extended && gamepad1.x && !x1Pressed) {
            robot.roll.setPosition(claw45);
            x1Pressed = true;
        } else if (!gamepad1.x) x1Pressed = false;
        if (extended && gamepad1.b && !b1Pressed) {
            robot.roll.setPosition(claw45_2);
            b1Pressed = true;
        } else if (!gamepad1.b) b1Pressed = false;

        //hanging (driver 1)
        if (gamepad1.back && !backPressed) {
            if (!hanging) {
                robot.hangerL.setTargetPosition(5700);
                robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerL.setPower(1);
                robot.hangerR.setTargetPosition(5700);
                robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerR.setPower(1);
                hanging = true;
            } else {
                robot.hangerL.setTargetPosition(-1000);
                robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerL.setPower(1);
                robot.hangerR.setTargetPosition(-5000);
                robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerR.setPower(1);
                hanging = false;
            }
            backPressed = true;
        } else if (!gamepad1.back) backPressed = false;

        // telemetry
        telemetry.addData("hang pos R, L", robot.hangerR.getCurrentPosition() + ", " + robot.hangerL.getCurrentPosition());
        telemetry.addLine("neck is " + (neckUp ? "UP" : "DOWN") + "!");
        telemetry.addLine("claw is " + (clawIsOpen ? "OPEN" : "CLOSED") + " and " + (clawOut ? "OUT" : "IN"));
        telemetry.addLine("robot is scoring " + (specMode ? "SPECIMEN" : "SAMPLE") + "!");
        telemetry.addData("depositTimer", depositTime);
        telemetry.update();
    }

    public void railExtend() {
        if (scoringSpec) {
            v4bPos = v4bFOut;
            robot.pitch.setPosition(pitchMUp);
            robot.roll.setPosition(claw180);
        } else {
            v4bPos = v4bFUp;
            robot.lilJarret.setPosition(clawOpen);
            clawIsOpen = true;
            robot.pitch.setPosition(pitchFDown);
            robot.roll.setPosition(claw0);
        }
        extendPosR = railRMax;
        extendPosL = railLMax;
    }

    public void railRetract() {
        robot.roll.setPosition(claw0);
        extendPosR = railRMin;
        extendPosL = railLMin;
        if (scoringSpec) {
            robot.lilJarret.setPosition(clawOpen);
            specRetractStartTime = System.currentTimeMillis();
            clawIsOpen = true;
        } else {
            v4bPos = v4bMUp;
            robot.pitch.setPosition(pitchFDown);
        }
    }
}