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
import static pandaPathing.robot.RobotConstants.slideMaxSpecTele;
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBDown;
import static pandaPathing.robot.RobotConstants.v4bMUp;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFOut;
import static pandaPathing.robot.RobotConstants.v4bFUp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.robot.Hardware;
import pandaPathing.util.Input;

@Config
@TeleOp(name = "BIG Tele", group = "#tellelelellel")
public class BIGTele extends OpMode {
    private Hardware robot;
    private Follower follower;

    boolean dUpPressed, dDownPressed, dLeftPressed, dLeft1Pressed, yPressed, y1Pressed, aPressed, a1Pressed, rBumpPressed, lBumpPressed, backPressed, back2Pressed, xPressed, x1Pressed, bPressed, b1Pressed,
            clawIsOpen = false, extended = false, neckUp = true, slowMode = false, hanging = false, slidesUp = false, slidesDown = false, clawOut = false,
            depositAction = true, grabAction = false, specRetractAction = true, specScoreRetryAction = true, retractAction = true,
            scoringSpec = false, specMode = false, autoDriveRunning = true;
    double slideTarget = slideMin, extendPosR = railRMin, extendPosL = railLMin, v4bPos = v4bMUp,
            depositTime, depositStartTime, grabStartTime, grabTime, specRetractStartTime, specRetractTime, specScoreRetryStartTime, specScoreRetryTime, retractStartTime, retractTime,
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
        robot.railL.setPosition(railLMin);
        robot.railR.setPosition(railRMin);
    }

    public void loop() {
        follower.setTeleOpMovementVectors(
                slideSpeed * driveSpeed * -gamepad1.left_stick_y,
                2 * slideSpeed * driveSpeed * (gamepad1.left_trigger > 0 ? gamepad1.left_trigger : (gamepad1.right_trigger > 0 ? -gamepad1.right_trigger : 0)),
                slideSpeed * driveSpeed * -gamepad1.right_stick_x,
                true);

        // auto drive code
        if(!follower.isBusy() && gamepad1.dpad_left && !dLeft1Pressed){
            Path toBucket = new Path(new BezierLine(
                    new Point(follower.getPose().getX(), follower.getPose().getY()),
                    new Point(10, 16)
            ));
            toBucket.setConstantHeadingInterpolation(Math.toRadians(-45));
            follower.followPath(toBucket);
            autoDriveRunning = true;
            dLeft1Pressed = true;
        } else if(!gamepad1.dpad_left) dLeft1Pressed = false;

        if(autoDriveRunning && !follower.isBusy()){
            follower.startTeleopDrive();
            autoDriveRunning = false;
        }
        follower.update();

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // automatic speed control
        if (extended || (clawOut && specMode))
            driveSpeed = 0.2;
        else driveSpeed = mult;

        slideSpeed = Math.sqrt((robot.rightSlides.getCurrentPosition() - 1510.41666) / -1510.41666);
        if (scoringSpec) slideSpeed *= 1.2;

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
            if (scoringSpec && extended) {
                slideTarget = slideMaxSpecTele - 100;
                specScoreRetryStartTime = System.currentTimeMillis();
            } else {
                if (scoringSpec) scoringSpec = false;
                slideTarget = slideMin;
            }
            dDownPressed = true;
        } else if (!gamepad2.dpad_down) dDownPressed = false;
        if (gamepad2.dpad_left && !dLeftPressed) {
            slideTarget = 900; // or slides down all the way
            dLeftPressed = true;
        } else if (!gamepad2.dpad_left) dLeftPressed = false;
        // automatic deposit setup when slides are up
        if (robot.rightSlides.getCurrentPosition() >= slideMax - 20 && !slidesUp){
            robot.roll.setPosition(claw90);
            slidesUp = true;
        } else if (robot.rightSlides.getCurrentPosition() >= slideMax - 100 && !slidesUp) {
            robot.pitch.setPosition(pitchBOut); // set claw above basket
            v4bPos = v4bBDown;
        } else if (robot.rightSlides.getCurrentPosition() <= slideMax - 100) slidesUp = false;
        if (slideTarget <= slideMin)
            slideTarget = slideMin; // reset slide position if out of bounds
        if (slideTarget >= slideMax) slideTarget = slideMax;

        specScoreRetryTime = (System.currentTimeMillis() - specScoreRetryStartTime) / 1000.0;
        if(specScoreRetryTime > 0.5 && !specScoreRetryAction){
            slideTarget = slideMaxSpecTele;
            specScoreRetryAction = true;
        }
        else if (specScoreRetryTime > 0.25 && !specScoreRetryAction){
            extendPosR = railRMin;
            extendPosL = railLMin;
        } else if (specScoreRetryTime < 0.25)
            specScoreRetryAction = false;

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
        retractTime = (System.currentTimeMillis() - retractStartTime) / 1000.0;
        if(retractTime > 0.5 && !retractAction){
            retractAction = true;
            v4bPos = v4bMUp;
        } else if(retractTime > 0.25 && !retractAction){
            extendPosR = railRMin;
            extendPosL = railLMin;
        } else if(retractTime < 0.25) retractAction = false;

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
                if(!slidesUp) robot.roll.setPosition(claw0);
                robot.pitch.setPosition(pitchBOut);
                clawOut = true;
            } else if(scoringSpec){
                robot.lilJarret.setPosition(clawClose);
                clawIsOpen = false;
                clawOut = false;
            }
            depositStartTime = System.currentTimeMillis();
            bPressed = true;
        } else if (!gamepad2.b) bPressed = false;
        depositTime = (System.currentTimeMillis() - depositStartTime) / 1000.0;
        if (clawOut) {
            if (depositTime > 0.4 && !depositAction) {
                v4bPos = v4bMUp;
                robot.pitch.setPosition(pitchFDown);
                clawOut = false;
                depositAction = true;
                scoringSpec = false;
            } else if (depositTime > 0.3 && !depositAction) {
                v4bPos = v4bBDown;
                robot.lilJarret.setPosition(clawOpen); // drop
                clawIsOpen = true;
                depositAction = specMode;
                scoringSpec = specMode;
            } else if (depositTime < 0.3) depositAction = false;
        } else {
            if (depositTime > 0.75 && !depositAction) {
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
        if(specRetractTime > 0.7 && !specRetractAction){
            slideTarget = slideMin;
            v4bPos = v4bMUp;
            scoringSpec = false;
            specRetractAction = true;
        } else if(specRetractTime > 0.2 && !specRetractAction){
            robot.roll.setPosition(claw0);
            extendPosR = railRMin;
            extendPosL = railLMin;
        } else if(specRetractTime < 0.2){
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
        if (gamepad1.y && !y1Pressed) {
            robot.roll.setPosition(claw0);
            y1Pressed = true;
        } else if (!gamepad1.y) y1Pressed = false;
        if (gamepad1.x && !x1Pressed) {
            robot.roll.setPosition(claw90);
            x1Pressed = true;
        } else if (!gamepad1.x) x1Pressed = false;
        if (gamepad1.a && !a1Pressed) {
            robot.roll.setPosition(claw45);
            a1Pressed = true;
        } else if (!gamepad1.a) a1Pressed = false;
        if (gamepad1.b && !b1Pressed) {
            robot.roll.setPosition(claw45_2);
            b1Pressed = true;
        } else if (!gamepad1.b) b1Pressed = false;

        //hanging (driver 1)
        if (gamepad1.dpad_up){
            robot.hangerL.setPower(1);
            robot.hangerR.setPower(1);
        } else if(gamepad1.dpad_down){
            robot.hangerL.setPower(-1);
            robot.hangerR.setPower(-1);
        } else {
            robot.hangerL.setPower(0);
            robot.hangerR.setPower(0);
        }

        // telemetry
        telemetry.addData("hang pos R, L", robot.hangerR.getCurrentPosition() + ", " + robot.hangerL.getCurrentPosition());
        telemetry.addLine("neck is " + (neckUp ? "UP" : "DOWN") + "!");
        telemetry.addLine("claw is " + (clawIsOpen ? "OPEN" : "CLOSED") + " and " + (clawOut ? "OUT" : "IN"));
        telemetry.addLine("robot is scoring " + (specMode ? "SPECIMEN" : "SAMPLE") + "!");
        telemetry.addData("slide pos", robot.rightSlides.getCurrentPosition());
        //telemetry.update();
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
        if (scoringSpec) {
            robot.lilJarret.setPosition(clawOpen);
            v4bPos = v4bFDown;
            specRetractStartTime = System.currentTimeMillis();
            clawIsOpen = true;
        } else {
            robot.roll.setPosition(claw0);
            v4bPos = v4bFOut;
            retractStartTime = System.currentTimeMillis();
            robot.pitch.setPosition(pitchMUp+0.15);
        }
    }
}