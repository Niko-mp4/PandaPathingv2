package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw45;
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
import static pandaPathing.robot.RobotConstants.slideMin;
import static pandaPathing.robot.RobotConstants.v4bBUp;
import static pandaPathing.robot.RobotConstants.v4bMUp;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFUp;
import static pandaPathing.robot.RobotConstants.yaw0;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pandaPathing.robot.Hardware;

@Config
@TeleOp(name = "I love Naveen so so so so so so so so so so so so so so so so so so so so much!!!", group = "Drive")
public class TelePOP extends OpMode {
    private Hardware robot;

    boolean dUpPressed, dDownPressed, dLeftPressed, yPressed, y1Pressed, aPressed, a1Pressed, rBumpPressed, lBumpPressed, backPressed, back2Pressed, xPressed, x1Pressed, bPressed, b1Pressed,
            clawIsOpen = false, extended = false, neckUp = true, slowMode = false, hanging = false, slidesUp = false, slidesDown = false,
            depositAction = false, grabAction = false;
    double slideTarget = slideMin, extendPosR = railRMin, extendPosL = railLMin, v4bPos = v4bMUp,
            depositTime, depositStartTime, grabStartTime, grabTime,
            driveSpeed, slideSpeed, mult = 1;

    public void init() {
        robot = new Hardware(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.v4b.setPosition(v4bMUp);
        robot.pitch.setPosition(pitchFDown);
        robot.lilJarret.setPosition(clawClose);
        robot.roll.setPosition(claw0);
        robot.yaw.setPosition(yaw0);
        robot.railL.setPosition(railLMin);
        robot.railR.setPosition(railRMin);
    }

    public void loop(){
        // Read gamepad input for movement
        double forward = slideSpeed*driveSpeed*-gamepad1.left_stick_y; // Forward/backward movement
        double strafe = 2*slideSpeed*driveSpeed*(gamepad1.left_trigger > 0 ? -gamepad1.left_trigger : (gamepad1.right_trigger > 0 ? gamepad1.right_trigger : 0));
        double turn = slideSpeed*driveSpeed*gamepad1.right_stick_x; // Turning (rotate)

        // Calculate power for each motor based on input
        double leftFrontPower = forward + strafe + turn;    // Add all components for front-left motor
        double rightFrontPower = forward - strafe - turn;  // Add all components for front-right motor
        double leftRearPower = forward - strafe + turn;    // Add all components for rear-left motor
        double rightRearPower = forward + strafe - turn;   // Add all components for rear-right motor

        // Normalize the motor powers to ensure they are between -1 and 1
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftRearPower /= maxPower;
            rightRearPower /= maxPower;
        }

        // Set motor powers directly
        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightRear.setPower(rightRearPower);

        // automatic speed control
        if(robot.railL.getPosition() <= 0.5)
            driveSpeed = 0.25;
        else driveSpeed = mult;

        slideSpeed = Math.sqrt((robot.rightSlides.getCurrentPosition()-1510.41666)/-1510.41666);

        // press 'left bumper' to toggle speed slow/fast (driver 1)
        if(gamepad1.left_bumper && !lBumpPressed) {
            if (!slowMode) {
                mult = 0.33;
                slowMode = true;
            } else {
                mult = 1;
                slowMode = false;
            }
            lBumpPressed = true;
        } else if(!gamepad1.left_bumper) lBumpPressed = false;

        robot.rightSlides.setPower(robot.slidePidPow(slideTarget));
        robot.leftSlides.setPower(robot.slidePidPow(slideTarget));

        //use 'left stick y' to move slides up/down (driver 2)
        if(gamepad2.left_stick_y != 0) {
            //if (-gamepad2.left_stick_y > 0 && slideTarget < slideMax) // upper limit
            //slideTarget += 10 * -gamepad2.left_stick_y;
            //if (-gamepad2.left_stick_y < 0 && slideTarget > slideMin) // lower limit
            //slideTarget += 10 * -gamepad2.left_stick_y;
        }

        // press 'dpad up/down' to set slide target to one of 4 positions (driver 2)
        else {
            if (gamepad2.dpad_up && !dUpPressed) {
                slideTarget = slideMax; // top bar for spec. or top basket
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
        } // automatic deposit setup when slides are up
        if(robot.rightSlides.getCurrentPosition() >= slideMax-20 && !slidesUp) {
            robot.pitch.setPosition(pitchBOut); // set claw above basket
            v4bPos = v4bBUp+0.05;
            slidesUp = true;
        } else if(robot.rightSlides.getCurrentPosition() <= slideMax - 100) slidesUp = false;
        if(slideTarget <= slideMin) slideTarget = slideMin; // reset slide position if out of bounds
        if(slideTarget >= slideMax) slideTarget = slideMax;
        //if(!robot.touchSensor.getState() && !slidesDown){
        // automatically reset slide zero point when touch sensor is triggered
        //robot.rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slidesDown = true;
        //} else if(robot.touchSensor.getState()) slidesDown = false;

        // press 'a' to toggle rail extension in/out (driver 2)
        robot.railR.setPosition(extendPosR);
        robot.railL.setPosition(extendPosL);
        if (gamepad2.a && !aPressed) {
            if (!extended) { // claw open & out when extend
                robot.pitch.setPosition(pitchFDown);
                robot.roll.setPosition(claw0);
                v4bPos = v4bFUp;
                robot.lilJarret.setPosition(clawOpen); clawIsOpen = true;
                extendPosR = railRMax;
                extendPosL = railLMax;
                extended = true;
            } else { // claw close & tuck when retract
                v4bPos = v4bMUp;
                robot.roll.setPosition(claw0);
                robot.lilJarret.setPosition(clawClose); clawIsOpen = false;
                robot.pitch.setPosition(pitchFDown);
                extendPosR = railRMin;
                extendPosL = railLMin;
                extended = false;
            }
            aPressed = true;
        } else if (!gamepad2.a) aPressed = false;
        robot.railR.resetDeviceConfigurationForOpMode();
        robot.railL.resetDeviceConfigurationForOpMode();

        // press 'right bumper' to toggle v4b grabbing up/down (driver 1)
        robot.v4b.setPosition(v4bPos);
        if(gamepad1.right_bumper && extended && !rBumpPressed){
            if(!neckUp){
                v4bPos = v4bFUp;
                grabStartTime = System.currentTimeMillis();
                neckUp = true;
            } else {
                v4bPos = v4bFDown;
                grabStartTime = System.currentTimeMillis();
                neckUp = false;
            }
            rBumpPressed = true;
        } else if(!gamepad1.right_bumper) rBumpPressed = false;
        grabTime = (System.currentTimeMillis() - grabStartTime) / 1000.0;
        if (grabTime > 0.25 && !grabAction){
            robot.lilJarret.setPosition(neckUp ? clawOpen : clawClose);
            clawIsOpen = neckUp;
            grabAction = true;
        } else if (grabTime < 0.25) grabAction = false;

        // press 'b' to deposit sample (driver 2)
        if(gamepad2.b && !bPressed && !extended) {
            v4bPos = v4bBUp;
            robot.pitch.setPosition(pitchBOut);
            depositStartTime = System.currentTimeMillis();
            bPressed = true;
        } else if(!gamepad2.b) bPressed = false;
        depositTime = (System.currentTimeMillis() - depositStartTime) / 1000.0;
        if(depositTime > 0.5 && !depositAction) { // make claw drop after return
            v4bPos = v4bMUp;
            robot.pitch.setPosition(pitchFDown);
            robot.lilJarret.setPosition(clawClose); // go back in
            clawIsOpen = false;
            depositAction = true;
        } else if(depositTime > 0.25 && !depositAction){
            v4bPos = v4bBUp;
            robot.lilJarret.setPosition(clawOpen); // drop
            clawIsOpen = true;
        } else if(depositTime < 0.25) depositAction = false;

        // press 'x' to toggle claw (driver 2)
        if(gamepad2.x && !xPressed) {
            if (!clawIsOpen) {
                robot.lilJarret.setPosition(clawOpen);
                clawIsOpen = true;
            } else {
                robot.lilJarret.setPosition(clawClose);
                clawIsOpen = false;
            }
            xPressed = true;
        } else if(!gamepad2.x) xPressed = false;

        // press 'x', 'y', 'a', and 'b' for roll control (driver 1)
        if(extended && gamepad1.y && !y1Pressed){
            robot.roll.setPosition(claw0);
            y1Pressed = true;
        } else if(!gamepad1.y) y1Pressed = false;
        if(extended && gamepad1.a && !a1Pressed){
            robot.roll.setPosition(claw90);
            a1Pressed = true;
        } else if(!gamepad1.a) a1Pressed = false;
        if(extended && gamepad1.x && !x1Pressed){
            robot.roll.setPosition(claw45);
            x1Pressed = true;
        } else if(!gamepad1.x) x1Pressed = false;
        if(extended && gamepad1.b && !b1Pressed){
            robot.roll.setPosition(claw45_2);
            b1Pressed = true;
        } else if(!gamepad1.b) b1Pressed = false;

        //hanging (driver 1)
        if((gamepad2.back || gamepad1.back) && !backPressed){
            if(!hanging) {
                robot.hangerL.setTargetPosition(5700);
                robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerL.setPower(1);
                robot.hangerR.setTargetPosition(5700);
                robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerR.setPower(1);
                hanging = true;
            } else{
                robot.hangerL.setTargetPosition(-1000);
                robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerL.setPower(1);
                robot.hangerR.setTargetPosition(-5000);
                robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerR.setPower(1);
                hanging = false;
            }
            backPressed = true;
        } else if (!(gamepad2.back || gamepad1.back)) backPressed = false;

        // telemetry
        telemetry.addData("hang pos", robot.hangerR.getCurrentPosition() + ", " + robot.hangerL.getCurrentPosition());
        telemetry.addData("neck", neckUp ? "up" : "down");
        telemetry.addData("claw open", clawIsOpen ? "open" : "closed");
        telemetry.addData("Slide Pos", robot.leftSlides.getCurrentPosition());
        telemetry.addData("Left slide  posit", robot.leftSlides.getCurrentPosition());
        telemetry.addData("Right slide  posit", robot.rightSlides.getCurrentPosition());
        telemetry.addData("Target", slideTarget);
        telemetry.addData("depositTimer", depositTime);
        telemetry.addData("pitch Position", robot.pitch.getPosition());
        telemetry.addData("v4b Position", robot.v4b.getPosition());
        telemetry.update();
    }
}