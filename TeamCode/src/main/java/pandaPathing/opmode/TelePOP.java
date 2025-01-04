package pandaPathing.opmode;

import static pandaPathing.Hardware.claw0;
import static pandaPathing.Hardware.claw45;
import static pandaPathing.Hardware.claw45_2;
import static pandaPathing.Hardware.claw90;
import static pandaPathing.Hardware.clawClose;
import static pandaPathing.Hardware.clawOpen;
import static pandaPathing.Hardware.pitchBOut;
import static pandaPathing.Hardware.pitchFDown;
import static pandaPathing.Hardware.railLMax;
import static pandaPathing.Hardware.railLMin;
import static pandaPathing.Hardware.railRMax;
import static pandaPathing.Hardware.railRMin;
import static pandaPathing.Hardware.slideMax;
import static pandaPathing.Hardware.slideMin;
import static pandaPathing.Hardware.v4bBackDown;
import static pandaPathing.Hardware.v4bBackUp;
import static pandaPathing.Hardware.v4bOutDown;
import static pandaPathing.Hardware.v4bOutUp;
import static pandaPathing.Hardware.yaw0;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pandaPathing.Hardware;

@Config
@TeleOp(name = "I HATE NAVEEN SO MUCHHH", group = "Drive")
public class TelePOP extends OpMode {
    private Hardware robot;

    boolean dUpPressed, dDownPressed, yPressed, y1Pressed, aPressed, a1Pressed, rbumpPressed, lBumpPressed, backPressed, back2Pressed, xPressed, x1Pressed, bPressed, b1Pressed,
            clawIsOpen = false, extended = false, neckUp = true, slowMode = false, hanging = false, slidesUp = false, slidesDown = false,
            depositing = false, clawTimerRunning = false;
    double slideTarget = slideMin, extendPosR = railRMin, extendPosL = railLMin, v4bPos = v4bBackUp,
            depositTimer = 0, clawTimer = 0, scoringTimer = 0, grabbingTimer = 0,
            strafePow, driveSpeed, turnSpeed, slideSpeed, mult = 1;

    public void init() {
        robot = new Hardware(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.v4b.setPosition(v4bBackUp);
        robot.pitch.setPosition(pitchFDown);
        robot.lilJarret.setPosition(clawClose);
        robot.roll.setPosition(claw0);
        robot.yaw.setPosition(yaw0);
        robot.railL.setPosition(railLMin);
        robot.railR.setPosition(railRMin);
    }

    public void loop(){
    // Read gamepad input for movement
        double forward = -gamepad1.left_stick_y; // Forward/backward movement
        double strafe = 0;
        if (gamepad1.left_trigger != 0) {
            strafe = gamepad1.left_trigger;  // Strafe left when left trigger is pressed
        } else if (gamepad1.right_trigger != 0) {
            strafe = -gamepad1.right_trigger;  // Strafe right when right trigger is pressed
        }
        double turn = gamepad1.right_stick_x; // Turning (rotate)

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
        robot.leftFront.setPower(slideSpeed*driveSpeed*leftFrontPower);
        robot.rightFront.setPower(slideSpeed*driveSpeed*rightFrontPower);
        robot.leftRear.setPower(slideSpeed*driveSpeed*leftRearPower);
        robot.rightRear.setPower(slideSpeed*driveSpeed*rightRearPower);

        // automatic speed control
        if(robot.railL.getPosition() <= 0.5)
            driveSpeed = 0.25;
        else driveSpeed = mult;

        // press 'left bumper' to toggle speed slow/fast (driver 1)
        if(gamepad1.left_bumper && !lBumpPressed) {
            if (!slowMode) {
                mult = 0.33;
                slowMode = true;
            } else if (slowMode) {
                mult = 1;
                slowMode = false;
            }
            lBumpPressed = true;
        } else if(!gamepad1.left_bumper) lBumpPressed = false;

        // use 'left stick y' to move slides up/down (driver 2)
        robot.rightSlides.setPower(robot.slidePidPow(slideTarget));
        robot.leftSlides.setPower(robot.slidePidPow(slideTarget));
        if(gamepad2.left_stick_y != 0) {
            if (-gamepad2.left_stick_y > 0 && slideTarget < slideMax) // upper limit
                slideTarget += 10 * -gamepad2.left_stick_y;
            if (-gamepad2.left_stick_y < 0 && slideTarget > slideMin) // lower limit
                slideTarget += 10 * -gamepad2.left_stick_y;
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
        } // automatic deposit setup when slides are up
        if(robot.rightSlides.getCurrentPosition() >= slideMax-20 && !slidesUp) {
            robot.pitch.setPosition(pitchBOut); // set claw above basket
            v4bPos = v4bBackDown;
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
                v4bPos = v4bOutUp;
                robot.lilJarret.setPosition(clawOpen); clawIsOpen = true;
                extendPosR = railRMax;
                extendPosL = railLMax;
                extended = true;
            } else { // claw close & tuck when retract
                v4bPos = v4bBackUp;
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
        if(gamepad1.right_bumper && extended && !rbumpPressed){
            if(!neckUp){
                v4bPos = v4bOutUp;
                clawTimerRunning = true;
                neckUp = true;
            } else {
                v4bPos = v4bOutDown;
                clawTimerRunning = true;
                neckUp = false;
            }
            rbumpPressed = true;
        } else if(!gamepad1.right_bumper) rbumpPressed = false;

        // press 'b' to deposit sample (driver 2)
        if(gamepad2.b && !bPressed) {
            if (!extended) {
                v4bPos = v4bBackDown - 0.05;
                robot.pitch.setPosition(pitchBOut);
                depositing = true;
            }else { // claw close
                robot.lilJarret.setPosition(clawClose);
                depositing = false;
            }
            bPressed = true;
        } else if(!gamepad2.b) bPressed = false;

        // press 'back' to cancel (driver 2)
        if(gamepad2.back && !back2Pressed){
            back2Pressed = true;
        }else if(!gamepad2.back) back2Pressed = false;

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
        if(gamepad1.back && !backPressed){
            if(!hanging) {
                robot.hangerL.setTargetPosition(1000);
                robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerL.setPower(1);
                robot.hangerR.setTargetPosition(1000);
                robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerR.setPower(1);
                hanging = true;
            } else{
                robot.hangerL.setTargetPosition(-5500);
                robot.hangerL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerL.setPower(1);
                robot.hangerR.setTargetPosition(-5500);
                robot.hangerR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangerR.setPower(1);
                hanging = false;
            }
            backPressed = true;
        } else if (!gamepad1.back) backPressed = false;

        if(slideTarget > 250 || slideTarget > 250)
            slideSpeed = 0.3;
        else slideSpeed = 1;

        // sequences
        /**Claw open/close timer:
         * claw open/close at 20
         */
        if(clawTimerRunning) clawTimer ++;
        if (clawTimer >= 8){
            robot.lilJarret.setPosition(neckUp ? clawOpen : clawClose);
            clawIsOpen = neckUp;
            clawTimerRunning = false; clawTimer = 0;
        }

        /**Claw deposit sequence:
         * claw open at 10
         * claw close & tuck at 20
         */
        if (depositing) depositTimer++;
        if(depositTimer >= 2 && !bPressed) { // make claw drop after return
            if (depositTimer >= 10) {
                v4bPos = v4bBackUp;
                robot.pitch.setPosition(pitchFDown);
                robot.lilJarret.setPosition(clawClose); // go back in
                clawIsOpen = false;
            } else if(depositTimer >= 5){
                v4bPos = v4bBackDown;
                robot.lilJarret.setPosition(clawOpen); // drop
            }
        }
        if(depositTimer >= 20){ // cancel sequence after 80 iterations
            depositing = false;
            depositTimer = 0;
        }

        // telemetry
        telemetry.addData("hang pos", robot.hangerR.getCurrentPosition() + ", " + robot.hangerL.getCurrentPosition());
        telemetry.addData("neck", neckUp ? "up" : "down");
        telemetry.addData("claw open", clawIsOpen ? "open" : "closed");
        telemetry.addData("Slide Pos", robot.leftSlides.getCurrentPosition());
        telemetry.addData("Left slide  posit", robot.leftSlides.getCurrentPosition());
        telemetry.addData("Right slide  posit", robot.rightSlides.getCurrentPosition());
        telemetry.addData("Target", slideTarget);
        telemetry.update();
    }
}