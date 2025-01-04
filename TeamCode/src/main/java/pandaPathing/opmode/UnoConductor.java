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
import static pandaPathing.robot.RobotConstants.v4bBackDown;
import static pandaPathing.robot.RobotConstants.v4bBackUp;
import static pandaPathing.robot.RobotConstants.v4bOutDown;
import static pandaPathing.robot.RobotConstants.v4bOutUp;
import static pandaPathing.robot.RobotConstants.yaw0;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pandaPathing.robot.Hardware;

@Config
@TeleOp(name = "UnoConductor", group = "Tele")
public class UnoConductor extends OpMode {
    private Hardware robot;
    private Follower follower;

    boolean dUpPressed, dDownPressed, yPressed, y1Pressed, dpad_upPressed, dpad_up1Pressed, rbumpPressed, lBumpPressed, x1Pressed, dpad_downPressed, dpad_down1Pressed,
            clawIsOpen = false, extended = false, neckUp = true, slowMode = false,
            depositing = false, clawTimerRunning = false;
    double  extendPosR = railRMin, extendPosL = railLMin, v4bPos = v4bBackUp,
            depositTimer = 0, clawTimer = 0,
            strafePow, driveSpeed, mult = 1;
    double startTime, elapsedTime, timeIndex = 0;
    double[] elapsedTimes = new double[10];
    boolean timerRunning = false;
    double highScore = Double.MAX_VALUE;

    public void init() {
        robot = new Hardware(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
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
        // pedro pathing mecanum drive
        strafePow = gamepad1.left_trigger != 0 ? gamepad1.left_trigger : gamepad1.right_trigger != 0 ? -gamepad1.right_trigger : 0;
        follower.setTeleOpMovementVectors(driveSpeed*-gamepad1.left_stick_y, 0.8*driveSpeed*strafePow, driveSpeed*-gamepad1.right_stick_x);
        follower.update();

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

        // press 'a' to toggle rail extension in/out (driver 1)
        robot.railR.setPosition(extendPosR);
        robot.railL.setPosition(extendPosL);
        if (gamepad1.dpad_up && !dpad_upPressed) {
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
            dpad_upPressed = true;
        } else if (!gamepad1.dpad_up) dpad_upPressed = false;
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

        // press 'd-pad down' to deposit sample (driver 1) and track the timer
        if(gamepad1.dpad_down && !dpad_downPressed) {
            // Start the timer
            if (!timerRunning) {
                startTime = getRuntime();
                timerRunning = true;
            } else {
                // Stop the timer
                elapsedTime = getRuntime() - startTime;
                if (timeIndex < 10) {
                    elapsedTimes[(int) timeIndex] = elapsedTime;
                    timeIndex++;
                } else {
                    // If the array is full, overwrite the earliest time
                    for (int i = 0; i < 9; i++) {
                        elapsedTimes[i] = elapsedTimes[i + 1];
                    }
                    elapsedTimes[9] = elapsedTime;
                }

                // Update high score if the new time is better (lower)
                if (elapsedTime < highScore) {
                    highScore = elapsedTime;
                }

                timerRunning = false;
            }

            // Claw and deposit actions
            if (!extended) {
                v4bPos = v4bBackDown - 0.05;
                robot.pitch.setPosition(pitchBOut);
                depositing = true;
            } else { // claw close
                robot.lilJarret.setPosition(clawClose);
                depositing = false;
            }
            dpad_downPressed = true;
        } else if(!gamepad1.dpad_down) dpad_downPressed = false;

        // press 'x', 'y', 'a', and 'b' for roll control (driver 1)
        if(extended && gamepad1.y && !y1Pressed){
            robot.roll.setPosition(claw0);
            y1Pressed = true;
        } else if(!gamepad1.y) y1Pressed = false;
        if(extended && gamepad1.a && !dpad_up1Pressed){
            robot.roll.setPosition(claw90);
            dpad_up1Pressed = true;
        } else if(!gamepad1.a) dpad_up1Pressed = false;
        if(extended && gamepad1.x && !x1Pressed){
            robot.roll.setPosition(claw45);
            x1Pressed = true;
        } else if(!gamepad1.x) x1Pressed = false;
        if(extended && gamepad1.b && !dpad_down1Pressed){
            robot.roll.setPosition(claw45_2);
            dpad_down1Pressed = true;
        } else if(!gamepad1.b) dpad_down1Pressed = false;

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
        if(depositTimer >= 2 && !dpad_downPressed) { // make claw drop after return
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

        // telemetry: display all times and highlight the high score
        StringBuilder timesDisplay = new StringBuilder();
        for (int i = 0; i < timeIndex; i++) {
            timesDisplay.append("Time ").append(i + 1).append(": ").append(elapsedTimes[i]).append("s");
            // Check if it's the high score time
            if (elapsedTimes[i] == highScore) {
                timesDisplay.append(" (High Score!)");
            }
            timesDisplay.append("\n");
        }

        telemetry.addData("Recorded Times", timesDisplay.toString());
        telemetry.addData("Current High Score", highScore + "s");
        telemetry.update();
    }
}