package pandaPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pandaPathing.robot.Hardware;

@Config
@TeleOp(name = "Slide Tester", group = "Test")
public class SlideTester extends LinearOpMode {

    private Hardware robot; // Instance of the Hardware class
    public static int slideTarget = 0;

    @Override
    public void runOpMode() {
        // Initialize the Hardware class
        robot = new Hardware(hardwareMap);

        waitForStart();
        motorTest();
    }

    public void motorTest() {
        // Set the initial slide target
        slideTarget = 0;

        while (opModeIsActive()) {
            // Get the slide power using the Hardware class's slidePidPow method
            double slidePower = robot.slidePidPow(slideTarget);

            // Set the power for the slides
            robot.rightSlides.setPower(slidePower);
            robot.leftSlides.setPower(slidePower);

            // Display telemetry data
            telemetry.addData("Right Slide Position", robot.rightSlides.getCurrentPosition());
            telemetry.addData("Left Slide Position", robot.leftSlides.getCurrentPosition());
            telemetry.addData("Slide Target", slideTarget);
            telemetry.addData("Slide Power Set", slidePower);
            telemetry.update();
        }
    }
}
