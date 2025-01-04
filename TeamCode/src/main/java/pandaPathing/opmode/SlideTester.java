package pandaPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import pandaPathing.util.PDFLController;


@Config
@TeleOp(name = "Slide Tester")

public class SlideTester extends LinearOpMode {
    boolean slidesDown = false, dUpPressed = false, dDownPressed = false;

    DcMotorEx rightSlides, leftSlides;
    public static int slideTarget = 0;
    public PDFLController slideyController;
    public static double p = 0.025, d = 0.6, f = 0.15, l = 0, homingConstant = 0;

    public void runOpMode() {
        slideyController = new PDFLController(p, d, f, l);
        waitForStart();
        motorTest();
    }
    public double slidePidPow(double target){
        if(target > 500){
            slideyController.updatePDFLConstants(p, d, f, l);
        } else {
            slideyController.updatePDFLConstants(p, d, homingConstant, l);
        }
        int slidePos = leftSlides.getCurrentPosition();
        return slideyController.calculatePow(slidePos, target);
    }

    public void motorTest(){
        rightSlides = hardwareMap.get(DcMotorEx.class, "em0");
        leftSlides = hardwareMap.get(DcMotorEx.class, "em1");
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideTarget = 0;
        while(opModeIsActive()) {
            int slidePos = leftSlides.getCurrentPosition();

            rightSlides.setPower(slidePidPow(slideTarget));
            leftSlides.setPower(slidePidPow(slideTarget));

            telemetry.addData("Right slide  posit", rightSlides.getCurrentPosition());
            telemetry.addData("Left slide  posit", leftSlides.getCurrentPosition());
            telemetry.addData("slideR ampss", rightSlides.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slideL ampss", leftSlides.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slide  power set ", slidePidPow(slideTarget));
            telemetry.addData("slide  power read", rightSlides.getPower());
            telemetry.update();
        }
    }

    public void servoTest(){
        Servo cs0;
        cs0 = (ServoImplEx)hardwareMap.get(Servo.class, "cs1");

        double position0 = 0;
        boolean aPressed = false;

        while(opModeIsActive()) {
            cs0.setPosition(position0);
            if(gamepad2.a && !aPressed){
                position0 += 0.01;
                aPressed = true;
            }
            else if(!gamepad2.a) aPressed = false;
            telemetry.addData("cs0", cs0.getPosition());
            telemetry.update();
        }
    }
}
