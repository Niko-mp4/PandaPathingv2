package pandaPathing.tuners_tests.opmode;

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
@TeleOp(name = "EncoderTester")

public class EncoderTester extends LinearOpMode {
    boolean slidesDown = false, dUpPressed = false, dDownPressed = false;

    DcMotorEx test1, test2;
    public static int slideTarget = 0;
    public PDFLController slideyController;
    public static double p = 0, d = 0, f = 0.15, l = 0, homingConstant = 0;

    public void runOpMode() {
        slideyController = new PDFLController(p, d, f, l);
        waitForStart();
        servoTest();
        //motorTest();
    }

    public void motorTest(){
        test1 = hardwareMap.get(DcMotorEx.class, "cm1");
        test2 = hardwareMap.get(DcMotorEx.class, "cm0");
        test1.setDirection(DcMotorSimple.Direction.REVERSE);
        test1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        test2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        test1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        test2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideTarget = 0;
        while(opModeIsActive()) {
            int slidePos = test1.getCurrentPosition();

            /*if (gamepad2.dpad_up) {
                frontSlides.setPower(1);
                backSlides.setPower(1);
            } else if (gamepad2.dpad_down) {
                frontSlides.setPower(-1);
                backSlides.setPower(-1);
            } else if (gamepad2.y) {
                frontSlides.setPower(0.75);
                backSlides.setPower(0.75);
            } else if (gamepad2.x) {
                frontSlides.setPower(-0.75);
                backSlides.setPower(-0.75);
            } else if (gamepad2.b) {
                frontSlides.setPower(0.5);
                backSlides.setPower(0.5);
            } else if (gamepad2.a) {
                frontSlides.setPower(-0.5);
                backSlides.setPower(-0.5);
            } else {
                frontSlides.setPower(0);
                backSlides.setPower(0);
            }*/

            telemetry.addData("hangR  posit", test1.getCurrentPosition());
            telemetry.addData("hangL  posit", test2.getCurrentPosition());
            telemetry.addData("hangR ampss", test1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("hangL ampss", test2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slide  power read", test1.getPower());
            telemetry.update();
        }
    }

    public void servoTest(){
        Servo cs0;
        cs0 = (ServoImplEx)hardwareMap.get(Servo.class, "es0");

        double position0 = 0;
        boolean aPressed = false;

        while(opModeIsActive()) {
            cs0.setPosition(position0);
            if(gamepad2.a && !aPressed){
                position0 += 0.01;
                aPressed = true;
            }
            else if(!gamepad2.a) aPressed = false;
            telemetry.addData("es0", cs0.getPosition());
            telemetry.update();
        }
    }
}
