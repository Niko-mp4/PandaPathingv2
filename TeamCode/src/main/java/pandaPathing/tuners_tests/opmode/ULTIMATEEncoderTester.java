package pandaPathing.tuners_tests.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import pandaPathing.util.Input;

@Config
@TeleOp(name = "EncoderTester")

public class ULTIMATEEncoderTester extends LinearOpMode {

    public String[] servoNames = {
            "cs0", "cs1", "cs2", "cs3", "cs4", "cs5", "cs6",
            "es0", "es1", "es2", "es3", "es4", "es5", "es6"
    };
    public String[] motorNames = {
            "cm0", "cm1", "cm2", "cm3",
            "em0", "em1", "em2", "em3"
    };
    public int index = 0;
    boolean reversed = false;
    double servoPos = 0;
    String tester = "s";

    Gamepad newgamey = new Gamepad();

    Input
        lTrigger = new Input(newgamey.left_trigger),
        rTrigger = new Input(newgamey.right_trigger),
        a = new Input(newgamey.a),
        b = new Input(newgamey.a),
        back = new Input(newgamey.back);

    public void runOpMode() {
        gamepad1 = newgamey;
        while(opModeInInit()) {
            lTrigger.run(() -> index-=index!=0?1:tester.equals("s")?14:8);
            rTrigger.run(() -> index+=index!=(tester.equals("s")?14:8)?1:0);
            a.run(() -> {
                if (tester.equals("s")) tester = "m";
                else tester = "s";
            });

            telemetry.addLine(tester.equals("s") ?
                    "Servo " + servoNames[index] :
                    "Motor " + motorNames[index]);
            telemetry.update();
        }

        waitForStart();

        if(tester.equals("s")) servoTest();
        else motorTest();
    }

    public void motorTest(){
        DcMotorEx test1;
        test1 = hardwareMap.get(DcMotorEx.class, motorNames[index]);
        test1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        test1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive()) {
            if (gamepad2.dpad_up) test1.setPower(1);
            else if (gamepad2.dpad_down) test1.setPower(-1);
            else if (gamepad2.y) test1.setPower(0.75);
            else if (gamepad2.x) test1.setPower(-0.75);
            else if (gamepad2.b) test1.setPower(0.5);
            else if (gamepad2.a) test1.setPower(-0.5);
            else test1.setPower(0);

            back.run(() -> reversed = !reversed);
            test1.setDirection(reversed ? DcMotorEx.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            telemetry.addLine("Dpad up/down for 1,\nY/X for 0.75,\nB/A for 0.5");
            telemetry.addLine("Testing " + motorNames[index]);
            telemetry.addLine("Motor is " + (reversed ? "" : "NOT") + " reversed!");
            telemetry.addLine("Position " + test1.getCurrentPosition());
            telemetry.addLine("Amp Draw " + test1.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("Power " + test1.getPower());
            telemetry.update();
        }
    }

    public void servoTest(){
        Servo test;
        test = (ServoImplEx)hardwareMap.get(Servo.class, servoNames[index]);

        while(opModeIsActive()) {
            a.run(() -> servoPos -= 0.01);
            b.run(() -> servoPos += 0.01);
            test.setPosition(servoPos);

            telemetry.addLine("Testing " + servoNames[index]);
            telemetry.addLine("Position " + test.getPosition());
            telemetry.addLine("Target " + servoPos);
            telemetry.update();
        }
    }
}
