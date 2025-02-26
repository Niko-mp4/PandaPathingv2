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
    String[] names = servoNames;

    public void runOpMode() {
        Input.initInputs(gamepad1);
        while(opModeInInit()) {
            Input.a.toggle(() -> names = motorNames,
                           () -> names = servoNames);
            Input.lTrigger.press(() -> index = (index!=0)? index-1 : names.length-1);
            Input.rTrigger.press(() -> index = (index!=names.length-1)? index+1:0);

            telemetry.addLine(names == servoNames ? "Servo " : "Motor " + names[index]);
            telemetry.update();
        }
        waitForStart();
        if(names == servoNames) servoTest();
        else motorTest();
    }

    public void servoTest(){
        Servo servo;
        servo = (ServoImplEx)hardwareMap.get(Servo.class, servoNames[index]);

        while(opModeIsActive()) {
            Input.a.press(() -> servoPos -= 0.01);
            Input.b.press(() -> servoPos += 0.01);
            servo.setPosition(servoPos);

            telemetry.addLine("Testing " + servoNames[index]);
            telemetry.addLine("Position " + servo.getPosition());
            telemetry.addLine("Target " + servoPos);
            telemetry.update();
        }
    }

    public void motorTest(){
        DcMotorEx motor;
        motor = hardwareMap.get(DcMotorEx.class, motorNames[index]);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive()) {
            Input.dPadUp.hold(() -> motor.setPower(1), () ->
                    Input.dPadDown.hold(() -> motor.setPower(-1), () ->
                            Input.y.hold(() -> motor.setPower(0.75), () ->
                                    Input.x.hold(() -> motor.setPower(-0.75), () ->
                                            Input.b.hold(() -> motor.setPower(0.5), () ->
                                                    Input.a.hold(() -> motor.setPower(-0.5), () ->
                                                            motor.setPower(0)))))));

            if (Input.dPadUp.get()) motor.setPower(1);
            else if (gamepad1.dpad_down) motor.setPower(-1);
            else if (gamepad1.y) motor.setPower(0.75);
            else if (gamepad1.x) motor.setPower(-0.75);
            else if (gamepad1.b) motor.setPower(0.5);
            else if (gamepad1.a) motor.setPower(-0.5);
            else motor.setPower(0);

            Input.back.press(() -> reversed = !reversed);
            motor.setDirection(reversed ? DcMotorEx.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            telemetry.addLine("Dpad up/down for 1,\nY/X for 0.75,\nB/A for 0.5");
            telemetry.addLine("Testing " + motorNames[index]);
            telemetry.addLine("Motor is " + (reversed ? "" : "NOT") + " reversed!");
            telemetry.addLine("Position " + motor.getCurrentPosition());
            telemetry.addLine("Amp Draw " + motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("Power " + motor.getPower());
            telemetry.update();
        }
    }
}
