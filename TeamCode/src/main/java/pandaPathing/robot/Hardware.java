package pandaPathing.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import pandaPathing.util.PDFLController;
import java.util.Arrays;
import java.util.List;

@Config
public class Hardware {
    private HardwareMap hardwareMap;

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, rightSlides, leftSlides, hangerL, hangerR;
    public Servo railL, railR, lilJarret, v4b, pitch, roll, yaw;
    public DigitalChannel touchSensor;
    public List<DcMotorEx> motors;

    public PDFLController slideyController;

    public double slidePidPow(double target) {
        if (target > 500) {
            slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        } else {
            slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.homingConstant, RobotConstants.l);
        }
        int slidePos = rightSlides.getCurrentPosition();
        return slideyController.calculatePow(slidePos, target);
    }

    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initializeMotors();
        initializeServos();
        initializeSensors();
    }

    private void initializeMotors() {
        leftFront = initMotor("em3", DcMotorSimple.Direction.REVERSE);
        leftRear = initMotor("cm2", DcMotorSimple.Direction.REVERSE);
        rightFront = initMotor("em2", DcMotorSimple.Direction.FORWARD);
        rightRear = initMotor("cm3", DcMotorSimple.Direction.FORWARD);
        rightSlides = initMotor("em1", DcMotorSimple.Direction.REVERSE);
        leftSlides = initMotor("em0", DcMotorSimple.Direction.FORWARD);
        hangerL = initMotor("cm1", DcMotorSimple.Direction.FORWARD);
        hangerR = initMotor("cm0", DcMotorSimple.Direction.FORWARD);
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        slideyController = new PDFLController(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
    }

    private DcMotorEx initMotor(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private void initializeServos() {
        railL = initServo("es2");
        railR = initServo("es1");
        v4b = initServo("es0");
        lilJarret = initServo("cs0");
        pitch = initServo("es5");
        roll = initServo("cs2");
        yaw = initServo("es3");
    }

    private Servo initServo(String name) {
        return (ServoImplEx) hardwareMap.get(Servo.class, name);
    }

    private void initializeSensors() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "cd0");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public double calculateSlidePower(double target) {
        slideyController.updatePDFLConstants(
                target > 500 ? RobotConstants.p : RobotConstants.homingConstant, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        return slideyController.calculatePow(rightSlides.getCurrentPosition(), target);
    }
}
