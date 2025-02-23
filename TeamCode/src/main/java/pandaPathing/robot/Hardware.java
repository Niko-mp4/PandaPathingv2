package pandaPathing.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import pandaPathing.util.PDFLController;
import java.util.List;

@Config
public class Hardware {
    private HardwareMap hardwareMap;

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, rightSlides, leftSlides, hangerL, hangerR;
    public Servo railL, railR, lilJarret, v4b, pitch, roll, yaw;
    public DigitalChannel touchSensor;
    public Limelight3A limelight;

    public PDFLController slideyController;

    public double slidePidPow(double target) {
        //slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        if (target > 500) {
            slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        } else {
            slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.homingConstant, RobotConstants.l);
        }
        int slidePos = rightSlides.getCurrentPosition();
        return slideyController.calculatePow(slidePos, target);
    }

    public Hardware(HardwareMap hardwareMap, boolean auto) {
        this.hardwareMap = hardwareMap;
        initializeMotors(auto);
        initializeServos();
        initializeSensors();
    }
    public Hardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        initializeMotors(false);
        initializeServos();
        initializeSensors();
    }

    private void initializeMotors(boolean auto) {
        leftFront = initMotor("em3", DcMotorEx.Direction.REVERSE);
        leftRear = initMotor("cm2", DcMotorEx.Direction.REVERSE);
        rightFront = initMotor("em2", DcMotorEx.Direction.FORWARD);
        rightRear = initMotor("cm3", DcMotorEx.Direction.FORWARD);
        rightSlides = initMotor("em0", DcMotorEx.Direction.REVERSE);
        if(auto) rightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides = initMotor("em1", DcMotorEx.Direction.FORWARD);
        hangerR = initMotor("cm1", DcMotorEx.Direction.FORWARD);
        hangerL = initMotor("cm0", DcMotorEx.Direction.FORWARD);
        slideyController = new PDFLController(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
    }

    private DcMotorEx initMotor(String name, DcMotorEx.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private void initializeServos() {
        railL = initServo("cs4");
        railR = initServo("cs5");
        v4b = initServo("es0");
        lilJarret = initServo("cs1");
        pitch = initServo("es5");
        roll = initServo("cs3");
        yaw = initServo("es3");
    }

    private Servo initServo(String name) {
        return (ServoImplEx) hardwareMap.get(Servo.class, name);
    }

    private void initializeSensors() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "cd0");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight3A");

    }

    public double calculateSlidePower(double target) {
        return slideyController.calculatePow(rightSlides.getCurrentPosition(), target);
    }
}
