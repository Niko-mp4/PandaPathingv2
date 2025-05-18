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
    public Servo railL, railR, claw, v4b, pitch, roll, yaw;
    public DigitalChannel touchSensor;
    public Limelight3A limelight;

    public PDFLController slideyController;

    public double slidePidPow(double target) {
        if (target > 800) slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        else if(target > 100) slideyController.updatePDFLConstants(RobotConstants.p1, RobotConstants.d1, RobotConstants.f1, RobotConstants.l1);
        else slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, 0, RobotConstants.l);

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

    public DcMotorEx initMotor(String name, DcMotorEx.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    private void initializeServos() {
        railL = initServo("sh3");
        railR = initServo("sh2");
        v4b = initServo("sh0");
        claw = initServo("es1");
        pitch = initServo("cs0");
        roll = initServo("es3");
        yaw = initServo("cs3");
    }

    private Servo initServo(String name) {
        return (ServoImplEx) hardwareMap.get(Servo.class, name);
    }

    private void initializeSensors() {
        touchSensor = hardwareMap.get(DigitalChannel.class, "cd0");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        //limelight = hardwareMap.get(Limelight3A.class, "Limelight3A");

    }

    public double calculateSlidePower(double target) {
        return slideyController.calculatePow(rightSlides.getCurrentPosition(), target);
    }
}
