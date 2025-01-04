package pandaPathing;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.pedropathing.localization.Localizer;

import java.util.Arrays;
import java.util.List;

import pandaPathing.util.PDFLController;

/**
 * This is the Follower class. It handles the actual following of the paths and all the on-the-fly
 * calculations that are relevant for movement.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class Hardware {
    private HardwareMap hardwareMap;

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;
    public DcMotorEx rightSlides, leftSlides, hangerL, hangerR;
    public Servo railL, railR, lilJarret, v4b, pitch, roll, yaw;
    public DigitalChannel touchSensor;
    public List<DcMotorEx> motors;

    public final static double
            pitchFDown = 0.48, pitchFOut = 1, pitchBOut = 0.2, pitchBDown = 0.2,
            railRMax = 0.74, railLMax = 0.28, railRMin = 0.1, railLMin = 0.9,
            v4bOutUp = 0.31, v4bOutDown = 0.37, v4bBackUp = 0.16, v4bBackDown = 0.065,
            clawOpen = 0.817, clawClose = 0.338,
            claw45_2 = 0.788, claw0 = 0.515, claw45 = 0.320, claw90 = 0.005,
            yaw45_2 = 0.334, yaw0 = 0.589, yaw45 = 0.845;
    //0.583

    public final static int
            slideMax = 1450, slideMaxSpec = 1000, slideMin = 0;
    public PDFLController slideyController;
    public static double p = 0.035, d = 0.7, f = 0.15, l = 0, homingConstant = 0;


    /**
     * This creates a new Follower given a HardwareMap.
     *
     * @param hardwareMap HardwareMap required
     */
    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
    }

    /**
     * This creates a new Follower given a HardwareMap and a localizer.
     *
     * @param hardwareMap HardwareMap required
     * @param localizer   the localizer you wish to use
     */
    public Hardware(HardwareMap hardwareMap, Localizer localizer) {
        this.hardwareMap = hardwareMap;
        initialize(localizer);
    }

    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     */
    public void initialize() {

        rightFront = hardwareMap.get(DcMotorEx.class, "em2");
        leftFront = hardwareMap.get(DcMotorEx.class, "em3");
        rightRear = hardwareMap.get(DcMotorEx.class, "cm3");
        leftRear = hardwareMap.get(DcMotorEx.class, "cm2");
        rightSlides = hardwareMap.get(DcMotorEx.class, "em1");
        leftSlides = hardwareMap.get(DcMotorEx.class, "em0");
        hangerL = hardwareMap.get(DcMotorEx.class, "cm1");
        hangerR = hardwareMap.get(DcMotorEx.class, "cm0");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servo configs
        railL = (ServoImplEx) hardwareMap.get(Servo.class, "es2");
        railR = (ServoImplEx) hardwareMap.get(Servo.class, "es1");
        v4b = (ServoImplEx) hardwareMap.get(Servo.class, "es0");
        lilJarret = (ServoImplEx) hardwareMap.get(Servo.class, "cs0");
        pitch = (ServoImplEx) hardwareMap.get(Servo.class, "es5");
        roll = (ServoImplEx) hardwareMap.get(Servo.class, "cs2");
        yaw = (ServoImplEx) hardwareMap.get(Servo.class, "es3");

        // touch sensor
        touchSensor = hardwareMap.get(DigitalChannel.class, "cd0");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        //slides
        slideyController = new PDFLController(p, d, f, l);
    }

    public double slidePidPow(double target) {
        if (target > 500) {
            slideyController.updatePDFLConstants(p, d, f, l);
        } else {
            slideyController.updatePDFLConstants(p, d, homingConstant, l);
        }
        int slidePos = rightSlides.getCurrentPosition();
        double pid = slideyController.calculatePow(slidePos, target);
        return pid;
    }

    public void initialize(Localizer localizer) {

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

    }
}