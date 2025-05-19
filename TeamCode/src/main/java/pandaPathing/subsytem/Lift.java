package pandaPathing.subsytem;

import static pandaPathing.robot.RobotConstants.slideHighBasket;
import static pandaPathing.robot.RobotConstants.slideHighChamber;
import static pandaPathing.robot.RobotConstants.slidePark;
import static pandaPathing.robot.RobotConstants.slideScoreHighBasket;
import static pandaPathing.robot.RobotConstants.slideZero;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pandaPathing.robot.RobotConstants;
import pandaPathing.util.CachedMotor;
import pandaPathing.util.PDFLController;

public class Lift extends SubsystemBase {

    private Telemetry telemetry;

    public CachedMotor rightSlides, leftSlides;
    public PDFLController slideyController;

    public int target;
    public int pos;


    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightSlides = new CachedMotor(hardwareMap.get(DcMotor.class, "em0"));
        leftSlides = new CachedMotor(hardwareMap.get(DcMotor.class, "em1"));

        rightSlides.setDirection(DcMotor.Direction.REVERSE);
        leftSlides.setDirection(DcMotor.Direction.FORWARD);

        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideyController = new PDFLController(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
    }

    public void update() {
        if (target >= 800) {
            slideyController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        }

        else {
            slideyController.updatePDFLConstants(RobotConstants.p1, RobotConstants.d1, RobotConstants.f1, RobotConstants.l1);
        }

        int slidePos = getPos();

        slideyController.calculatePow(slidePos, target);
    }

    public void setTarget(int b) {
        target = b;
    }

    public int getPos() {
        pos = rightSlides.getCurrentPosition();
        return rightSlides.getCurrentPosition();
    }


    public void init() {
        slideyController = new PDFLController(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
    }


    public void start() {
        target = 0;
    }
    public void toZero() {
        setTarget(slideZero);
    }

    public void toHighBucket() {
        setTarget(slideHighBasket);
    }

    public void toScoreHighBucket() {
        setTarget(slideScoreHighBasket);
    }

    public void toChamber() {
        setTarget(slideHighChamber);
    }

    public void toPark() {
        setTarget(slidePark);
    }


    private void telemetry() {
        telemetry.addData("Lift Pos: ", getPos());
        telemetry.addData("Lift Target: ", target);
    }

    @Override
    public void periodic() {
        update();
        telemetry();
    }
}
