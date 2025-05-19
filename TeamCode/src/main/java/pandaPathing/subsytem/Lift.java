package pandaPathing.subsytem;

import static pandaPathing.robot.RobotConstants.slideHighBasket;
import static pandaPathing.robot.RobotConstants.slideHighChamber;
import static pandaPathing.robot.RobotConstants.slidePark;
import static pandaPathing.robot.RobotConstants.slideScoreHighBasket;
import static pandaPathing.robot.RobotConstants.slideScoreLowBasket;
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

    public enum LiftState{
        HIGH_BASKET, LOW_BASKET, BOTTOM, HIGH_CHAMBER, PARK;
    }
    public static Lift.LiftState liftState;

    private Telemetry telemetry;

    public CachedMotor rightSlides, leftSlides;
    public PDFLController slideController;

    public int target;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightSlides = new CachedMotor(hardwareMap.get(DcMotor.class, "em0"));
        leftSlides = new CachedMotor(hardwareMap.get(DcMotor.class, "em1"));

        rightSlides.setDirection(DcMotor.Direction.REVERSE);
        leftSlides.setDirection(DcMotor.Direction.FORWARD);

        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideController = new PDFLController(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
    }
    public void init() { setLiftState(LiftState.BOTTOM); }

    // State functions
    public void setLiftState(Lift.LiftState liftState){
        switch(liftState) {
            case HIGH_BASKET:
                setTarget(slideScoreHighBasket);
            case LOW_BASKET:
                setTarget(slideScoreLowBasket);
            case HIGH_CHAMBER:
                setTarget(slideHighChamber);
            case BOTTOM:
                setTarget(slideZero);
            case PARK:
                setTarget(slidePark);
        }
        Lift.liftState = liftState;
    }
    public boolean is(LiftState state) { return liftState == state; }

    public void update() {
        if (target >= 800) slideController.updatePDFLConstants(RobotConstants.p, RobotConstants.d, RobotConstants.f, RobotConstants.l);
        else slideController.updatePDFLConstants(RobotConstants.p1, RobotConstants.d1, RobotConstants.f1, RobotConstants.l1);

        double power = slideController.calculatePow(getPos(), target);
        rightSlides.setPower(power);
        leftSlides.setPower(power);
    }

    public void setTarget(int b) { target = b; }

    public boolean atTargetWithin(int error){
        return Math.abs(getPos() - target) < error;
    }

    public int getPos() { return rightSlides.getCurrentPosition(); }

    private void telemetry() {
        telemetry.addData("Lift Pos", getPos());
        telemetry.addData("Lift Target", target);
    }

    @Override
    public void periodic() {
        update();
        telemetry();
    }
}
