package pandaPathing.subsytem;

import static pandaPathing.robot.RobotConstants.claw0;
import static pandaPathing.robot.RobotConstants.claw180;
import static pandaPathing.robot.RobotConstants.claw45;
import static pandaPathing.robot.RobotConstants.claw45_2;
import static pandaPathing.robot.RobotConstants.claw90;
import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.clawOpen;
import static pandaPathing.robot.RobotConstants.pitchDeposit;
import static pandaPathing.robot.RobotConstants.pitchGrab;
import static pandaPathing.robot.RobotConstants.pitchInRobot;
import static pandaPathing.robot.RobotConstants.pitchSpecimen;
import static pandaPathing.robot.RobotConstants.v4bDeposit;
import static pandaPathing.robot.RobotConstants.v4bExtend;
import static pandaPathing.robot.RobotConstants.v4bGrab;
import static pandaPathing.robot.RobotConstants.v4bGrabSpec;
import static pandaPathing.robot.RobotConstants.v4bScoreSpec;
import static pandaPathing.robot.RobotConstants.v4bUp;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pandaPathing.robot.RobotConstants;
import pandaPathing.util.PDFLController;

public class Claw extends SubsystemBase {

    public enum GrabState {
        CLOSED, OPEN
    }

    public enum RollState {
        ZERO, ONE45, TWO45, NINTY, ONEEIGHTY
    }

    public enum PitchState {
        INROBOT, EXTEND, DEPOSIT, SPECIMEN
    }

    public enum V4BState {
        UPV4B, EXTENDV4B, GRABV4B, DEPOSITV4B, GRABSPECIMENV4B, SCORESPECIMENV4B
    }

    public Servo v4b, claw, pitch, roll;

    public static GrabState grabState;
    public static RollState rollState;
    public static PitchState pitchState;
    public static V4BState v4bState;
    private Telemetry telemetry;


    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        v4b = hardwareMap.get(Servo.class, "sh0");
        claw = hardwareMap.get(Servo.class, "es1");
        pitch = hardwareMap.get(Servo.class, "cs0");
        roll = hardwareMap.get(Servo.class, "es3");
        this.telemetry = telemetry;
    }

    public void setGrabState(GrabState grabState) {
        switch (grabState) {
            case CLOSED:
                claw.setPosition(clawClose);
                break;
            case OPEN:
                claw.setPosition(clawOpen);
                break;
        }
        Claw.grabState = grabState;
    }

    public void setRollState(RollState rollState) {
        switch (rollState) {
            case ZERO:
                roll.setPosition(claw0);
                break;
            case ONE45:
                roll.setPosition(claw45);
                break;
            case TWO45:
                roll.setPosition(claw45_2);
                break;
            case NINTY:
                roll.setPosition(claw90);
                break;
            case ONEEIGHTY:
                roll.setPosition(claw180);
                break;
        }
        Claw.rollState = rollState;
    }

    public void setPitchState(PitchState pitchState) {
        switch (pitchState) {
            case INROBOT:
                pitch.setPosition(pitchInRobot);
                break;
            case EXTEND:
                pitch.setPosition(pitchGrab);
                break;
            case DEPOSIT:
                pitch.setPosition(pitchDeposit);
                break;
            case SPECIMEN:
                pitch.setPosition(pitchSpecimen);
                break;
        }
        Claw.pitchState = pitchState;
    }

    public void setV4BState(V4BState v4bState) {
        switch (v4bState) {
            case UPV4B:
                v4b.setPosition(v4bUp);
                break;
            case EXTENDV4B:
                v4b.setPosition(v4bExtend);
                break;
            case GRABV4B:
                v4b.setPosition(v4bGrab);
                break;
            case DEPOSITV4B:
                v4b.setPosition(v4bDeposit);
                break;
            case GRABSPECIMENV4B:
                v4b.setPosition(v4bGrabSpec);
                break;
            case SCORESPECIMENV4B:
                v4b.setPosition(v4bScoreSpec);
                break;
        }
        Claw.v4bState = v4bState;
    }

    public void init() {
        setGrabState(GrabState.CLOSED);
        setRollState(RollState.ZERO);
        setPitchState(PitchState.INROBOT);
        setV4BState(V4BState.UPV4B);
    }


    public void start() {
        setGrabState(GrabState.CLOSED);
        setRollState(RollState.ZERO);
        setPitchState(PitchState.INROBOT);
        setV4BState(V4BState.UPV4B);
    }

    public void telemetry() {
        telemetry.addData("Claw Grab State: ", grabState);
        telemetry.addData("Claw Roll State: ", rollState);
        telemetry.addData("Claw Pitch State: ", pitchState);
        telemetry.addData("Claw V4B State: ", v4bState);
    }

    @Override
    public void periodic() {
        telemetry();
    }
}
