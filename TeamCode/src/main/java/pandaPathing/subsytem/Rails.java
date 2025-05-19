package pandaPathing.subsytem;

import static pandaPathing.robot.RobotConstants.pitchDeposit;
import static pandaPathing.robot.RobotConstants.pitchGrab;
import static pandaPathing.robot.RobotConstants.pitchInRobot;
import static pandaPathing.robot.RobotConstants.pitchSpecimen;
import static pandaPathing.robot.RobotConstants.railLIn;
import static pandaPathing.robot.RobotConstants.railLOut;
import static pandaPathing.robot.RobotConstants.railRIn;
import static pandaPathing.robot.RobotConstants.railROut;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Rails extends SubsystemBase {

    public enum RailState {
        IN, OUT
    }

    private MultipleTelemetry telemetry;

    public Servo railL, railR;

    public static Rails.RailState railState;

    private double pos = 0;


    public Rails(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        railL = hardwareMap.get(Servo.class, "sh3");
        railR = hardwareMap.get(Servo.class, "sh2");

    }

    public void setRailState(Rails.RailState railState) {
        switch (railState) {
            case IN:
                railL.setPosition(railLIn);
                railR.setPosition(railRIn);
                break;
            case OUT:
                railL.setPosition(railLOut);
                railR.setPosition(railROut);
                break;
        }
        Rails.railState = railState;
    }

    public void init() {
        setRailState(RailState.IN);
    }

    public double getPos() {
        pos = railR.getPosition();
        return pos;
    }

    public RailState getState() {
        return railState;
    }


    public void telemetry() {
        telemetry.addData("Extend Pos: (1 = Out)", getPos());
    }

    @Override
    public void periodic() {
        telemetry();
    }
}
