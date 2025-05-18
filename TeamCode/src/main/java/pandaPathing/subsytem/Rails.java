package pandaPathing.subsytem;

import static pandaPathing.robot.RobotConstants.railLIn;
import static pandaPathing.robot.RobotConstants.railLOut;
import static pandaPathing.robot.RobotConstants.railRIn;
import static pandaPathing.robot.RobotConstants.railROut;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Rails {

    public enum ExtendState {
        IN, OUT
    }

    private MultipleTelemetry telemetry;

    public Servo railL, railR;

    private ExtendState state = ExtendState.IN;

    private double pos = 0;


    public Rails(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        railL = hardwareMap.get(Servo.class, "sh3");
        railR = hardwareMap.get(Servo.class, "sh2");

    }

    public void setTargetIn() {
        railL.setPosition(railLIn);
        railR.setPosition(railRIn);
    }
    public void setTargetOut() {
        railL.setPosition(railLOut);
        railR.setPosition(railROut);
    }

    public void toIn() {
        setTargetIn();
        state = ExtendState.IN;
    }
    public void toOut() {
        setTargetOut();
        state = ExtendState.OUT;
    }

    public double getPos() {
        pos = railR.getPosition();
        return pos;
    }

    public ExtendState getState() {
        return state;
    }


    public void telemetry() {
        telemetry.addData("Extend Pos: (1 = Out)", getPos());
    }

    public void periodic() {
        telemetry();
    }
}
