package pandaPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pandaPathing.subsytem.Claw;
import pandaPathing.subsytem.Lift;
import pandaPathing.subsytem.Rails;
import pandaPathing.subsytem.Robot;

@Config
@TeleOp(name = "BIGGER Tele", group = "#tellelelellel")
public class newTele extends OpMode{
    private Robot robot;
    private Follower follower;

    // Subsystems
    public Lift lift;
    public Claw claw;
    public Rails rails;

    public void init(){
        lift = new Lift(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        rails = new Rails(hardwareMap, telemetry);
        claw.init();
        rails.init();
    }
    public void loop(){

    }
}
