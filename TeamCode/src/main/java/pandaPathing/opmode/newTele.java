package pandaPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pandaPathing.commands.Deposit;
import pandaPathing.commands.RaiseSlidesHighBasket;
import pandaPathing.subsytem.Claw;
import pandaPathing.subsytem.Lift;
import pandaPathing.subsytem.Rails;
import pandaPathing.subsytem.Robot;
import pandaPathing.util.Input;

@Config
@TeleOp(name = "BIGGER Tele", group = "#tellelelellel")
public class newTele extends OpMode{
    private Robot robot;
    private Follower follower;

    private GamepadEx driver1;
    private GamepadEx driver2;
    public Button
            a, b, x, y,
            dPadU1, dPadDown, dPadLeft, dPadRight,
            lBump, rBump, lTrigger, rTrigger,
            back, lStickButton, rStickButton;
    public Button
            a2, b2, x2, y2,
            dPadUp2, dPadDown2, dPadLeft2, dPadRight2,
            lBump2, rBump2, lTrigger2, rTrigger2,
            back2, lStickButton2, rStickButton2;

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

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        b2 = new GamepadButton(driver2, GamepadKeys.Button.B);
        dPadU1 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP);
    }
    public void loop(){
        dPadU1.whenPressed(new RaiseSlidesHighBasket(lift, claw));
        b2.whenPressed(new Deposit(lift, claw));
    }
}
