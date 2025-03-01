package pandaPathing.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Input {
    private final boolean input;
    private boolean pressing;
    private boolean pressed;

    public Input(boolean input){
        this.input = input;
    }
    public Input(double input){
        this.input = input > 0;
    }

    public void press(Runnable body) {
        if(input && !pressing){
            body.run();
            pressing = true;
        }
        else if(!input) pressing = false;
    }
    public static void press(boolean input, Runnable body){
        Input in = new Input(input);
        if(input && !in.pressing){
            body.run();
            in.pressing = true;
        }
        else if(!input) in.pressing = false;
    }
    public void hold(Runnable body, Runnable elif){
        if(input) body.run();
        else if(elif != null) elif.run();
    }
    public void toggle(Runnable body, Runnable body2){
        press(()-> {
            if (!pressed)
            {body.run(); pressed = true;}
            else
            {body2.run();pressed = false;}
        });
    }

    public boolean get(){
        return this.input;
    }

    public static Input
            a, b, x, y,
            dPadUp, dPadDown, dPadLeft, dPadRight,
            lBump, rBump, lTrigger, rTrigger,
            back, lStickButton, rStickButton;
    public static Input
            a2, b2, x2, y2,
            dPadUp2, dPadDown2, dPadLeft2, dPadRight2,
            lBump2, rBump2, lTrigger2, rTrigger2,
            back2, lStickButton2, rStickButton2;
    public static void initInputs(Gamepad gamepad){
        a = new Input(gamepad.a);
        b = new Input(gamepad.b);
        x = new Input(gamepad.x);
        y = new Input(gamepad.y);
        dPadUp = new Input(gamepad.dpad_up);
        dPadDown = new Input(gamepad.dpad_down);
        dPadLeft = new Input(gamepad.dpad_left);
        dPadRight = new Input(gamepad.dpad_right);
        lBump = new Input(gamepad.left_bumper);
        rBump = new Input(gamepad.right_bumper);
        lTrigger = new Input(gamepad.left_trigger);
        rTrigger = new Input(gamepad.right_trigger);
        back = new Input(gamepad.back);
        lStickButton = new Input(gamepad.left_stick_button);
        rStickButton = new Input(gamepad.right_stick_button);
    }
    public static void initInputs(Gamepad gamepad1, Gamepad gamepad2){
        initInputs(gamepad1);
        a2 = new Input(gamepad2.a);
        b2 = new Input(gamepad2.b);
        x2 = new Input(gamepad2.x);
        y2 = new Input(gamepad2.y);
        dPadUp2 = new Input(gamepad2.dpad_up);
        dPadDown2 = new Input(gamepad2.dpad_down);
        dPadLeft2 = new Input(gamepad2.dpad_left);
        dPadRight2 = new Input(gamepad2.dpad_right);
        lBump2 = new Input(gamepad2.left_bumper);
        rBump2 = new Input(gamepad2.right_bumper);
        lTrigger2 = new Input(gamepad2.left_trigger);
        rTrigger2 = new Input(gamepad2.right_trigger);
        back2 = new Input(gamepad2.back) ;
        lStickButton2 = new Input(gamepad2.left_stick_button);
        rStickButton2 = new Input(gamepad2.right_stick_button);
    }

}
