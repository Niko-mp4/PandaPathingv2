package pandaPathing.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Input {
    private final boolean input;
    private boolean pressed;
    private Runnable body;

    public Input(boolean input){
        this.input = input;
    }
    public Input(double input){
        this.input = input > 0;
    }

    public void run(String action, Runnable body) {
        this.body = body;
        if (action.equals("while"))
            whilePressed();
        else singlePress();
    }
    public void run(Runnable body) {
        run("", body);
    }

    public void singlePress(){
        if(input && !pressed){
            body.run();
            pressed = true;
        }
        else if(!input) pressed = false;
    }
    public void whilePressed(){
        if(input) body.run();
    }

}
