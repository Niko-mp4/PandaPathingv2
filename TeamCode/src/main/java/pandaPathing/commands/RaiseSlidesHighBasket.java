package pandaPathing.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import pandaPathing.subsytem.Claw;
import pandaPathing.subsytem.Lift;
import pandaPathing.util.Timer;

public class RaiseSlidesHighBasket extends CommandBase {
    private final Lift lift;
    private final Claw claw;

    private int state = 0;
    private Timer timer;

    public RaiseSlidesHighBasket(Lift lift, Claw claw) {
        this.lift = lift;
        this.claw = claw;
        addRequirements(this.lift, this.claw);
    }

    @Override
    public void initialize() {setState(1);}

    @Override
    public void execute() {
        double time = timer.getElapsedTime();
        switch(state) {
            case 1:
                lift.setLiftState(Lift.LiftState.HIGH_BASKET);
                if (lift.atTargetWithin(100))
                    setState(2);
                break;

            case 2:
                claw.setV4BState(Claw.V4BState.DEPOSIT);
                claw.setPitchState(Claw.PitchState.DEPOSIT);
                if(time > 200) claw.setRollState(Claw.RollState.NINETY);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2;
    }

    public void setState(int state){
        this.state = state;
        timer.reset();
    }
}
