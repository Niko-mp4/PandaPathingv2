package pandaPathing.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import pandaPathing.subsytem.Claw;
import pandaPathing.subsytem.Lift;
import pandaPathing.util.Timer;

public class Deposit extends CommandBase {
    private final Lift lift;
    private final Claw claw;

    private int state = 0;
    private final Timer timer = new Timer();

    public Deposit(Lift lift, Claw claw) {
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
                if(lift.is(Lift.LiftState.BOTTOM)){
                    claw.setV4BState(Claw.V4BState.DEPOSIT);
                    claw.setPitchState(Claw.PitchState.DEPOSIT);
                }
                claw.setGrabState(Claw.GrabState.OPEN);
                if(time > 400) setState(2);
                break;

            case 2:
                claw.setPitchState(Claw.PitchState.DOWN);
                claw.setV4BState(Claw.V4BState.UP);
                claw.setRollState(Claw.RollState.ZERO);
                if (lift.is(Lift.LiftState.HIGH_BASKET) && time > 300)
                    setState(3);
                break;

            case 3:
                lift.setLiftState(Lift.LiftState.BOTTOM);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2 && !lift.is(Lift.LiftState.HIGH_BASKET)
                || state == 3;
    }

    public void setState(int state){
        this.state = state;
        timer.reset();
    }
}
