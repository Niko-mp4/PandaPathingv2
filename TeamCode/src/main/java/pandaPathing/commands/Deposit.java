package pandaPathing.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import pandaPathing.subsytem.Claw;
import pandaPathing.subsytem.Lift;
import pandaPathing.subsytem.Robot;
import pandaPathing.util.Timer;

public class Deposit extends CommandBase {
    private final Lift lift;
    private final Claw claw;

    private int state = 0;
    private Timer wait = new Timer();

    public Deposit(Lift lift, Claw claw) {
        this.lift = lift;
        this.claw = claw;
        addRequirements(lift, claw);
    }

    @Override
    public void initialize() {
        state = 1;
    }

    @Override
    public void execute() {
        switch(state) {
            case 1:
                claw.setGrabState(Claw.GrabState.OPEN);
                if (wait.waitMs(200)) {
                    state = 2;
                }
                break;

            case 2:
                claw.setPitchState(Claw.PitchState.IN_ROBOT);
                claw.setV4BState(Claw.V4BState.UP_V4B);
                claw.setRollState(Claw.RollState.ZERO);
                if (Robot.is(Robot.RobotState.HIGH_BASKET_DEPOSIT)) {
                    if (wait.waitMs(300)) {
                        state = 3;
                    }
                }
                break;

            case 3:
                lift.toZero();
                Robot.set(Robot.RobotState.POOPSHOOT);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2 && !Robot.is(Robot.RobotState.HIGH_BASKET_DEPOSIT)
                || state == 3;
    }
}
