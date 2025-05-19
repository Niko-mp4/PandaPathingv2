package pandaPathing.subsytem;

public class Robot {
    public enum RobotState {
        POOPSHOOT,
        HIGH_BASKET_DEPOSIT
    }

    // Global state variable
    public static RobotState current = RobotState.POOPSHOOT;


    public static void set(RobotState robotState) {
        current = robotState;
    }

    public static boolean is(RobotState robotState) {
        return current == robotState;
    }
}