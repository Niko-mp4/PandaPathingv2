package pandaPathing.subsytem;

public class Robot {
    public enum RobotState {
        SAMPLE,
        SPECIMEN
    }

    // Global state variable
    public static RobotState current = RobotState.SAMPLE;
    public static void set(RobotState robotState) {
        current = robotState;
    }
    public static boolean is(RobotState robotState) {
        return current == robotState;
    }

}