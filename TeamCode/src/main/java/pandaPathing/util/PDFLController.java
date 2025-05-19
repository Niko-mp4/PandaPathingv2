package pandaPathing.util;

public class PDFLController {

    double kP, kD, kF, kL;

    private Timer timer = new Timer();

    double slowDown = 0.35;

    private RingBuffer<Double> timeBuffer = new RingBuffer<>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<>(3, 0.0);

    public PDFLController(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void updatePDFLConstants(double kP, double kD, double kF, double kL) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public double calculatePow(double targetPos, double actualPos) {
        double error = actualPos - targetPos;

        double time = timer.getElapsedTime();

        // Use the oldest (overwritten) values as previous time and error
        double previous_time = timeBuffer.add(time);
        double previous_error = errorBuffer.add(error);

        double delta_time = time - previous_time;
        double delta_error = error - previous_error;
        double derivative = delta_time != 0 ? delta_error / delta_time : 0.0;

        // If the controller hasn’t been updated recently, reset it
        if (delta_time > 200) {
            reset();
            return calculatePow(targetPos, actualPos);
        }

        double response =
                proportionalError(error)
                        + differentialError(derivative)
                        + gravityComp()
                        + frictionComp(error);

        if (error < 0) {
            response *= slowDown;
        }

        return response;
    }

    private double proportionalError(double error) {
        return kP * error;
    }

    private double differentialError(double derivative) {
        return kD * derivative;
    }

    private double gravityComp() {
        return kF;
    }

    private double frictionComp(double error) {
        return kL * Math.signum(error);
    }

    public void reset() {
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.reset();
    }
}
