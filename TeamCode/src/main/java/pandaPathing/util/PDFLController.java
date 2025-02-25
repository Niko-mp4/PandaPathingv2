package pandaPathing.util;

public class PDFLController {

    double kP, kD, kF, kL;

    private Timer timer = new Timer();

    double slowDown = 0.35;

    private RingBuffer<Double> timeBuffer = new RingBuffer<Double>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<Double>(3, 0.0);

    public PDFLController(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void updatePDFLConstants(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public double calculatePow(double targetPos, double actualPos){
        double error = actualPos - targetPos;

        // calculate error derivative
        double time = timer.getElapsedTime();
        double previous_time = timeBuffer.getValue(time);
        double previous_error = errorBuffer.getValue(error);
        double delta_time = time - previous_time;
        double delta_error = error - previous_error;
        double derivative = delta_error / delta_time;

        //If the PDFL hasn't been updated, reset it
        if (delta_time > 200){
            reset();
            return calculatePow(targetPos, actualPos);
        }

        double response =
                proportionalError(error)
                        + differentialError(derivative)
                        + gravityComp()
                        + frictionComp(error);

        if (error < 0){
            response = slowDown * response;
        }

        return response;
    }


    private double proportionalError(double error){
        return kP * error;
    }

    private double differentialError(double derivative){
        return kD * derivative;
    }

    private double gravityComp(){
        double response = kF;
        return response;
    }

    private double frictionComp(double error){
        return kL * Math.signum(error);
    }

    public void reset(){
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.resetTimer();
    }
}