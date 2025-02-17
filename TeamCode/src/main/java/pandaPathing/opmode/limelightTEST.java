package pandaPathing.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;

import pandaPathing.constants.FConstants;
import pandaPathing.constants.LConstants;
import pandaPathing.constants.PIDConfig;
import pandaPathing.robot.Hardware;
import pandaPathing.robot.RobotConstants;
import pandaPathing.util.Vec2;

@Config
@TeleOp(name = "limeyTest!")
public class limelightTEST extends LinearOpMode {

    List<Vec2> sampling = new ArrayList<>();
    public void runOpMode(){
        Hardware robot = new Hardware(hardwareMap);
        Follower follower;

        int frames = 0;
        double relX = 0;
        double relY = 0;
        double tx = 0;
        double ty = 0;
        Path toSample = null;
        int nextPath = 0;
        

        PIDConfig.translationalP = 1.4;
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        robot.limelight.start();
        robot.limelight.pipelineSwitch(0);

        waitForStart();
        while(opModeIsActive()){
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            double r = follower.getPose().getHeading();

            LLResult result = robot.limelight.getLatestResult();

            //If the followr is not currently running
            if(!follower.isBusy()) {
                // record a snapshot if a sample is detected (!null and within area threshold)
                if (result != null && result.getTa() > 2 && frames < 15) {
                    Vec2 position = new Vec2(result.getTx(), result.getTy());
                    // only add a sample to the list if it the first sample or it is within the position threshold of the last sample
                    if(frames == 0 || (position.x-5 <= sampling.get(frames-1).x && sampling.get(frames-1).x <= position.x+5)) {
                        sampling.add(position);
                        frames++;
                    } else {
                        // otherwise, reset the list and restart counting
                        sampling.clear();
                        frames = 0;
                    }
                } else if(result != null && result.getTa() > 2){
                    Vec2 resultPos = new Vec2(0.0, 0.0);
                    for (Vec2 sample : sampling)
                        resultPos.add(sample);
                    resultPos.divide(15);

                    relX = limeToIn(resultPos.x, resultPos.y);
                    relY = resultPos.y;

                    ty = y - relX;
                    tx = x + Math.abs(relY+20)/40;

                    toSample = new Path(new BezierLine(
                            new Point(x, y),
                            new Point(x, ty)
                    ));
                    toSample.setConstantHeadingInterpolation(r);
                    follower.followPath(toSample);

                    frames = 0;
                    sampling.clear();
                }
            }
            follower.update();
            telemetry.addLine("frame " + frames);
            telemetry.addLine("result " + ((result != null) ? result.getTa() + "" : "null"));
            telemetry.addLine("robot (x, y) = ("+follower.getPose().getX()+", "+follower.getPose().getY()+")");
            telemetry.addLine("limel (x, y) = ("+relX+", "+relY+")");
            telemetry.addLine("targe (x, y) = ("+tx+", "+ty+")");
            telemetry.update();
        }
    }

    public double limeToIn(double distance, double fovUnit){
        return (distance/40)*(7.6549983233 + Math.pow(1.02183885967, fovUnit));
    }

    public Double sampleYaw(LLResult analysis) {
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        for (LLResultTypes.ColorResult cr : crs) {
            Pose3D samplePos = cr.getTargetPoseCameraSpace();
            return samplePos.getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return null;
    }
}
