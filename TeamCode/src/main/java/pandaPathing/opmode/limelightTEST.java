package pandaPathing.opmode;

import static pandaPathing.robot.RobotConstants.clawClose;
import static pandaPathing.robot.RobotConstants.clawOpen;
import static pandaPathing.robot.RobotConstants.railLMax;
import static pandaPathing.robot.RobotConstants.railRMax;
import static pandaPathing.robot.RobotConstants.v4bFDown;
import static pandaPathing.robot.RobotConstants.v4bFUp;

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

        boolean dLeftPressed = false, dRightPressed = false, neckUp = false, grabAction = false;
        double grabStartTime = 0, grabTime = 0;

        robot.railL.setPosition(railLMax);
        robot.railR.setPosition(railRMax);
        

        PIDConfig.translationalP = 1;
        //PIDConfig.driveP = 0.02;
        Constants.setConstants(FConstants.class, LConstants.class);
        //Constants.pathEndTranslationalConstraint = 0;
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
            //if(!follower.isBusy()) {
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

                    relX = limeXToIn(resultPos.x, resultPos.y);
                    relY = limeYToIn(resultPos.y);

                    ty = y - relX;
                    tx = x + relY - 3;

                    toSample = new Path(new BezierLine(
                            new Point(x, y),
                            new Point(tx, ty)
                    ));
                    toSample.setConstantHeadingInterpolation(0);
                    if(gamepad1.dpad_left && !dLeftPressed) {
                        follower.followPath(toSample);
                        dLeftPressed = true;
                    } else if(!gamepad1.dpad_left)
                        dLeftPressed = false;

                    frames = 0;
                    sampling.clear();
                }

            if (gamepad1.dpad_right && !dRightPressed) {
                if (!neckUp) {
                    robot.v4b.setPosition(v4bFUp);
                    grabStartTime = System.currentTimeMillis();
                    neckUp = true;
                } else {
                    robot.v4b.setPosition(v4bFDown);
                    grabStartTime = System.currentTimeMillis();
                    neckUp = false;
                }
                dRightPressed = true;
            } else if (!gamepad1.dpad_right) dRightPressed = false;
            grabTime = (System.currentTimeMillis() - grabStartTime) / 1000.0;
            if (grabTime > 0.12 && !grabAction) { // timer to open/close claw after v4b moves
                robot.lilJarret.setPosition(neckUp ? clawOpen : clawClose);
                grabAction = true;
            } else if (grabTime < 0.12) grabAction = false;
            //}
            follower.update();
            telemetry.addLine("frame " + frames);
            telemetry.addLine("result " + ((result != null) ? result.getTa() : "null"));
            telemetry.addLine("robot (x, y) = ("+follower.getPose().getX()+", "+follower.getPose().getY()+")");
            telemetry.addLine("limel (x, y) = ("+relX+", "+relY+")");
            telemetry.addLine("targe (x, y) = ("+tx+", "+ty+")");
            telemetry.addLine("angle " + ((result != null) ?
                    result.getColorResults().get(0).getTargetCorners().get(0).get(0)-
                    result.getColorResults().get(0).getTargetCorners().get(2).get(0)
                    : "null"));
            telemetry.update();
        }
    }

    public double limeXToIn(double distance, double fovUnit){
        return (distance/50)*(7.6549983233 + Math.pow(1.02183885967, fovUnit));
    }

    public double limeYToIn(double distance){
        return (distance+20)/40*12;
    }

    public Double sampleYaw(LLResult analysis) {
        if (analysis == null) return null;
        List<LLResultTypes.ColorResult> crs = analysis.getColorResults();
        for (LLResultTypes.ColorResult cr : crs) {
            return cr.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return null;
    }
}
