package edu.exeter.apex.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.exeter.apex.ftc.teamcode.subsystems.Intake;
import edu.exeter.apex.ftc.teamcode.subsystems.LimelightJava;
import edu.exeter.apex.ftc.teamcode.subsystems.Outtake;
import pedroPathing.Constants;
import pedroPathing.poses;

//will just score
@Autonomous(name = "Example Auto", group = "Examples")
public abstract class blueTop3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Path scorePreload;
    private final Pose startPoseBlueTop = new Pose(56, 88, Math.toRadians(180)); //Aligned with top right of robot on W4
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing
    private Intake intake;
    private Outtake outtake;
    private LimelightJava limelight;
    public void buildPaths() {
        /* straight to score */
        scorePreload = new Path(new BezierLine(startPoseBlueTop, blueScore));
        scorePreload.setLinearHeadingInterpolation(startPoseBlueTop.getHeading(), blueScore.getHeading());
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseBlueTop);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start → first scoring position
                follower.followPath(scorePreload);
                outtake.outtake();
                setPathState(1);
                break;
            case 1:
                // Final score or park
                if (!follower.isBusy()) {
                    setPathState(2); // End of routine
                }
                break;

            case 2:
                // Autonomous routine finished
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}
