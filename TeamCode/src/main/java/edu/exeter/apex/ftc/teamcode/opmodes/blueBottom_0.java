package edu.exeter.apex.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import pedroPathing.Constants;

@Autonomous(name = "blueBottom_0", group = "Examples")
public class blueBottom_0 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPoseBlueBottom = new Pose(56, 16, Math.toRadians(180)); //Aligned with top right of robot on W1
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing

    private PathChain blueStartToScore;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseBlueBottom);
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

    public void buildPaths() {

        blueStartToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPoseBlueBottom, blueScore)))
                .setLinearHeadingInterpolation(startPoseBlueBottom.getHeading(), blueScore.getHeading())
                .build();

        //Limelight localization
        //Shoot here


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start → first scoring position
                follower.followPath(blueStartToScore);
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