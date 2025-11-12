package edu.exeter.apex.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import edu.exeter.apex.ftc.teamcode.subsystems.Intake;


import pedroPathing.Constants;

@Autonomous(name = "blueBottom_1", group = "Examples")
public class blueBottom_1 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPoseBlueBottom = new Pose(56, 16, Math.toRadians(180)); //Aligned with top right of robot on W1
    private final Pose blueBottomRowStart = new Pose(40, 36, Math.toRadians(180));
    private final Pose blueBottomRowEnd = new Pose(20, 36, Math.toRadians(180));
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing

    private Intake intake;

    private PathChain blueStartToScore, blueScoreToBottom, blueBottomIntake, blueBottomToScore;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseBlueBottom);
        intake = new Intake(hardwareMap);
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


        //Limelight localization
        //Shoot here

        blueScoreToBottom = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueScore, blueBottomRowStart)))
                .setLinearHeadingInterpolation(blueScore.getHeading(), blueBottomRowStart.getHeading())
                .build();

        blueBottomIntake = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueBottomRowStart, blueBottomRowEnd)))
                .setLinearHeadingInterpolation(blueBottomRowStart.getHeading(), blueBottomRowEnd.getHeading())
                .build();

        blueBottomToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueBottomRowEnd, blueScore)))
                .setLinearHeadingInterpolation(blueBottomRowEnd.getHeading(), blueScore.getHeading())
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
                // Return to score after middle row
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToBottom);
                    setPathState(2);
                }
                break;

            case 2:
                // Move to bottom row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueBottomIntake);
                    setPathState(3);
                }
                break;

            case 3:
                // Finish bottom intake
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueBottomToScore);
                    setPathState(4);
                }
                break;

            case 4:
                // Final score or park
                if (!follower.isBusy()) {
                    setPathState(5); // End of routine
                }
                break;

            case 5:
                // Autonomous routine finished
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}