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
import edu.exeter.apex.ftc.teamcode.subsystems.Outtake;

import pedroPathing.Constants;

@Autonomous(name = "blueBottom_3", group = "Examples")
public class blueBottom_3 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPoseBlueBottom = new Pose(56, 16, Math.toRadians(180)); //Aligned with top right of robot on W1
    private final Pose blueTopRowStart = new Pose(40, 84, Math.toRadians(180));
    private final Pose blueTopRowEnd = new Pose(20, 84, Math.toRadians(180));
    private final Pose blueMiddleRowStart = new Pose(40, 60, Math.toRadians(180));
    private final Pose blueMiddleRowEnd = new Pose(20, 60, Math.toRadians(180));
    private final Pose blueBottomRowStart = new Pose(40, 36, Math.toRadians(180));
    private final Pose blueBottomRowEnd = new Pose(20, 36, Math.toRadians(180));
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing

    private Intake intake;
    private Outtake outtake;

    private PathChain blueStartToScore, blueScoreToTop, blueTopIntake, blueTopToScore, blueScoreToMiddle, blueMiddleIntake, blueMiddleToScore, blueScoreToBottom, blueBottomIntake, blueBottomToScore;

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
        outtake = new Outtake(hardwareMap);
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

        blueScoreToTop = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueScore, blueTopRowStart)))
                .setLinearHeadingInterpolation(blueScore.getHeading(), blueTopRowStart.getHeading())
                .build();

        blueTopIntake = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueTopRowStart, blueTopRowEnd)))
                .setLinearHeadingInterpolation(blueTopRowStart.getHeading(), blueTopRowEnd.getHeading())
                .build();

        blueTopToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueTopRowEnd, blueScore)))
                .setLinearHeadingInterpolation(blueTopRowEnd.getHeading(), blueScore.getHeading())
                .build();

        //Limelight localization
        //Shoot here

        blueScoreToMiddle = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueScore, blueMiddleRowStart)))
                .setLinearHeadingInterpolation(blueScore.getHeading(), blueMiddleRowStart.getHeading())
                .build();

        blueMiddleIntake = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueMiddleRowStart, blueMiddleRowEnd)))
                .setLinearHeadingInterpolation(blueMiddleRowStart.getHeading(), blueMiddleRowEnd.getHeading())
                .build();

        blueMiddleToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(blueMiddleRowEnd, blueScore)))
                .setLinearHeadingInterpolation(blueMiddleRowEnd.getHeading(), blueScore.getHeading())
                .build();

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
                outtake.outtake();
                setPathState(1);
                break;

            case 1:
                // Wait until start-to-score path is done
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToTop);
                    setPathState(2);
                }
                break;

            case 2:
                // Move to top row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueTopIntake);
                    setPathState(3);
                }
                break;

            case 3:
                // After finishing top intake path
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueTopToScore);
                    outtake.outtake();
                    setPathState(4);
                }
                break;

            case 4:
                // Return from top row to scoring
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToMiddle);
                    setPathState(5);
                }
                break;

            case 5:
                // Move to middle row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueMiddleIntake);
                    setPathState(6);
                }
                break;

            case 6:
                // Finish middle intake
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueMiddleToScore);
                    outtake.outtake();
                    setPathState(7);
                }
                break;

            case 7:
                // Return to score after middle row
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToBottom);
                    setPathState(8);
                }
                break;

            case 8:
                // Move to bottom row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueBottomIntake);
                    setPathState(9);
                }
                break;

            case 9:
                // Finish bottom intake
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueBottomToScore);
                    outtake.outtake();
                    setPathState(10);
                }
                break;

            case 10:
                // Final score or park
                if (!follower.isBusy()) {
                    setPathState(11); // End of routine
                }
                break;

            case 11:
                // Autonomous routine finished
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
}