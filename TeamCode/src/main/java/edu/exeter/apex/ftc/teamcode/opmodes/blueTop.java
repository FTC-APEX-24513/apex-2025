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

//will do all three
@Autonomous(name = "Example Auto", group = "Examples")
public abstract class blueTop extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Path scorePreload;
    private PathChain blueScoreToTopStart, blueTopStartToTopEnd, blueTopEndtoScore, blueScoreToMiddleStart, blueMiddleStartToMiddleEnd, blueMiddleEndToScore, blueScoreToBottomStart, blueBottomStartToBottomEnd, blueBottomEndToScore;
    private final Pose startPoseBlueTop = new Pose(56, 88, Math.toRadians(180)); //Aligned with top right of robot on W4
    private final Pose blueTopRowStart = new Pose(40, 84, Math.toRadians(180));
    private final Pose blueTopRowEnd = new Pose(20, 84, Math.toRadians(180));
    private final Pose blueMiddleRowStart = new Pose(40, 60, Math.toRadians(180));
    private final Pose blueMiddleRowEnd = new Pose(20, 60, Math.toRadians(180));
    private final Pose blueBottomRowStart = new Pose(40, 36, Math.toRadians(180));
    private final Pose blueBottomRowEnd = new Pose(20, 36, Math.toRadians(180));
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing
    private Intake intake;
    private Outtake outtake;
    private LimelightJava limelight;
    public void buildPaths() {
        /* straight to score */
        scorePreload = new Path(new BezierLine(startPoseBlueTop, blueScore));
        scorePreload.setLinearHeadingInterpolation(startPoseBlueTop.getHeading(), blueScore.getHeading());

        /* all three pt 1/just closest pt 1*/
        blueScoreToTopStart = follower.pathBuilder()
                .addPath(new BezierLine(blueScore, blueTopRowStart))
                .setLinearHeadingInterpolation(blueScore.getHeading(), blueTopRowStart.getHeading())
                .build();

        /* all three pt 2/just closest pt 2 */
        blueTopStartToTopEnd = follower.pathBuilder()
                .addPath(new BezierLine(blueTopRowStart, blueTopRowEnd))
                .setLinearHeadingInterpolation(blueTopRowStart.getHeading(), blueTopRowEnd.getHeading())
                .build();

        /* all three pt 3/just closest pt 3*/
        blueTopEndtoScore = follower.pathBuilder()
                .addPath(new BezierLine(blueTopRowEnd, blueScore))
                .setLinearHeadingInterpolation(blueTopRowEnd.getHeading(), blueScore.getHeading())
                .build();

        /* all three pt 4*/
        blueScoreToMiddleStart = follower.pathBuilder()
                .addPath(new BezierLine(blueScore, blueMiddleRowStart))
                .setLinearHeadingInterpolation(blueScore.getHeading(), blueMiddleRowStart.getHeading())
                .build();

        /* all three pt 5 */
        blueMiddleStartToMiddleEnd = follower.pathBuilder()
                .addPath(new BezierLine(blueMiddleRowStart, blueMiddleRowEnd))
                .setLinearHeadingInterpolation(blueMiddleRowStart.getHeading(), blueMiddleRowEnd.getHeading())
                .build();

        /* all three pt 6 */
        blueMiddleEndToScore = follower.pathBuilder()
                .addPath(new BezierLine(blueMiddleRowEnd, blueScore))
                .setLinearHeadingInterpolation(blueMiddleRowEnd.getHeading(), blueScore.getHeading())
                .build();

        /* all three pt 7*/
        blueScoreToBottomStart = follower.pathBuilder()
                .addPath(new BezierLine(blueScore, blueBottomRowStart))
                .setLinearHeadingInterpolation(blueScore.getHeading(), blueBottomRowStart.getHeading())
                .build();

        /* all three pt 8 */
        blueBottomStartToBottomEnd = follower.pathBuilder()
                .addPath(new BezierLine(blueBottomRowStart, blueBottomRowEnd))
                .setLinearHeadingInterpolation(blueBottomRowStart.getHeading(), blueBottomRowEnd.getHeading())
                .build();

        /* all three pt 9 */
        blueBottomEndToScore = follower.pathBuilder()
                .addPath(new BezierLine(blueBottomRowEnd, blueScore))
                .setLinearHeadingInterpolation(blueBottomRowEnd.getHeading(), blueScore.getHeading())
                .build();

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
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        limelight = new LimelightJava();
        limelight.init(hardwareMap, telemetry);
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
                // Wait until start-to-score path is done
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToTopStart);
                    setPathState(2);
                }
                break;

            case 2:
                // Move to top row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueTopStartToTopEnd);
                    setPathState(3);
                }
                break;

            case 3:
                // After finishing top intake path
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueTopEndtoScore);
                    limelight.update();
                    limelight.localize();
                    outtake.outtake();
                    setPathState(4);
                }
                break;

            case 4:
                // Return from top row to scoring
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToMiddleStart);
                    setPathState(5);
                }
                break;

            case 5:
                // Move to middle row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueMiddleStartToMiddleEnd);
                    setPathState(6);
                }
                break;

            case 6:
                // Finish middle intake
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueMiddleEndToScore);
                    limelight.update();
                    limelight.localize();
                    outtake.outtake();
                    setPathState(7);
                }
                break;

            case 7:
                // Return to score after middle row
                if (!follower.isBusy()) {
                    follower.followPath(blueScoreToBottomStart);
                    setPathState(8);
                }
                break;

            case 8:
                // Move to bottom row for intake
                if (!follower.isBusy()) {
                    intake.intake();
                    follower.followPath(blueBottomStartToBottomEnd);
                    setPathState(9);
                }
                break;

            case 9:
                // Finish bottom intake
                if (!follower.isBusy()) {
                    intake.intakeStop();
                    follower.followPath(blueBottomEndToScore);
                    limelight.update();
                    limelight.localize();
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
