package edu.exeter.apex.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import pedroPathing.Constants;
import pedroPathing.poses;
@Autonomous(name = "blueBottom", group = "Examples")
public class blueBottom extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPoseBlueTop = new Pose(56, 88, Math.toRadians(180)); //Aligned with top right of robot on W4
    private final Pose startPoseBlueBottom = new Pose(56, 16, Math.toRadians(180)); //Aligned with top right of robot on W1
    private final Pose blueTopRowStart = new Pose(40, 84, Math.toRadians(180));
    private final Pose blueTopRowEnd = new Pose(20, 84, Math.toRadians(180));
    private final Pose blueMiddleRowStart = new Pose(40, 60, Math.toRadians(180));
    private final Pose blueMiddleRowEnd = new Pose(20, 60, Math.toRadians(180));
    private final Pose blueBottomRowStart = new Pose(40, 36, Math.toRadians(180));
    private final Pose blueBottomRowEnd = new Pose(20, 36, Math.toRadians(180));
    private final Pose blueScore = new Pose(); //Scoring Pose will be determined by testing

    //Red Alliance Poses
    private final Pose startPoseRedTop = new Pose(88, 88, Math.toRadians(180)); //Aligned with top left of robot on Y4
    private final Pose startPoseRedBottom = new Pose(88, 16, Math.toRadians(180)); //Aligned with left right of robot on Y1
    private final Pose redTopRowStart = new Pose(104, 84, Math.toRadians(180));
    private final Pose redTopRowEnd = new Pose(124, 84, Math.toRadians(180));
    private final Pose redMiddleRowStart = new Pose(104, 60, Math.toRadians(180));
    private final Pose redMiddleRowEnd = new Pose(124, 60, Math.toRadians(180));
    private final Pose redBottomRowStart = new Pose(104, 36, Math.toRadians(180));
    private final Pose redBottomRowEnd = new Pose(124, 36, Math.toRadians(180));
    private final Pose redScore = new Pose();

    private PathChain blueBottomToScore;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseBlueBottom);
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

        blueBottomToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPoseBlueBottom, blueScore)))
                .setLinearHeadingInterpolation(startPoseBlueBottom.getHeading(), blueScore.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(blueBottomToScore);
                break;
        }
    }
}