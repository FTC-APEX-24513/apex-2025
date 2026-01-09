package edu.exeter.apex.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Suppress("UNUSED")
val redBottom3 = Mercurial.autonomous {
    // subsystems
    val intake = IntakeSubsystem(hardwareMap)
    val outtake = OuttakeSubsystem(hardwareMap)
    val transfer = TransferSubsystem(hardwareMap)
    val spindexer = SpindexerSubsystem(hardwareMap)
    val limelight = LimelightSubsystem(hardwareMap)

    // pathing
    val follower: Follower = Constants.createFollower(hardwareMap)
    val pathTimer = Timer()
    var pathState = 0

    val startPoseRedBottom = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(180.0))
    val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(180.0))
    val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    val redScore = Pose(112.0, 48.0, Math.toRadians(180.0))

    // paths
    lateinit var redStartToScore: Path
    lateinit var redScoreToTop: PathChain
    lateinit var redTopIntake: PathChain
    lateinit var redTopToScore: PathChain
    lateinit var redScoreToMiddle: PathChain
    lateinit var redMiddleIntake: PathChain
    lateinit var redMiddleToScore: PathChain
    lateinit var redScoreToBottom: PathChain
    lateinit var redBottomIntake: PathChain
    lateinit var redBottomToScore: PathChain

    fun buildPaths() {
        redStartToScore = Path(BezierLine(startPoseRedBottom, redScore))
        redStartToScore.setLinearHeadingInterpolation(startPoseRedBottom.heading, redScore.heading)

        redScoreToTop = follower.pathBuilder()
            .addPath(BezierLine(redScore, redTopRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redTopRowStart.heading)
            .build()

        redTopIntake = follower.pathBuilder()
            .addPath(BezierLine(redTopRowStart, redTopRowEnd))
            .setLinearHeadingInterpolation(redTopRowStart.heading, redTopRowEnd.heading)
            .build()

        redTopToScore = follower.pathBuilder()
            .addPath(BezierLine(redTopRowEnd, redScore))
            .setLinearHeadingInterpolation(redTopRowEnd.heading, redScore.heading)
            .build()

        redScoreToMiddle = follower.pathBuilder()
            .addPath(BezierLine(redScore, redMiddleRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redMiddleRowStart.heading)
            .build()

        redMiddleIntake = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowStart, redMiddleRowEnd))
            .setLinearHeadingInterpolation(redMiddleRowStart.heading, redMiddleRowEnd.heading)
            .build()

        redMiddleToScore = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowEnd, redScore))
            .setLinearHeadingInterpolation(redMiddleRowEnd.heading, redScore.heading)
            .build()

        redScoreToBottom = follower.pathBuilder()
            .addPath(BezierLine(redScore, redBottomRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redBottomRowStart.heading)
            .build()

        redBottomIntake = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowStart, redBottomRowEnd))
            .setLinearHeadingInterpolation(redBottomRowStart.heading, redBottomRowEnd.heading)
            .build()

        redBottomToScore = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowEnd, redScore))
            .setLinearHeadingInterpolation(redBottomRowEnd.heading, redScore.heading)
            .build()
    }

    // initialize
    buildPaths()
    follower.setStartingPose(startPoseRedBottom)

    // main autonomous sequence
    schedule(
        sequence(
            exec { pathState = 0 },
            loop(
                exec {
                    follower.update()
                    when (pathState) {
                        0 -> {
                            follower.followPath(redStartToScore)
                            outtake.launch()
                            pathState = 1
                        }
                        1 -> if (!follower.isBusy) {
                            follower.followPath(redScoreToTop)
                            pathState = 2
                        }
                        2 -> if (!follower.isBusy) {
                            intake.collect()
                            follower.followPath(redTopIntake)
                            pathState = 3
                        }
                        3 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redTopToScore)
                            outtake.launch()
                            pathState = 4
                        }
                        4 -> if (!follower.isBusy) {
                            follower.followPath(redScoreToMiddle)
                            pathState = 5
                        }
                        5 -> if (!follower.isBusy) {
                            intake.collect()
                            follower.followPath(redMiddleIntake)
                            pathState = 6
                        }
                        6 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redMiddleToScore)
                            outtake.launch()
                            pathState = 7
                        }
                        7 -> if (!follower.isBusy) {
                            follower.followPath(redScoreToBottom)
                            pathState = 8
                        }
                        8 -> if (!follower.isBusy) {
                            intake.collect()
                            follower.followPath(redBottomIntake)
                            pathState = 9
                        }
                        9 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redBottomToScore)
                            outtake.launch()
                            pathState = 10
                        }
                        10 -> if (!follower.isBusy) pathState = 11
                        11 -> { /* Finished */ }
                    }
                }
            )
        )
    )

    dropToScheduler()
}
