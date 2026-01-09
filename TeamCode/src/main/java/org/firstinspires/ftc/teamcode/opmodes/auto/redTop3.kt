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
val redTop3 = Mercurial.autonomous {
    // Subsystems
    val intake = IntakeSubsystem(hardwareMap)
    val outtake = OuttakeSubsystem(hardwareMap)
    val transfer = TransferSubsystem(hardwareMap)
    val spindexer = SpindexerSubsystem(hardwareMap)
    val limelight = LimelightSubsystem(hardwareMap)

    // Pathing
    val follower: Follower = Constants.createFollower(hardwareMap)
    val pathTimer = Timer()
    var pathState = 0

    val startPoseredTop = Pose(88.0, 88.0, Math.toRadians(180.0))
    val startPoseredBottom = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(90.0))
    val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(90.0))
    val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    val redScore = Pose() // As in original

    // Paths
    lateinit var scorePreload: Path
    lateinit var redScoreToTopStart: PathChain
    lateinit var redTopStartToTopEnd: PathChain
    lateinit var redTopEndToScore: PathChain
    lateinit var redScoreToMiddleStart: PathChain
    lateinit var redMiddleStartToMiddleEnd: PathChain
    lateinit var redMiddleEndToScore: PathChain
    lateinit var redScoreToBottomStart: PathChain
    lateinit var redBottomStartToBottomEnd: PathChain
    lateinit var redBottomEndToScore: PathChain

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseredTop, redScore)).apply {
            setLinearHeadingInterpolation(startPoseredTop.heading, redScore.heading)
        }

        redScoreToTopStart = follower.pathBuilder()
            .addPath(BezierLine(redScore, redTopRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redTopRowStart.heading)
            .build()

        redTopStartToTopEnd = follower.pathBuilder()
            .addPath(BezierLine(redTopRowStart, redTopRowEnd))
            .setLinearHeadingInterpolation(redTopRowStart.heading, redTopRowEnd.heading)
            .build()

        redTopEndToScore = follower.pathBuilder()
            .addPath(BezierLine(redTopRowEnd, redScore))
            .setLinearHeadingInterpolation(redTopRowEnd.heading, redScore.heading)
            .build()

        redScoreToMiddleStart = follower.pathBuilder()
            .addPath(BezierLine(redScore, redMiddleRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redMiddleRowStart.heading)
            .build()

        redMiddleStartToMiddleEnd = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowStart, redMiddleRowEnd))
            .setLinearHeadingInterpolation(redMiddleRowStart.heading, redMiddleRowEnd.heading)
            .build()

        redMiddleEndToScore = follower.pathBuilder()
            .addPath(BezierLine(redMiddleRowEnd, redScore))
            .setLinearHeadingInterpolation(redMiddleRowEnd.heading, redScore.heading)
            .build()

        redScoreToBottomStart = follower.pathBuilder()
            .addPath(BezierLine(redScore, redBottomRowStart))
            .setLinearHeadingInterpolation(redScore.heading, redBottomRowStart.heading)
            .build()

        redBottomStartToBottomEnd = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowStart, redBottomRowEnd))
            .setLinearHeadingInterpolation(redBottomRowStart.heading, redBottomRowEnd.heading)
            .build()

        redBottomEndToScore = follower.pathBuilder()
            .addPath(BezierLine(redBottomRowEnd, redScore))
            .setLinearHeadingInterpolation(redBottomRowEnd.heading, redScore.heading)
            .build()
    }

    buildPaths()
    follower.setStartingPose(startPoseredTop)

    // Autonomous sequence
    schedule(
        sequence(
            exec { pathState = 0 },
            loop(
                exec {
                    follower.update()
                    when (pathState) {
                        0 -> {
                            follower.followPath(scorePreload)
                            outtake.launch()
                            pathState = 1
                        }
                        1 -> if (!follower.isBusy) {
                            follower.followPath(redScoreToTopStart)
                            pathState = 2
                        }
                        2 -> if (!follower.isBusy) {
                            intake.collect()
                            follower.followPath(redTopStartToTopEnd)
                            pathState = 3
                        }
                        3 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redTopEndToScore)
                            outtake.launch()
                            pathState = 4
                        }
                        4 -> if (!follower.isBusy) {
                            follower.followPath(redScoreToMiddleStart)
                            pathState = 5
                        }
                        5 -> if (!follower.isBusy) {
                            intake.collect()
                            follower.followPath(redMiddleStartToMiddleEnd)
                            pathState = 6
                        }
                        6 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redMiddleEndToScore)
                            outtake.launch()
                            pathState = 7
                        }
                        7 -> if (!follower.isBusy) {
                            follower.followPath(redScoreToBottomStart)
                            pathState = 8
                        }
                        8 -> if (!follower.isBusy) {
                            intake.collect()
                            follower.followPath(redBottomStartToBottomEnd)
                            pathState = 9
                        }
                        9 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redBottomEndToScore)
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
