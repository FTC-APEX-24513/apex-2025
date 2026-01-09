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
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Suppress("UNUSED")
val blueTop3 = Mercurial.autonomous {

    lateinit var follower: Follower
    lateinit var pathTimer: Timer

    var pathState = 0

    lateinit var intake: IntakeSubsystem
    lateinit var outtake: OuttakeSubsystem
    lateinit var transfer: TransferSubsystem
    lateinit var spindexer: SpindexerSubsystem
    lateinit var limelight: LimelightSubsystem

    lateinit var scorePreload: Path
    lateinit var blueScoreToTopStart: PathChain
    lateinit var blueTopStartToTopEnd: PathChain
    lateinit var blueTopEndToScore: PathChain
    lateinit var blueScoreToMiddleStart: PathChain
    lateinit var blueMiddleStartToMiddleEnd: PathChain
    lateinit var blueMiddleEndToScore: PathChain
    lateinit var blueScoreToBottomStart: PathChain
    lateinit var blueBottomStartToBottomEnd: PathChain
    lateinit var blueBottomEndToScore: PathChain

    val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
    val blueMiddleRowStart = Pose(40.0, 60.0, Math.toRadians(180.0))
    val blueMiddleRowEnd = Pose(20.0, 60.0, Math.toRadians(180.0))
    val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseBlueTop, blueScore))
        scorePreload.setLinearHeadingInterpolation(
            startPoseBlueTop.heading,
            blueScore.heading
        )

        blueScoreToTopStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueTopRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueTopRowStart.heading)
            .build()

        blueTopStartToTopEnd = follower.pathBuilder()
            .addPath(BezierLine(blueTopRowStart, blueTopRowEnd))
            .setLinearHeadingInterpolation(blueTopRowStart.heading, blueTopRowEnd.heading)
            .build()

        blueTopEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueTopRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueTopRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToMiddleStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueMiddleRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueMiddleRowStart.heading)
            .build()

        blueMiddleStartToMiddleEnd = follower.pathBuilder()
            .addPath(BezierLine(blueMiddleRowStart, blueMiddleRowEnd))
            .setLinearHeadingInterpolation(blueMiddleRowStart.heading, blueMiddleRowEnd.heading)
            .build()

        blueMiddleEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueMiddleRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueMiddleRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToBottomStart = follower.pathBuilder()
            .addPath(BezierLine(blueScore, blueBottomRowStart))
            .setLinearHeadingInterpolation(blueScore.heading, blueBottomRowStart.heading)
            .build()

        blueBottomStartToBottomEnd = follower.pathBuilder()
            .addPath(BezierLine(blueBottomRowStart, blueBottomRowEnd))
            .setLinearHeadingInterpolation(blueBottomRowStart.heading, blueBottomRowEnd.heading)
            .build()

        blueBottomEndToScore = follower.pathBuilder()
            .addPath(BezierLine(blueBottomRowEnd, blueScore))
            .setLinearHeadingInterpolation(blueBottomRowEnd.heading, blueScore.heading)
            .build()
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(scorePreload)
                outtake.launch()
                pathState = 1
            }
            1 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToTopStart)
                pathState = 2
            }
            2 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueTopStartToTopEnd)
                pathState = 3
            }
            3 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueTopEndToScore)
                outtake.launch()
                pathState = 4
            }
            4 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToMiddleStart)
                pathState = 5
            }
            5 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueMiddleStartToMiddleEnd)
                pathState = 6
            }
            6 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueMiddleEndToScore)
                outtake.launch()
                pathState = 7
            }
            7 -> if (!follower.isBusy) {
                follower.followPath(blueScoreToBottomStart)
                pathState = 8
            }
            8 -> if (!follower.isBusy) {
                intake.collect()
                follower.followPath(blueBottomStartToBottomEnd)
                pathState = 9
            }
            9 -> if (!follower.isBusy) {
                intake.stop()
                follower.followPath(blueBottomEndToScore)
                outtake.launch()
                pathState = 10
            }
            10 -> if (!follower.isBusy) {
                pathState = 11
            }
            11 -> { }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()
            follower = Constants.createFollower(hardwareMap)

            buildPaths()
            follower.setStartingPose(startPoseBlueTop)

            intake = IntakeSubsystem(hardwareMap)
            outtake = OuttakeSubsystem(hardwareMap)
            transfer = TransferSubsystem(hardwareMap)
            spindexer = SpindexerSubsystem(hardwareMap)
            limelight = LimelightSubsystem(hardwareMap)
        },
        wait { inLoop },
        exec { setPathState(0) },
        loop(
            exec {
                follower.update()
                autonomousPathUpdate()
            }
        )
    )

    dropToScheduler()
}
