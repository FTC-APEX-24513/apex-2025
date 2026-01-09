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
val blueTop2 = Mercurial.autonomous {

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

    val startPoseBlueTop = Pose(56.0, 88.0, Math.toRadians(180.0))
    val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
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
                pathState = 5
            }
            5 -> { }
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
        loop(exec {
            follower.update()
            autonomousPathUpdate()
        })
    )

    dropToScheduler()
}
