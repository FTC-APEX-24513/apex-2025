package org.firstinspires.ftc.teamcode.opmodes.auto

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
val redBottom1 = Mercurial.autonomous {

    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    var pathState = 0

    val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    val redScore = Pose() // define scoring position

    lateinit var scorePreload: Path
    lateinit var redScoreToBottom: PathChain
    lateinit var redBottomIntake: PathChain
    lateinit var redBottomToScore: PathChain

    lateinit var intake: IntakeSubsystem
    lateinit var outtake: OuttakeSubsystem
    lateinit var limelight: LimelightSubsystem
    lateinit var transfer: TransferSubsystem
    lateinit var spindexer: SpindexerSubsystem

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPose, redScore)).apply {
            setLinearHeadingInterpolation(startPose.heading, redScore.heading)
        }

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

    sequence(
        exec {
            pathTimer = Timer()
            follower = Constants.createFollower(hardwareMap)
            buildPaths()
            follower.setStartingPose(startPose)

            intake = IntakeSubsystem(hardwareMap)
            outtake = OuttakeSubsystem(hardwareMap)
            limelight = LimelightSubsystem()
            limelight.init(hardwareMap, telemetry)
            transfer = TransferSubsystem(hardwareMap)
            spindexer = SpindexerSubsystem(hardwareMap)
        },
        wait { inLoop },
        exec { pathState = 0 },
        loop(
            exec {
                follower.update()
                when (pathState) {
                    0 -> {
                        follower.followPath(scorePreload)
                        outtake.outtake()
                        pathState = 1
                    }
                    1 -> if (!follower.isBusy) {
                        follower.followPath(redScoreToBottom)
                        pathState = 2
                    }
                    2 -> if (!follower.isBusy) {
                        intake.intake()
                        follower.followPath(redBottomIntake)
                        pathState = 3
                    }
                    3 -> if (!follower.isBusy) {
                        intake.stop()
                        follower.followPath(redBottomToScore)
                        outtake.outtake()
                        pathState = 4
                    }
                    4 -> if (!follower.isBusy) pathState = 5
                    5 -> { /* Finished */ }
                }
            }
        )
    )

    dropToScheduler()
}
