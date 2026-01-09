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
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem

@Suppress("UNUSED")
val redTop1 = Mercurial.autonomous {
    // subsystems
    val intake = IntakeSubsystem(hardwareMap)
    val outtake = OuttakeSubsystem(hardwareMap)
    val limelight = LimelightSubsystem()
    limelight.init(hardwareMap, telemetry)
    val transfer = TransferSubsystem(hardwareMap)
    val spindexer = SpindexerSubsystem(hardwareMap)

    // pathing
    val follower: Follower = Constants.createFollower(hardwareMap)
    val pathTimer = Timer()
    var pathState = 0

    val startPoseRedTop = Pose(88.0, 88.0, Math.toRadians(180.0))
    val startPoseRedBottom = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redTopRowStart = Pose(104.0, 84.0, Math.toRadians(90.0))
    val redTopRowEnd = Pose(124.0, 84.0, Math.toRadians(90.0))
    val redMiddleRowStart = Pose(104.0, 60.0, Math.toRadians(180.0))
    val redMiddleRowEnd = Pose(124.0, 60.0, Math.toRadians(180.0))
    val redBottomRowStart = Pose(104.0, 36.0, Math.toRadians(180.0))
    val redBottomRowEnd = Pose(124.0, 36.0, Math.toRadians(180.0))
    val redScore = Pose() // as in original

    lateinit var scorePreload: Path
    lateinit var redScoreToTop: PathChain
    lateinit var redTopIntake: PathChain
    lateinit var redTopToScore: PathChain

    fun buildPaths() {
        scorePreload = Path(BezierLine(startPoseRedTop, redScore)).apply {
            setLinearHeadingInterpolation(startPoseRedTop.heading, redScore.heading)
        }

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
    }

    buildPaths()
    follower.setStartingPose(startPoseRedTop)

    // main autonomous sequence
    schedule(
        sequence(
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
                            follower.followPath(redScoreToTop)
                            pathState = 2
                        }
                        2 -> if (!follower.isBusy) {
                            intake.intake()
                            follower.followPath(redTopIntake)
                            pathState = 3
                        }
                        3 -> if (!follower.isBusy) {
                            intake.stop()
                            follower.followPath(redTopToScore)
                            outtake.outtake()
                            pathState = 4
                        }
                        4 -> if (!follower.isBusy) pathState = 5
                        5 -> { /* Finished */ }
                    }
                }
            )
        )
    )

    dropToScheduler()
}
