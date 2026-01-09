package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Suppress("UNUSED")
val blueBottom0 = Mercurial.autonomous {

    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer
    lateinit var opmodeTimer: Timer

    var pathState = 0
    var actionState = 0

    lateinit var intake: IntakeSubsystem
    lateinit var transfer: TransferSubsystem
    lateinit var outtake: OuttakeSubsystem
    lateinit var spindexer: SpindexerSubsystem
    lateinit var limelight: LimelightSubsystem

    val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    lateinit var blueStartToScore: PathChain

    fun buildPaths() {
        blueStartToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseBlueBottom, blueScore)))
            .setLinearHeadingInterpolation(
                startPoseBlueBottom.heading,
                blueScore.heading
            )
            .build()
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun shootingSequence() {
        when (actionState) {
            0 -> if (actionTimer.elapsedTimeSeconds > 0.5) { actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.elapsedTimeSeconds > 0.35) { transfer.transfer(); actionTimer.resetTimer(); actionState = 2 }
            2 -> if (actionTimer.elapsedTimeSeconds > 0.6) { outtake.launch(); actionTimer.resetTimer(); actionState = 3 }
            3 -> if (actionTimer.elapsedTimeSeconds > 0.3) { spindexer.switch(); actionTimer.resetTimer(); actionState = 4 }
            4 -> if (actionTimer.elapsedTimeSeconds > 0.35) { transfer.transfer(); actionTimer.resetTimer(); actionState = 5 }
            5 -> if (actionTimer.elapsedTimeSeconds > 0.6) { outtake.launch(); actionTimer.resetTimer(); actionState = 6 }
            6 -> if (actionTimer.elapsedTimeSeconds > 0.3) { spindexer.switch(); actionTimer.resetTimer(); actionState = 7 }
            7 -> if (actionTimer.elapsedTimeSeconds > 0.35) { transfer.transfer(); actionTimer.resetTimer(); actionState = 8 }
            8 -> if (actionTimer.elapsedTimeSeconds > 0.6) { outtake.launch(); setPathState(3) }
        }
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                intake.collect()
                follower.followPath(blueStartToScore)
                setPathState(1)
            }
            1 -> if (!follower.isBusy) {
                intake.stop()
                limelight.useAprilTagPipeline()
                actionTimer.resetTimer()
                actionState = 0
                setPathState(2)
            }
            2 -> shootingSequence()
            3 -> { }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()
            actionTimer = Timer()
            opmodeTimer = Timer()

            follower = Constants.createFollower(hardwareMap)

            intake = IntakeSubsystem(hardwareMap)
            transfer = TransferSubsystem(hardwareMap)
            outtake = OuttakeSubsystem(hardwareMap)
            spindexer = SpindexerSubsystem(hardwareMap)
            limelight = LimelightSubsystem(hardwareMap)

            buildPaths()
            follower.setStartingPose(startPoseBlueBottom)
        },
        wait { inLoop },
        exec {
            setPathState(0)
        },
        loop(exec {
            follower.update()
            autonomousPathUpdate()
        })
    )

    dropToScheduler()
}
