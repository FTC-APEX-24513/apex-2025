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
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem

@Suppress("UNUSED")
val blueBottom3 = Mercurial.autonomous {

    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer
    lateinit var opmodeTimer: Timer

    var pathState = 0
    var actionState = 0

    lateinit var intake: IntakeSubsystem
    lateinit var outtake: OuttakeSubsystem
    lateinit var transfer: TransferSubsystem
    lateinit var spindexer: SpindexerSubsystem
    lateinit var limelight: LimelightSubsystem

    val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
    val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
    val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
    val blueMiddleRowStart = Pose(40.0, 60.0, Math.toRadians(180.0))
    val blueMiddleRowEnd = Pose(20.0, 60.0, Math.toRadians(180.0))
    val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
    val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
    val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0))

    lateinit var blueStartToScore: PathChain
    lateinit var blueScoreToTop: PathChain
    lateinit var blueTopIntake: PathChain
    lateinit var blueTopToScore: PathChain
    lateinit var blueScoreToMiddle: PathChain
    lateinit var blueMiddleIntake: PathChain
    lateinit var blueMiddleToScore: PathChain
    lateinit var blueScoreToBottom: PathChain
    lateinit var blueBottomIntake: PathChain
    lateinit var blueBottomToScore: PathChain

    fun buildPaths() {
        blueStartToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(startPoseBlueBottom, blueScore)))
            .setLinearHeadingInterpolation(startPoseBlueBottom.heading, blueScore.heading)
            .build()

        blueScoreToTop = follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueTopRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueTopRowStart.heading)
            .build()

        blueTopIntake = follower.pathBuilder()
            .addPath(Path(BezierLine(blueTopRowStart, blueTopRowEnd)))
            .setLinearHeadingInterpolation(blueTopRowStart.heading, blueTopRowEnd.heading)
            .build()

        blueTopToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(blueTopRowEnd, blueScore)))
            .setLinearHeadingInterpolation(blueTopRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToMiddle = follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueMiddleRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueMiddleRowStart.heading)
            .build()

        blueMiddleIntake = follower.pathBuilder()
            .addPath(Path(BezierLine(blueMiddleRowStart, blueMiddleRowEnd)))
            .setLinearHeadingInterpolation(blueMiddleRowStart.heading, blueMiddleRowEnd.heading)
            .build()

        blueMiddleToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(blueMiddleRowEnd, blueScore)))
            .setLinearHeadingInterpolation(blueMiddleRowEnd.heading, blueScore.heading)
            .build()

        blueScoreToBottom = follower.pathBuilder()
            .addPath(Path(BezierLine(blueScore, blueBottomRowStart)))
            .setLinearHeadingInterpolation(blueScore.heading, blueBottomRowStart.heading)
            .build()

        blueBottomIntake = follower.pathBuilder()
            .addPath(Path(BezierLine(blueBottomRowStart, blueBottomRowEnd)))
            .setLinearHeadingInterpolation(blueBottomRowStart.heading, blueBottomRowEnd.heading)
            .build()

        blueBottomToScore = follower.pathBuilder()
            .addPath(Path(BezierLine(blueBottomRowEnd, blueScore)))
            .setLinearHeadingInterpolation(blueBottomRowEnd.heading, blueScore.heading)
            .build()
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> { follower.followPath(blueStartToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(1) }
            1 -> if (!follower.isBusy) { follower.followPath(blueScoreToTop); setPathState(2) }
            2 -> if (!follower.isBusy) { intake.collect(); follower.followPath(blueTopIntake); setPathState(3) }
            3 -> if (!follower.isBusy) { intake.stop(); follower.followPath(blueTopToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(4) }
            4 -> if (!follower.isBusy) { follower.followPath(blueScoreToMiddle); setPathState(5) }
            5 -> if (!follower.isBusy) { intake.collect(); follower.followPath(blueMiddleIntake); setPathState(6) }
            6 -> if (!follower.isBusy) { intake.stop(); follower.followPath(blueMiddleToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(7) }
            7 -> if (!follower.isBusy) { follower.followPath(blueScoreToBottom); setPathState(8) }
            8 -> if (!follower.isBusy) { intake.collect(); follower.followPath(blueBottomIntake); setPathState(9) }
            9 -> if (!follower.isBusy) { intake.stop(); follower.followPath(blueBottomToScore); outtake.launch(); limelight.useAprilTagPipeline(); actionTimer.resetTimer(); actionState = 0; setPathState(10) }
            10 -> if (!follower.isBusy) { setPathState(11) }
            11 -> { }
        }
    }

    fun shootingSequence() {
        if (pathState !in 1..10) return
        when (actionState) {
            0 -> if (actionTimer.elapsedTime > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 1 }
            1 -> if (actionTimer.elapsedTime > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 2 }
            2 -> if (actionTimer.elapsedTime > 300) { spindexer.rotateRight(); actionTimer.resetTimer(); actionState = 3 }
            3 -> if (actionTimer.elapsedTime > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 4 }
            4 -> if (actionTimer.elapsedTime > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 5 }
            5 -> if (actionTimer.elapsedTime > 300) { spindexer.rotateRight(); actionTimer.resetTimer(); actionState = 6 }
            6 -> if (actionTimer.elapsedTime > 350) { transfer.transfer(); actionTimer.resetTimer(); actionState = 7 }
            7 -> if (actionTimer.elapsedTime > 600) { outtake.launch(); actionTimer.resetTimer(); actionState = 0 }
        }
    }

    sequence(
        exec {
            pathTimer = Timer()
            actionTimer = Timer()
            opmodeTimer = Timer()

            follower = Constants.createFollower(hardwareMap)

            intake = IntakeSubsystem(hardwareMap)
            outtake = OuttakeSubsystem(hardwareMap)
            transfer = TransferSubsystem(hardwareMap)
            spindexer = SpindexerSubsystem(hardwareMap)
            limelight = LimelightSubsystem(hardwareMap)

            buildPaths()
            follower.setStartingPose(startPoseBlueBottom)
        },
        wait { inLoop },
        exec { setPathState(0) },
        loop(exec {
            follower.update()
            autonomousPathUpdate()
            shootingSequence()
        })
    )

    dropToScheduler()
}