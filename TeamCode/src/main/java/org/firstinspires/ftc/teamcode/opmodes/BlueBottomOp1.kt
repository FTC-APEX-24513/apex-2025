package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.continuations.Fiber
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.constants.Alliance
import org.firstinspires.ftc.teamcode.constants.RobotConstants
import org.firstinspires.ftc.teamcode.di.HardwareContainer
import org.firstinspires.ftc.teamcode.di.create
import org.firstinspires.ftc.teamcode.physics.ShootingCalculator
import kotlin.math.abs
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem




@Suppress("UNUSED")

private lateinit var follower: Follower
private lateinit var pathTimer: Timer
private lateinit var actionTimer: Timer
private lateinit var opmodeTimer: Timer

private var pathState = 0
private var actionState = 0
// Subsystems
private lateinit var intake: IntakeSubsystem
private lateinit var outtake: OuttakeSubsystem
private lateinit var transfer: TransferSubsystem
private lateinit var spindexer: SpindexerSubsystem
private lateinit var limelight: LimelightSubsystem
// Pose
private val startPoseBlueBottom = Pose(56.0, 16.0, Math.toRadians(180.0))
private val blueTopRowStart = Pose(40.0, 84.0, Math.toRadians(180.0))
private val blueTopRowEnd = Pose(20.0, 84.0, Math.toRadians(180.0))
private val blueMiddleRowStart = Pose(40.0, 60.0, Math.toRadians(180.0))
private val blueMiddleRowEnd = Pose(20.0, 60.0, Math.toRadians(180.0))
private val blueBottomRowStart = Pose(40.0, 36.0, Math.toRadians(180.0))
private val blueBottomRowEnd = Pose(20.0, 36.0, Math.toRadians(180.0))
private val blueScore = Pose(48.0, 48.0, Math.toRadians(180.0)) // tune after testing

// Paths
private lateinit var blueStartToScore: PathChain
private lateinit var blueScoreToTop: PathChain
private lateinit var blueTopIntake: PathChain
private lateinit var blueTopToScore: PathChain
private lateinit var blueScoreToMiddle: PathChain
private lateinit var blueMiddleIntake: PathChain
private lateinit var blueMiddleToScore: PathChain
private lateinit var blueScoreToBottom: PathChain
private lateinit var blueBottomIntake: PathChain
private lateinit var blueBottomToScore: PathChain

private fun buildPaths() {
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
fun followPathBlocking(path: PathChain) = sequence(
    exec { follower.followPath(path) },
    waitUntil { !follower.isBusy }
)

fun shootingLoop() = loop(
    sequence(
        exec { transfer.transfer() },
        wait(0.35),

        exec { outtake.launch() },
        wait(0.6),

        exec { spindexer.switch() },
        wait(0.3),
    )
)

fun aprilTagLoop() = loop(
    exec {
        limelight.useAprilTagPipeline()
    }
)


val blueBottom1 = Mercurial.autonomous {

    buildPaths()
    follower.setStartingPose(startPoseBlueBottom)

    waitForStart()

    schedule(shootingLoop())
    schedule(aprilTagLoop())

    schedule(
        sequence(
            followPathBlocking(blueStartToScore),
            exec { outtake.launch() },

            followPathBlocking(blueScoreToTop),
            exec { intake.collect() },
            followPathBlocking(blueTopIntake),
            exec { intake.stop() },
            followPathBlocking(blueTopToScore),

            followPathBlocking(blueScoreToMiddle),
            exec { intake.collect() },
            followPathBlocking(blueMiddleIntake),
            exec { intake.stop() },
            followPathBlocking(blueMiddleToScore),

            followPathBlocking(blueScoreToBottom),
            exec { intake.collect() },
            followPathBlocking(blueBottomIntake),
            exec { intake.stop() },
            followPathBlocking(blueBottomToScore),
        )
    )

    dropToScheduler()
}
