package org.firstinspires.ftc.teamcode.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence
import dev.frozenmilk.dairy.mercurial.continuations.Continuations.wait
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem

@Suppress("UNUSED")
val redBottom0 = Mercurial.autonomous {

    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    var pathState = 0

    val startPose = Pose(88.0, 16.0, Math.toRadians(180.0))
    val redScore = Pose() // define scoring position

    lateinit var redStartToScore: PathChain

    lateinit var outtake: OuttakeSubsystem
    lateinit var limelight: LimelightSubsystem

    fun buildPaths() {
        redStartToScore = follower.pathBuilder()
            .addPath(BezierLine(startPose, redScore))
            .setLinearHeadingInterpolation(startPose.heading, redScore.heading)
            .build()
    }

    sequence(
        exec {
            pathTimer = Timer()
            follower = Constants.createFollower(hardwareMap)
            buildPaths()
            follower.setStartingPose(startPose)

            outtake = OuttakeSubsystem(hardwareMap)
            limelight = LimelightSubsystem()
            limelight.init(hardwareMap, telemetry)
        },
        wait { inLoop },
        exec { pathState = 0 },
        loop(
            exec {
                follower.update()
                when (pathState) {
                    0 -> {
                        follower.followPath(redStartToScore)
                        outtake.outtake()
                        pathState = 1
                    }
                    1 -> if (!follower.isBusy) pathState = 2
                    2 -> { /* Finished */ }
                }
            }
        )
    )

    dropToScheduler()
}
