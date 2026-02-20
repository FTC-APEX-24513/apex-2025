package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Autonomous(name = "RedTop", group = "Autonomous")
@Configurable // Panels
class PedroAutonomous : OpMode() {
    private var panelsTelemetry: TelemetryManager? = null // Panels Telemetry instance
    var follower: Follower? = null // Pedro Pathing follower instance
    private var pathState = 0 // Current autonomous path state (state machine)
    private var paths: Paths? = null // Paths defined in the Paths class

    override fun init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry()

        follower = Constants.createFollower(hardwareMap)
        follower!!.setStartingPose(Pose(72.0, 8.0, Math.toRadians(90.0)))

        paths = PedroAutonomous.Paths(follower!!) // Build paths

        panelsTelemetry.debug("Status", "Initialized")
        panelsTelemetry.update(telemetry)
    }

    override fun loop() {
        follower!!.update() // Update Pedro Pathing
        pathState = autonomousPathUpdate() // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState)
        panelsTelemetry.debug("X", follower!!.getPose().getX())
        panelsTelemetry.debug("Y", follower!!.getPose().getY())
        panelsTelemetry.debug("Heading", follower!!.getPose().getHeading())
        panelsTelemetry.update(telemetry)
    }


    class Paths(follower: Follower) {
        var Path1: PathChain?
        var Path2: PathChain?
        var Path3: PathChain?
        var Path4: PathChain?
        var Path5: PathChain?
        var Path6: PathChain?
        var Path7: PathChain?
        var Path8: PathChain?
        var Path9: PathChain?
        var Path10: PathChain?
        var Path11: PathChain?
        var Path12: PathChain?
        var Path13: PathChain?

        init {
            Path1 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(122.212, 124.987),

                    Pose(86.000, 86.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(35.0), Math.toRadians(45.0))

                .build()

            Path2 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(86.000, 86.000),

                    Pose(100.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

                .build()

            Path3 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(100.000, 60.000),

                    Pose(123.000, 60.000)
                )
            ).setTangentHeadingInterpolation()

                .build()

            Path4 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(123.000, 60.000),

                    Pose(86.000, 86.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

                .build()

            Path5 = follower.pathBuilder().addPath(
                BezierCurve(
                    Pose(86.000, 86.000),
                    Pose(106.712, 61.293),
                    Pose(134.298, 59.882)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(30.0))

                .build()

            Path6 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(134.298, 59.882),

                    Pose(86.000, 86.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(30.0), Math.toRadians(45.0))

                .build()

            Path7 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(86.000, 86.000),

                    Pose(100.000, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

                .build()

            Path8 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(100.000, 84.000),

                    Pose(123.000, 84.000)
                )
            ).setTangentHeadingInterpolation()

                .build()

            Path9 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(123.000, 84.000),

                    Pose(86.000, 86.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

                .build()

            Path10 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(86.000, 86.000),

                    Pose(100.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(0.0))

                .build()

            Path11 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(100.000, 36.000),

                    Pose(123.000, 36.000)
                )
            ).setTangentHeadingInterpolation()

                .build()

            Path12 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(123.000, 36.000),

                    Pose(86.000, 86.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(45.0))

                .build()

            Path13 = follower.pathBuilder().addPath(
                BezierLine(
                    Pose(86.000, 86.000),

                    Pose(94.361, 77.402)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(45.0), Math.toRadians(45.0))

                .build()
        }
    }


    fun autonomousPathUpdate(): Int {
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState
    }
}