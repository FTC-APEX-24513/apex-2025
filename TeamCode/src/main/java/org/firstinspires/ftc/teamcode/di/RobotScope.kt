package org.firstinspires.ftc.teamcode.di

import software.amazon.lastmile.kotlin.inject.anvil.SingleIn

/**
 * Scope for robot-level dependencies.
 * All subsystems and robot components are singletons within this scope,
 * meaning one instance per OpMode execution.
 */
abstract class RobotScope private constructor()
