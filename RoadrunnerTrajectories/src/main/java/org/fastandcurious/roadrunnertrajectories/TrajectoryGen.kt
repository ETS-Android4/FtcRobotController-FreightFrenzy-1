package org.fastandcurious.roadrunnertrajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

import kotlin.math.PI

abstract class TrajectoryGen (RedAlliance: Boolean){
    private val FP = if (RedAlliance) {-1} else {1} //Field Parity
    val sqX = 24.0
    val sqY = sqX * FP
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(50.0, 50.0, 0.0, 180.0.toRadians, 180.0.toRadians, 0.0)
    lateinit var currentPose : Pose2d
    fun initialize(startPose: Pose2d, startHeading: Double) : TrajectoryBuilder{
        return TrajectoryBuilder(startPose, startHeading, driveConstraints)
    }
    fun createTraj(poses: Array<Pose2d>, buildTraj: (Array<Pose2d>) -> Trajectory) : Trajectory{
        currentPose = poses.last();
        return buildTraj(poses)
    }
    fun angleToPose(a: Pose2d, b: Pose2d): Double{
        return (b.vec()-a.vec()).angle();
    }

    val Double.toRadians get() = (Math.toRadians(this))

    abstract fun createTrajectory(): ArrayList<Trajectory>
}


