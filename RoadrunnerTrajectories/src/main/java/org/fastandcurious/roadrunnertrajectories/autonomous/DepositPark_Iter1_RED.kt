package org.fastandcurious.roadrunnertrajectories.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.fastandcurious.roadrunnertrajectories.TrajectoryGen
import kotlin.math.PI

class DepositPark_Iter1_RED(RedAlliance: Boolean) : TrajectoryGen(RedAlliance){
    override fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        currentPose = Pose2d(-1.5*sqX , 2.5*sqY, 0.0);

        //Detect
        //Carousel
        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(-2.1*sqX, 2.5*sqY)
        )) { poses: Array<Pose2d> ->
            initialize(poses[0], -PI / 2)
                    .lineToConstantHeading(poses[1].vec())
                    .build()
        })
        //Deposit
        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(-1*sqX, 2*sqY, 0.0),
                Pose2d(-0.5*sqX, 1.8*sqY, PI /2)
        )) { poses: Array<Pose2d> ->
            initialize(poses[0], angleToPose(poses[0], poses[1]))
                    .splineToSplineHeading(poses[1], angleToPose(poses[0], poses[1]))
                    .splineToSplineHeading(poses[2], angleToPose(poses[1], poses[2]))
                    .build()
        })
        //Park
        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(0.5*sqX, 1.9*sqY, 0.0),
                Pose2d(1.6*sqX, 1.9*sqY, 0.0)
        )){ poses: Array<Pose2d> ->
            initialize(poses[0], PI /4)
                    .splineToSplineHeading(poses[1], 0.0)
                    .splineToSplineHeading(poses[2], 0.0)
                    .build()
        })
        return list
    }
}