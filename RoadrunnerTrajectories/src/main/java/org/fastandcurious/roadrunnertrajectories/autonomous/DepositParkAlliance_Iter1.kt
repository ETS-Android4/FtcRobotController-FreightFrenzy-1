package org.fastandcurious.roadrunnertrajectories.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.fastandcurious.roadrunnertrajectories.TrajectoryGen
import kotlin.math.PI

class DepositParkAlliance_Iter1(RedAlliance: Boolean) : TrajectoryGen(RedAlliance) {
    override fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        currentPose = Pose2d(0.4*sqX , 2.5*sqY, 0.0);

        //Detect
        //Deposit
        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(0.4*sqX, 1*sqY, 0.0)
        )) { poses: Array<Pose2d> ->
            initialize(poses[0], angleToPose(poses[0], poses[1]))
                    .splineToSplineHeading(poses[1], angleToPose(poses[0], poses[1]))
                    .build()
        })
        //Park
        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(0.4*sqX, 1.9*sqY, 0.0)
        )){ poses: Array<Pose2d> ->
            initialize(poses[0], PI /2)
                    .splineToConstantHeading(poses[1].vec(), PI /2)
                    .build()
        })
        //Park
        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(1.6*sqX, 1.9*sqY, 0.0)
        )){ poses: Array<Pose2d> ->
            initialize(poses[0], 0.0)
                    .splineToConstantHeading(poses[1].vec(), 0.0)
                    .build()
        })
        return list
    }
}