package org.fastandcurious.roadrunnertrajectories.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.fastandcurious.roadrunnertrajectories.TrajectoryGen
import kotlin.math.PI

class Park_Iter1(RedAlliance: Boolean) : TrajectoryGen(RedAlliance) {
    override fun createTrajectory(): ArrayList<Trajectory> {
        var list = ArrayList<Trajectory>();
        currentPose = Pose2d(0.0, 0.0, 0.0);

        list.add(createTraj(arrayOf(
                currentPose,
                Pose2d(0.0, -2*sqY)
        )) { poses: Array<Pose2d> ->
            initialize(poses[0], -PI / 2)
                    .lineToConstantHeading(poses[1].vec())
                    .build()
        })
        return list;
    }
}