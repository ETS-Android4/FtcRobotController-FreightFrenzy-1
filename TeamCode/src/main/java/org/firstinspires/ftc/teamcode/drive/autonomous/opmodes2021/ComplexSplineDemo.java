package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes2021;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ComplexSplineDemo extends Autonomous2021 {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();
        drive.setPoseEstimate(new Pose2d());
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(2*sq, sq), 0.0)
                .splineTo(new Vector2d(2*sq, 0.0), -Math.PI/2)
                .splineToSplineHeading(new Pose2d(), 0.0)
                .build();

        drive.followTrajectory(traj);

        drive.setMotorPowers(0,0,0,0);
    }
}

