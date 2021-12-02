package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes2021;

import com.acmerobotics.roadrunner.geometry.Vector2d;

//@Autonomous (name = "Park_OUTDATED")
public class Park extends Autonomous2021{
    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        drive.followTrajectory(drive.trajectoryBuilder(startPose)
        .lineTo(new Vector2d(sq/2, startPose.getY()))
        .build());

        drive.setMotorPowers(0,0,0,0);
    }
}
