package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.teleop.Teleop2022;

@Autonomous(name = "ParkRed")
public class ParkRed extends Teleop2022 {
    @Override
    public void runOpMode(){
        MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);

        drive.setPoseEstimate(new Pose2d());

        waitForStart();

        Servo odoX = hardwareMap.servo.get("odoX");
        odoX.setPosition(0.6); //lowers back odo //scuffed

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(20.0)
                .build();

        drive.followTrajectory(traj);
    }
}