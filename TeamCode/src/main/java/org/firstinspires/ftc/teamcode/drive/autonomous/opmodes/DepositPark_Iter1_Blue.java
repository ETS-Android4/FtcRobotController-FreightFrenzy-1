package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.fastandcurious.roadrunnertrajectories.autonomous.DepositPark_Iter1;

import org.firstinspires.ftc.teamcode.drive.autonomous.Autonomous2022;


import java.util.List;

@Autonomous(name = "DepositPark_Iter1_Blue")
public class DepositPark_Iter1_Blue extends Autonomous2022 {
    @Override
    public void auto(){
        lowerOdometry();
        List<Trajectory> trajectories = new DepositPark_Iter1(false).build();
        detectState();
        drive.followTrajectory(trajectories.get(0)); //Go to Carousel
        dropDuck();
        drive.followTrajectory(trajectories.get(1)); //Go to Deposit
        depositFreight();
        raiseOdometry();
        drive.followTrajectory(trajectories.get(2)); //Park
    }
}
