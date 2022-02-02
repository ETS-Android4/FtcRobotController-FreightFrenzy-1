package org.firstinspires.ftc.teamcode.drive.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = -90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);

        waitForStart();

        Servo odoX = hardwareMap.servo.get("odoX");
        odoX.setPosition(0.6); //lowers back odo //scuffed

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
