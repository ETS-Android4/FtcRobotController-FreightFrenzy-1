package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.opencv.OpenCVHelper;
import org.firstinspires.ftc.teamcode.opencv.UltimateGoalCVHelper;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Config
public abstract class Autonomous2022 extends LinearOpMode {
    //Configurables
    public static int visionEnabled = 1; //Stand in for boolean

    //OpenCV
    OpenCVHelper helper;
    UltimateGoalCVHelper vision;

    // Motors
    DcMotor carousel;
    DcMotor intake;
    DcMotor capstoneLift;
    DcMotor outtakeLift;

    // Servos
    Servo capstoneServo;
    Servo outtakeServo;
    Servo odoY1;
    Servo odoY2;
    Servo odoX;

    //Roadrunner
    protected MecanumDrivetrain drive;

    //Field state detected through CV
    int state = 2;

    void initializeRobot(){
        if (visionEnabled == 1){
            helper  = new OpenCVHelper();
            helper.initializeOpenCVAndVuforiaCamera(hardwareMap, "Internal" , FRONT , false);
            vision = new UltimateGoalCVHelper();
        }

        drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //DC Motors
        intake = hardwareMap.dcMotor.get("intake");
        carousel = hardwareMap.dcMotor.get("carousel");
        outtakeLift = hardwareMap.dcMotor.get("outtakeLift");
        capstoneLift = hardwareMap.dcMotor.get("capstoneLift");

        //Servos
        outtakeServo = hardwareMap.servo.get("outtakeServo");
        capstoneServo = hardwareMap.servo.get("capstoneServo");
        odoY1 = hardwareMap.servo.get("odoY1");
        odoY2 = hardwareMap.servo.get("odoY2");
        odoX = hardwareMap.servo.get("odoX");


        waitForStart();

        if (isStopRequested()) return;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Configure per autonomous path
        //Make sure to set the pose estimate
        initializeRobot();
        auto();
        drive.setMotorPowers(0,0,0,0);
    }

    public abstract void auto();

    protected void detectState(){
        //OpenCV
    }
    protected void turnOnIntake(){
        intake.setPower(0.7);
    }
    protected void turnOffIntake(){
        intake.setPower(-0.7);
    }
    protected void dropDuck(){
        carousel.setPower(0.6);
        sleep(3500);
        carousel.setPower(0.0);
    }
    protected void depositFreight(){
        outtakeLift.setPower(-0.7);
        sleep(4000);
        outtakeLift.setPower(0.0);
        outtakeServo.setPosition(1.0);
        sleep(1000);
        outtakeLift.setPower(0.7);
        sleep(3000);
    }
    protected void raiseOdometry(){
        odoX.setPosition(1.0);
        odoY1.setPosition(1.0);
        odoY2.setPosition(1.0);
    }
    protected void lowerOdometry(){
        odoX.setPosition(0.0);
        odoY1.setPosition(0.0);
        odoY2.setPosition(0.0);
    }
}