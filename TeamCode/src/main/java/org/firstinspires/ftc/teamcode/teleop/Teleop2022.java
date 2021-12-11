package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrivetrain;


@Config
@TeleOp(name = "Teleop2022")
public class Teleop2022 extends LinearOpMode {
    static boolean dashboardEnabled = true;

    MecanumDrivetrain drive;
    FtcDashboard dashboard;

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;

    DcMotor carousel;
    DcMotor intake;
    DcMotor capstoneLift;
    DcMotor outtakeLift;

    Servo capstoneServo;
    Servo outtakeServo;
    Servo odoY1;
    Servo odoY2;
    Servo odoX;

    double basePower = 0.6;
    double turnPower = 0.2;
    double indivTurnPower = 0.5;

    public void initialize() {
        //Roadrunner Configuration
        drive = new MecanumDrivetrain(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Dashboard Configuration
        if(dashboardEnabled){
            dashboard = FtcDashboard.getInstance();
            dashboard.setTelemetryTransmissionInterval(25);
        }

        //Drive Motors
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

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
    }

    //Vector Combine
    public void vectorCombineRoadrunner(double x, double y, double turn, double indivTurn) {
        double a = y - x;
        double b = y + x;

        double scalar = basePower /  Math.max(Math.abs(a), Math.abs(b));

        if(a!=0 || b!=0) {
            a *= scalar; b *= scalar;
            drive.setMotorPowers(a + turn, b + turn, a - turn, b - turn);
        }else{
            drive.setMotorPowers(indivTurn, indivTurn, -indivTurn, -indivTurn);
        }
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            updateMovement();
            updateGamepadControl();
            updateTelemetry();
        }
    }

    public void updateMovement(){
        double turn = 0.0;
        double indivTurn = 0.0;
        if (gamepad1.left_bumper) {
            turn += turnPower;
            indivTurn += indivTurnPower;
        }
        if (gamepad1.right_bumper){
            turn -= turnPower;
            indivTurn -= indivTurnPower;
        }
        vectorCombineRoadrunner(gamepad1.left_stick_x, gamepad1.left_stick_y, turn, indivTurn);
    }

    public void updateGamepadControl(){
        //gamepad 1
        if(gamepad2.dpad_left) {
            intake.setPower(0.5);
        } else if (gamepad2.dpad_right){
            intake.setPower(-0.5);
        } else {
            intake.setPower(0.0);
        }
        if(gamepad2.x){
            outtakeLift.setPower(0.5);
        } else if (gamepad2.y) {
            outtakeLift.setPower(-0.5);
        } else {
            outtakeLift.setPower(0.0);
        }
        if(gamepad2.dpad_up){
            outtakeServo.setPosition(1.0);
        } else if (gamepad2.dpad_down) {
            outtakeServo.setPosition(0.0);
        }
        //gamepad 1
        if (gamepad1.dpad_up) {
            capstoneLift.setPower(0.5);
        } else if (gamepad1.dpad_down){
            capstoneLift.setPower(-0.5);
        } else {
            capstoneLift.setPower(0.0);
        }
        if (gamepad1.x){
            capstoneServo.setPosition(1.0);
        } else if (gamepad1.y){
            capstoneServo.setPosition(0.0);
        }
        if (gamepad1.dpad_left){
            carousel.setPower(0.5);
        } else if (gamepad1.dpad_right) {
            carousel.setPower(-0.5);
        } else {
            carousel.setPower(0.0);
        }
        //Gamepad 1
//     dpad left = intake back
//                dpad right = intake fwd
//                x = lift up
//                y = lift down
//                dpad up = outtake servo up
//                dpad down = outtake servo down
//
//                dpad up gamepad 1 = capstone up
//                dpad down gamepad 1 = capstone down
//                x game pad 1 = capstoneServo up
//                y game pad 1 = capstoneServo down
        //Gamepad 2

    }

    public void updateTelemetry(){
        //telemetry.addData("odoPose", drive.getPoseEstimate().toString());
        telemetry.update();
        drive.update(); //comment this out during real runs
    }
}
