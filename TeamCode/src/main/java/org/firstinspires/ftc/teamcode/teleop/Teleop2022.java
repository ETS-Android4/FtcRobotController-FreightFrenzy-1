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
import org.opencv.core.Mat;

@Config
@TeleOp(name = "Teleop2022")
public class Teleop2022 extends LinearOpMode {
    static boolean dashboardEnabled = true;

    MecanumDrivetrain drive;
    FtcDashboard dashboard;
    DcMotor in_front;
    DcMotor in_back;
    DcMotor wobbleGoal;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;
    DcMotor shooter;
    Servo wobbleGrab;

    double basePower = 0.6;
    double turnPower = 0.2;
    double indivTurnPower = 0.5;

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
        rightBack = hardwareMap.dcMotor.get("rightRear");
        leftBack = hardwareMap.dcMotor.get("leftRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //DC Motors
        in_front = hardwareMap.dcMotor.get("in_front");
        in_back = hardwareMap.dcMotor.get("in_back");
        wobbleGoal = hardwareMap.dcMotor.get("WobbleGoal");
        shooter = hardwareMap.dcMotor.get("Launch");

        //Servos
        wobbleGrab = hardwareMap.servo.get("wobbleGrab");
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
        //Gamepad 1

        //Gamepad 2
        if (gamepad2.left_trigger > 0.1) {
            in_front.setPower(-1);
            in_back.setPower(-1);
        }
        if (gamepad2.right_trigger > 0.1) {
            in_front.setPower(1);
            in_back.setPower(1);
        }
        if (gamepad2.y) {
            wobbleGoal.setPower(1);
        }
        if (gamepad2.x) {
            wobbleGoal.setPower(-1);
        }
        if (gamepad2.right_bumper) {
            shooter.setPower(-0.8);
        }
        if (gamepad2.left_bumper) {
            shooter.setPower(0);
        }
        if (gamepad2.dpad_left) {
            wobbleGrab.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            wobbleGrab.setPosition(1);
        }
    }

    public void updateTelemetry(){
        //telemetry.addData("odoPose", drive.getPoseEstimate().toString());
        telemetry.update();
        drive.update(); //comment this out during real runs
    }
}
