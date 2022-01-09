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

    Servo intakeServo;
    Servo capstoneServo;
    Servo outtakeServo;
    Servo odoY1;
    Servo odoY2;
    Servo odoX;

    double basePower = 0.8;
    double turnPower = 0.2;
    double indivTurnPower = 0.5;

    boolean intakeLock = false;

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
        intakeServo = hardwareMap.servo.get("intakeServo");
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
        vectorCombineRoadrunner(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -turn, -indivTurn);
    }

    public void updateGamepadControl(){
        //TODO: add override of gamepad2 controls for gamepad1
        //^^^ assign it to the right left_stick y to toggle. (+ means gp1, - means gp2)

        //intake
        if(gamepad2.left_bumper) {
            releaseIntakeLock();
            intake.setPower(0.5);
        } else if (gamepad2.right_bumper){
            releaseIntakeLock();
            intake.setPower(-0.5);
        } else {
            intake.setPower(0.0);
        }
        //TODO: create an unsafe context for the lift, where outtake is not repositioned
        if(gamepad2.x){ //down
            repositionOuttake();
            triggerIntakeLock();
            outtakeLift.setPower(-0.3);
        } else if (gamepad2.y) { //up
            secureOuttake();
            triggerIntakeLock();
            outtakeLift.setPower(0.3);
        } else {
            outtakeLift.setPower(0.0);
        }
        //deposit
        if(gamepad2.dpad_up){
            depositOuttake();
        } else if (gamepad2.dpad_down) {
            repositionOuttake();
        } else if (gamepad2.left_stick_button){
            secureOuttake();
        }
        //Capstone
        if (gamepad1.y) {
            capstoneLift.setPower(0.5);
        } else if (gamepad1.x){
            capstoneLift.setPower(-0.5);
        } else {
            capstoneLift.setPower(0.0);
        }
        if (gamepad1.dpad_down){
            capstoneServo.setPosition(0.85);
        } else if (gamepad1.dpad_up){
            capstoneServo.setPosition(0.5);
        }
        //Odometry raise/lower
        if (gamepad1.right_trigger>0.1){ //down
            lowerOdo();
        } else if (gamepad1.left_trigger>0.1) { // up
            raiseOdo();
        }
        //DEBUG
        if (gamepad2.left_trigger>0.1){
            leftFront.setPower(0.5);
        } else if (gamepad2.right_trigger>0.1){
            rightFront.setPower(0.5);
        }
        //Carousel
        if (gamepad1.dpad_left){
            carousel.setPower(0.2);
        } else if (gamepad1.dpad_right) {
            carousel.setPower(-0.2);
        } else {
            carousel.setPower(0.0);
        }

    }

    private void triggerIntakeLock(){
        intakeServo.setPosition(1.0);
        intakeLock = true;
    }
    private void releaseIntakeLock(){
        intakeServo.setPosition(0.0);
        intakeLock = false;
    }
    private void secureOuttake(){
        outtakeServo.setPosition(0.0);
    }
    private void repositionOuttake(){
        outtakeServo.setPosition(0.1);
    }
    private void depositOuttake(){
        outtakeServo.setPosition(0.5);
    }
    //TODO: update with new odo system
    private void raiseOdo(){
        odoX.setPosition(0.5);
        odoY1.setPosition(0.0);
        odoY2.setPosition(1.0);
    }
    private void lowerOdo(){
        odoX.setPosition(0.9);
        odoY1.setPosition(0.96);
        odoY2.setPosition(0.85);
    }

    public void updateTelemetry(){
        //telemetry.addData("odoPose", drive.getPoseEstimate().toString());
        telemetry.update();
        drive.update(); //comment this out during real runs
    }
}
