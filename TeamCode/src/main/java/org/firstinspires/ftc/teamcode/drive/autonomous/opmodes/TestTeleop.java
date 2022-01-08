package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.autonomous.Autonomous2022;

@TeleOp
public class TestTeleop extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor intake;
        intake = hardwareMap.dcMotor.get("intake");
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake.setPower(0.5);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-0.5);
            } else {
                intake.setPower(0);
            }
        }
    }
}
