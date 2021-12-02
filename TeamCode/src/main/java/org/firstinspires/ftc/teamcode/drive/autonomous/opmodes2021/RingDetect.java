package org.firstinspires.ftc.teamcode.drive.autonomous.opmodes2021;

//@Autonomous
public class RingDetect extends Autonomous2021{
    @Override
    public void runOpMode() throws InterruptedException {
        initializeRobot();

        waitForStart();

        int c = -1;
        while (opModeIsActive()){
                c = getRingState();
                drive.forceSendTelemetryToDashboard("RingState", (double) c);
                sleep(250);
        }
        drive.setMotorPowers(0,0,0,0);
    }
}
