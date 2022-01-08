package org.firstinspires.ftc.teamcode2021;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opencv.OpenCVHelper;
import org.firstinspires.ftc.teamcode.opencv.UltimateGoalCVHelper;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name = "RingAutonomous_OUTDATED")
public class RingAutonomous extends
        gary {

    private static final String TAG = "RingAutonomous";

    @Override
    public void runOpMode()  {
        OpenCVHelper openCVHelper = new OpenCVHelper();
        openCVHelper.initializeOpenCVAndVuforiaCamera(hardwareMap, "Internal" , FRONT , false);
        UltimateGoalCVHelper ultimateGoalCVHelper = new UltimateGoalCVHelper();
        char ch_result = '?';

        // try once before we click Go
        ch_result = ultimateGoalCVHelper.detectRings(openCVHelper);
        telemetry.addData("Ring Detected", "Path = %s", String.valueOf(ch_result));
        Log.d(TAG, "Ring = " + String.valueOf(ch_result));
        telemetry.update();

        waitForStart();
        Log.d(TAG, "start happened");

        while (opModeIsActive()) {
            ch_result = ultimateGoalCVHelper.detectRings(openCVHelper);
            telemetry.addData("Ring Detected", "Path = %s", String.valueOf(ch_result));
            Log.d(TAG, "Ring = " + String.valueOf(ch_result));
            telemetry.update();
            idle();
        }
        telemetry.addData("Ring Detection Termination", "");
        // close camera
        telemetry.update();
    }
}
