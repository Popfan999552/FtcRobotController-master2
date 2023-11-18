package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.CENTERSTAGEROBOT;
import org.firstinspires.ftc.teamcode.drive.opmode.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous(name = "Signal Sleeve Test")
public class VisionTest extends LinearOpMode {

    CENTERSTAGEROBOT drive;
   /* DcMotor slide;
    DcMotor roller;

    Servo v4b;*/

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;


    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";
    public boolean stop = false;
    //rr stuff
    /*Trajectory trajToSpike = drive.trajectoryBuilder(new Pose2d())
            .forward(2d)
            .build();*/

    @Override
    public void runOpMode() throws InterruptedException {
        //hardware map
        drive = new CENTERSTAGEROBOT(hardwareMap);
        /*slide=hardwareMap.dcMotor.get("slide");
        v4b=hardwareMap.servo.get("v4b");
        roller=hardwareMap.dcMotor.get("roller");*/


        //cv
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        
        //v4b.setPosition(0);
        //slide.setPower(.2);
        sleep(1000);
        telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
        SleeveDetection.ParkingPosition pixelpos = sleeveDetection.getPosition();
        /*while (!isStarted()) {



        }*/

        waitForStart();
        if(pixelpos== SleeveDetection.ParkingPosition.LEFT){

            //drive.followTrajectory(trajToSpike);
            //intake.setPower(0.1);
        } else if(pixelpos== SleeveDetection.ParkingPosition.CENTER){
            //drive.followTrajectory(trajToSpike);
            //intake.setPower(0.1);

        }   else if(pixelpos== SleeveDetection.ParkingPosition.RIGHT){
            //drive.followTrajectory(trajToSpike);
            //intake.setPower(0.1);
        }
        //park in all of them

        telemetry.update();
    }
}