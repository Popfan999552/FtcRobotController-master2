package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.CENTERSTAGEROBOT;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.SleeveDetection;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.AprilTagAutonomousInitDetectionExample;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.ApriltagDetector;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.BlueApriltagDetector;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.BlueSleeveDetection;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous(name = "ShortBlueAuto")

public class ShortBlueAuto extends LinearOpMode {

   /* DcMotor slide;
    DcMotor intake;

    Servo v4b;*/
    DcMotor slides;
    DcMotor futureVirtualFourBar;
    //Intake motors
    DcMotor roller;
    //endgame motor
    DcMotor futureLeadScrew;
    DcMotor winch1;
    DcMotor winch2;
    //Outtake servos
    Servo leftServo;
    Servo rightServo;
    CRServo bucketServo;
    private BlueSleeveDetection blueSleeveDetection;
    private OpenCvCamera camera;
    public BlueApriltagDetector apriltagDetector;
    public BlueSleeveDetection.ParkingPosition pixelpos;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";
    public boolean stop = false;
    //rr stuff
    SampleMecanumDrive drive;
    Trajectory trajToSpike;
    Trajectory trajSpikeToBb;
    Trajectory right;
    Trajectory left;
    double d = 11.5; // in
    boolean Apriltag5PixAway;


    @Override
    public void runOpMode() throws InterruptedException {
        //Outtake motors
        slides = hardwareMap.dcMotor.get("slides");
        //futureVirtualFourBar = hardwareMap.dcMotor.get("futureVirtualFourBar");

        //Outtake servos
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        bucketServo = hardwareMap.crservo.get("bucketServo");

        //Intake motors
        roller = hardwareMap.dcMotor.get("roller");
        right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(1)
                .build();
        left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(1)
                .build();
        //hardware map
        drive = new SampleMecanumDrive(hardwareMap);


        trajToSpike = drive.trajectoryBuilder(new Pose2d())
                .forward(2d+4)
                .build();
        trajSpikeToBb = drive.trajectoryBuilder(new Pose2d())
                .forward(2d)
                .build();

        /*slide=hardwareMap.dcMotor.get("slide");
        v4b=hardwareMap.servo.get("v4b");
        roller=hardwareMap.dcMotor.get("roller");*/


        //cv
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        blueSleeveDetection = new BlueSleeveDetection();
        camera.setPipeline(blueSleeveDetection);

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
        telemetry.addData("ROTATION: ", blueSleeveDetection.getPosition());
        drive.followTrajectory(trajToSpike);
        //SleeveDetection.ParkingPosition pixelpos = sleeveDetection.getPosition();
        /*while (!isStarted()) {



        }*/

        waitForStart();
        //while (opModeIsActive()) {
        telemetry.addData("ROTATION: ", blueSleeveDetection.getPosition());
        telemetry.update();
        sleep(100);
        //}
        pixelpos = blueSleeveDetection.getPosition();

        if(pixelpos== BlueSleeveDetection.ParkingPosition.LEFT){

            drive.followTrajectory(trajToSpike);
            drive.turn(-90);
            //intake.setPower(0.1);
            drive.turn(-180);

            drive.followTrajectory(trajSpikeToBb);

        } else if(pixelpos== BlueSleeveDetection.ParkingPosition.CENTER){
            drive.followTrajectory(trajToSpike);
            drive.turn(180);
            //intake.setPower(0.1);
            drive.turn(90);
            drive.followTrajectory(trajSpikeToBb);

        }   else if(pixelpos== BlueSleeveDetection.ParkingPosition.RIGHT){
            drive.followTrajectory(trajToSpike);
            drive.turn(90);
            //intake.setPower(0.1);
            drive.followTrajectory(trajSpikeToBb);

        }
        apriltagDetector.runOpMode();
        AprilTagDetection tagOfInterest = apriltagDetector.setTagOfInterest("blue", pixelpos);
        double xposition=apriltagDetector.getTranslationX(tagOfInterest);
        while(Math.abs(xposition-180)<5){
            if(xposition>=180){
                drive.followTrajectory(left);
            }else if (xposition<=180){
                drive.followTrajectory(right);
            } else{
                break;
            }
        }
        slides.setPower(1);
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        sleep(1000);
        bucketServo.setPower(1);
        //park in all of them

        telemetry.update();
    }
}