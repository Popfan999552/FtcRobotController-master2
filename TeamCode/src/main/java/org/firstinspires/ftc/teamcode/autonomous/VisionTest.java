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
import org.firstinspires.ftc.teamcode.drive.opmode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.ApriltagDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous(name = "Signal Sleeve Test")

public class VisionTest extends LinearOpMode {
    DcMotor slides;
    DcMotor futureVirtualFourBar;
    //Intake motors
    DcMotor roller;

    //Outtake servos
    CRServo leftServo;
    CRServo rightServo;
    CRServo bucketServo;

   /* DcMotor slide;
    DcMotor intake;

    Servo v4b;*/
    //TODO CENTERSTAGETELEOP get 1 and 0 positions
    //TODO: CENTERSTAGETELEOP get slides up and down directions and time, hanging direction and time
    //TODO:
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    public ApriltagDetector apriltagDetector;
    public SleeveDetection.ParkingPosition pixelpos;

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

    //apriltag variables
    //OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    //public SleeveDetection.ParkingPosition pixelpos;



    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //drivetrain motors
        drive = new SampleMecanumDrive(hardwareMap);
        apriltagDetector = new ApriltagDetector(this);

        //Outtake motors
        slides = hardwareMap.dcMotor.get("slides");
        //futureVirtualFourBar = hardwareMap.dcMotor.get("futureVirtualFourBar");

        //Outtake servos
        leftServo = hardwareMap.crservo.get("leftServo");
        rightServo = hardwareMap.crservo.get("rightServo");
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

        //SleeveDetection.ParkingPosition pixelpos = sleeveDetection.getPosition();
        /*while (!isStarted()) {



        }*/

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
            sleep(100);

        pixelpos = sleeveDetection.getPosition();

        if(pixelpos== SleeveDetection.ParkingPosition.LEFT){
            drive.followTrajectory(trajToSpike);
            //drive.turn(-90);
            roller.setPower(1);
            drive.followTrajectory(trajSpikeToBb);


            //intake.setPower(0.1);
        } else if(pixelpos== SleeveDetection.ParkingPosition.CENTER){
            drive.followTrajectory(trajToSpike);
            drive.turn(180);
            roller.setPower(1);
            drive.followTrajectory(trajSpikeToBb);
            //intake.setPower(0.1);

        }   else if(pixelpos== SleeveDetection.ParkingPosition.RIGHT){
            drive.followTrajectory(trajToSpike);
            drive.turn(90);
            roller.setPower(1);
            drive.followTrajectory(trajSpikeToBb);
            //intake.setPower(0.1);


        }
        //cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        apriltagDetector.runOpMode();
        AprilTagDetection tagOfInterest = apriltagDetector.setTagOfInterest("blue", pixelpos);
        if (tagOfInterest!=null){
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
        }


        //park in all of them

        telemetry.update();
        }
    }
}