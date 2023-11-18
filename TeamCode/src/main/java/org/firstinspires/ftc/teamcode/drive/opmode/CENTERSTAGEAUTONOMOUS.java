package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.CENTERSTAGEROBOT;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.AprilTagAutonomousInitDetectionExample;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.net.IDN;
import java.util.ArrayList;
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class CENTERSTAGEAUTONOMOUS extends LinearOpMode {
    public static double d = 11.5; // in
    //public static double ANGLE = 90; // deg

    //AprilTagAutonomousInitDetectionExample csDetector= new AprilTagAutonomousInitDetectionExample();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;





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

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 16 from the 36h11 family
    int Left = 17;
    int Middle = 18;
    int Right =19;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);  CENTERSTAGEMECHANUMDRIVE

        CENTERSTAGEROBOT drive = new CENTERSTAGEROBOT(hardwareMap);
        telemetry.addData("finalX", 3);
        Trajectory trajToSpike = drive.trajectoryBuilder(new Pose2d())
                .forward(2d)
                .build();
        Trajectory trajSpikeToBb = drive.trajectoryBuilder(new Pose2d())
                .forward(10d)
                .build();
        Trajectory right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(3)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(3)
                .build();
        telemetry.addData("finalX", 2343);
        opencvPixelDetector pixelDetector = new opencvPixelDetector();


        while(!isStopRequested()){
            pixelDetector.runOpMode();

        }

        //camRunOpMode(drive);
        //drive.followTrajectory();
        drive.followTrajectory(trajToSpike);
        //detect object and location
        //drive.turn(90);
        //drive.followTrajectory(trajSpikeToBb);
        //detect AprilTag
        //move 6'' left or right

        //drive.setMotorPower(gamepad1.right_stick_y);
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

    }










    public void camRunOpMode(CENTERSTAGEROBOT drive)
    {

        waitForStart();
        telemetry.addData("reached",5);
        telemetry.update();
        //Servo servo = hardwareMap.servo.get("servoName");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                telemetry.addData("reached",1);
                telemetry.update();
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        //while (!isStarted() && !isStopRequested())
        //{
        boolean stop=false;
        while (!isStopRequested() && !stop)
        {
            telemetry.addData("reached",1);
            telemetry.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();



            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagOfInterest==null ) {
                    telemetry.addData("hi",3);
                    telemetry.update();
                }
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    stop=true;
                    break;

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        telemetry.addData("hihih",1234);
        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();

        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        telemetry.addData("hi",1);

        if(tagOfInterest==null || tagOfInterest.id == Left) {

            //left code
        }else if (tagOfInterest.id == Middle){
            //middle code
        }else if (tagOfInterest.id == Right){
            //right code
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if(tagOfInterest.pose.x <= 20)
            {
                telemetry.addData("less than 20", 3);
                //servo.setPosition(0);
                return;
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
            {
                //servo.setPosition(90);
                telemetry.addData("between 20 and 50",3);

                return;

                // do something else
            }
            else if(tagOfInterest.pose.x >= 50)
            {
                telemetry.addData("over 50",3);
                //servo.setPosition(180);
                return;

                // do something else
            }
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        //Servo servo = hardwareMap.servo.get("servoName");
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));

    }






}

