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

package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.opmode.SleeveDetection;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp
public class ApriltagDetector //extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public SleeveDetection.ParkingPosition pixelpos;



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

    HardwareMap myHardwareMap;
    LinearOpMode myLinearOpMode;
    public ApriltagDetector(LinearOpMode linearOpMode){
        myLinearOpMode=linearOpMode;
        myHardwareMap=linearOpMode.hardwareMap;

    }
    public AprilTagDetection setTagOfInterest(String color, SleeveDetection.ParkingPosition pixelpos) {

        if(pixelpos==SleeveDetection.ParkingPosition.RIGHT && color == "blue") {
            ID_TAG_OF_INTEREST=1;

        } else if (pixelpos==SleeveDetection.ParkingPosition.CENTER && color == "blue"){
            ID_TAG_OF_INTEREST=2;
        } else{
            ID_TAG_OF_INTEREST=3;

        }
        if(pixelpos==SleeveDetection.ParkingPosition.RIGHT && color == "red") {
            ID_TAG_OF_INTEREST=4;
        } else if (pixelpos==SleeveDetection.ParkingPosition.CENTER && color == "red"){
            ID_TAG_OF_INTEREST=5;
        } else{
            ID_TAG_OF_INTEREST=6;

        }
        while (!myLinearOpMode.isStarted() && !myLinearOpMode.isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    myLinearOpMode.telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    return tagOfInterest;
                }
                else
                {
                    myLinearOpMode.telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        myLinearOpMode.telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        myLinearOpMode.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                myLinearOpMode.telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    myLinearOpMode.telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    myLinearOpMode.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            myLinearOpMode.telemetry.update();
            myLinearOpMode.sleep(20);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

        return tagOfInterest;
    }

    public void runOpMode()
    {
        //hwmap where??
        //Servo servo = hardwareMap.servo.get("servoName");
        int cameraMonitorViewId = myHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(myHardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        //myLinearOpMode.telemetry.setMsTransmissionInterval(50);
        //myLinearOpMode.waitForStart();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        /*
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
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


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        Servo servo = myHardwareMap.servo.get("servoName");
        myLinearOpMode.telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        myLinearOpMode.telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        myLinearOpMode.telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        myLinearOpMode.telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        myLinearOpMode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        myLinearOpMode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        myLinearOpMode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
    public double getTranslationX(AprilTagDetection detection){
        return detection.pose.x*FEET_PER_METER;
    }
}
