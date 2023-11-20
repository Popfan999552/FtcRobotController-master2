package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */


    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(320, 0);

    private static Point rSLEEVE_TOPLEFT_ANCHOR_POINT = new Point(0, 0);
    // Width and height for the bounding box
    public static int REGION_WIDTH = 320;
    public static int REGION_HEIGHT = 360;

    public static int rREGION_WIDTH = 320;
    public static int rREGION_HEIGHT = 360;

    // Color definitions
    /*private final Scalar
            RED  = new Scalar(255, 0, 0),
            BLUE    = new Scalar(0, 0, 255),
            BLACK = new Scalar(0, 0, 0);*/
    private final Scalar
            RED  = new Scalar(255, 0, 0),
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point rsleeve_pointA = new Point(
            rSLEEVE_TOPLEFT_ANCHOR_POINT.x,
            rSLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point rsleeve_pointB = new Point(
            rSLEEVE_TOPLEFT_ANCHOR_POINT.x + rREGION_WIDTH,
            rSLEEVE_TOPLEFT_ANCHOR_POINT.y + rREGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override

    public Mat processFrame(Mat input) {
        // Get the submat frame, and then sum all the values
        /*Mat areaMatleft = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColorsleft = Core.sumElems(areaMatleft);

        // Get the minimum RGB value from every single channel
        double minColorleft = Math.min(sumColorsleft.val[0], Math.min(sumColorsleft.val[1], sumColorsleft.val[2]));
        */


        Mat areaMatmid = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColorsmid = Core.sumElems(areaMatmid);
        double minColormid = Math.max(sumColorsmid.val[0], Math.max(sumColorsmid.val[1], sumColorsmid.val[2]));

        Mat areaMatright = input.submat(new Rect(rsleeve_pointA, rsleeve_pointB));
        Scalar sumColorsright = Core.sumElems(areaMatright);
        double minColorright = Math.max(sumColorsright.val[0], Math.max(sumColorsright.val[1], sumColorsright.val[2]));

        // Get the minimum RGB value from every single channel

        // Change the bounding box color based on the sleeve color
        //1 for green, 2 for red
        if (sumColorsmid.val[0] == minColormid) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );
        } else if (sumColorsright.val[0] == minColorright) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    rsleeve_pointA,
                    rsleeve_pointB,
                    RED,
                    2
            );
        } else {
            position = ParkingPosition.LEFT;
            /*Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    RED,
                    2
            );*/
        }

        // Release and return input
        areaMatmid.release();
        areaMatright.release();

        return input;
    }

    public Mat processFrameTest(Mat input)
    {
        //telemetry.addData("Status", "process frame");
        Mat YCBCr = new Mat ();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat ();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);

        Imgproc.cvtColor(input,YCBCr,Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect (0, 0, 0, 0);
        Rect rightRect = new Rect (0, 0, 0, 0);

        input.copyTo(outPut);
        Imgproc.rectangle(outPut, leftRect, rectColor, 0);
        Imgproc.rectangle(outPut, rightRect, rectColor, 0);

        leftCrop =YCBCr.submat(leftRect);
        rightCrop =YCBCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 0);
        Core.extractChannel(rightCrop, rightCrop, 0);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);

        Double leftavg_o = leftavg.val[0];
        Double rightavgf_o = rightavg.val[0];

        if (leftavg_o > rightavgf_o){
            //telemetry.addLine("Left");
        }
        else{

            //telemetry.addLine("Right");
        }

        return(outPut);

        //return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }


}