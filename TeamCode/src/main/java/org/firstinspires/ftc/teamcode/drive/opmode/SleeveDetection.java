package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
    private static Point rSLEEVE_TOPLEFT_ANCHOR_POINT = new Point(500, 368);

    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 200);
    // Width and height for the bounding box
    public static int REGION_WIDTH = 140;
    public static int REGION_HEIGHT = 50;

    public static int rREGION_WIDTH = 80;
    public static int rREGION_HEIGHT = 150;

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
        double minColormid = Math.max(sumColorsmid.val[0], Math.min(sumColorsmid.val[1], sumColorsmid.val[2]));

        Mat areaMatright = input.submat(new Rect(rsleeve_pointA, rsleeve_pointB));
        Scalar sumColorsright = Core.sumElems(areaMatright);
        double minColorright = Math.max(sumColorsright.val[0], Math.min(sumColorsright.val[1], sumColorsright.val[2]));

        // Get the minimum RGB value from every single channel

        // Change the bounding box color based on the sleeve color
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

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}