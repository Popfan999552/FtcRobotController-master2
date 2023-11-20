package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class arushDrivetrainTeleOp extends OpMode {
    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;

    DcMotor[] drivetrainMotors={leftFront,leftRear,rightFront,rightRear};
    double leftFrontSpeed = 0;
    double leftRearSpeed = 0;
    double rightFrontSpeed = 0;
    double rightRearSpeed = 0;

    double[] motorSpeeds={leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed};

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor .class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        for (int i = 0; i < 4; i++){
            if(motorSpeeds[i]>1){
                drivetrainMotors[i].setPower(1);
            } else if(motorSpeeds[i]<-1){
                drivetrainMotors[i].setPower(-1);
            }
        }

    }


}
