package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.CENTERSTAGEROBOT;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class CENTERSTAGETELEOP extends OpMode {
    Gamepad previousGamepad;
    Gamepad currentGamepad;
    //Drivetrain motors
    SampleMecanumDrive drive;

    //Outtake motors
    DcMotor slides;
    DcMotor futureVirtualFourBar;
    //Intake motors
    DcMotor roller;
    //endgame motor
    DcMotor futureLeadScrew;
    DcMotor winch1;
    DcMotor winch2;
    //Outtake servos
    CRServo leftServo;
    CRServo rightServo;
    CRServo bucketServo;
    //endgame servo
    Servo droneLauncher;
    CRServo scissorLift;


    Trajectory forward;
    Trajectory backward;
    Trajectory left;
    Trajectory right;
    @Override
    public void init() {
        previousGamepad=gamepad1;
        currentGamepad=gamepad1;
        //Drivetrain
        drive = new SampleMecanumDrive(hardwareMap);

        //Outtake motors
        slides = hardwareMap.dcMotor.get("slides");
        //futureVirtualFourBar = hardwareMap.dcMotor.get("futureVirtualFourBar");

        //Outtake servos
        leftServo = hardwareMap.crservo.get("leftServo");
        rightServo = hardwareMap.crservo.get("rightServo");
        bucketServo = hardwareMap.crservo.get("bucketServo");

        //Intake motors
        roller = hardwareMap.dcMotor.get("roller");

        //endgame servo
        droneLauncher = hardwareMap.servo.get("droneLauncher");
        scissorLift = hardwareMap.crservo.get("scissorLift");

        //endgame motor
        //futureLeadScrew = hardwareMap.dcMotor.get("futureLeadScrew");
        winch1 = hardwareMap.dcMotor.get("winch1");
        winch2 = hardwareMap.dcMotor.get("winch2");





        forward = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
        backward = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .build();
        right = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(5)
                .build();
        left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(5)
                .build();

    }

    @Override
    public void loop() {
        previousGamepad=currentGamepad;
        currentGamepad=gamepad1;

        //intake controls
        roller.setPower(currentGamepad.right_trigger);

        //outtake controls
        if(currentGamepad.dpad_up){
            slides.setPower(1);
        } else{
            slides.setPower(0);
        }
        if(currentGamepad.dpad_down){
            slides.setPower(-1);
        } else{
            slides.setPower(0);
        }
        if(currentGamepad.dpad_left){
            rightServo.setPower(1);
            leftServo.setPower(-1);
        } else if(currentGamepad.dpad_right){
        rightServo.setPower(-1);
        leftServo.setPower(1);
        } else{
            rightServo.setPower(0);
            leftServo.setPower(0);
        }
        if(currentGamepad.dpad_right){
            bucketServo.setPower(1);
        } else if(currentGamepad.dpad_left){
            bucketServo.setPower(-1);
        } else {
            bucketServo.setPower(0);
        }

        //endgame controls
        //hang
        if(currentGamepad.b){
            winch1.setPower(1);
            winch2.setPower(1);

            double x=getRuntime();
            while(getRuntime()<getRuntime()+500){

            }
            scissorLift.setPower(1);
            x=getRuntime();
            while(getRuntime()<getRuntime()+500){

            }
            winch2.setPower(0);
            winch1.setPower(0);
            scissorLift.setPower(0);
        } else if(currentGamepad.a){
            scissorLift.setPower(-1);
            double x=getRuntime();
            while(getRuntime()<getRuntime()+500){

            }
            winch1.setPower(1);
            winch2.setPower(1);
            x=getRuntime();
            while(getRuntime()<getRuntime()+500){

            }
        }

        if(currentGamepad.x){
            //retract
            droneLauncher.setPosition(0.4);
        }
        if(currentGamepad.y){
            //release servo
            droneLauncher.setPosition(0.7);
        }
        //Gamepad1: drive, hanging

/*
        if(!previousGamepad.a && currentGamepad.a){
            //turn arm motor in the positive direction
            slidePower +=0.05;

            slide.setPower(slidePower);

        } else if (!previousGamepad.b && currentGamepad.b){
            //turn arm motor in the negative direction
            slidePower -=0.05;
            slide.setPower(slidePower);

        }

        if(!previousGamepad.right_bumper && currentGamepad.right_bumper){
            //turn arm motor in the positive direction
            v4bPos +=0.05;

            v4b.setPosition(v4bPos);

        } else if (!previousGamepad.left_bumper && currentGamepad.left_bumper){
            //turn arm motor in the negative direction
            v4bPos -=0.05;

            v4b.setPosition(v4bPos);

        }*/
        //motion
        //drive.followTrajectory(forward);
        if(currentGamepad.left_stick_y>0.9){
            drive.followTrajectory(forward);

        } else if (currentGamepad.left_stick_y<-0.9){

            drive.followTrajectory(backward);

        }else if (currentGamepad.left_stick_x<-0.9){

            drive.followTrajectory(left);

        }else if (currentGamepad.left_stick_y>0.9){

            drive.followTrajectory(right);

        }
        telemetry.addData("righttrigger", currentGamepad.right_trigger);
        telemetry.addData("rightbumper", currentGamepad.right_bumper);
        telemetry.addData("leftbumper", currentGamepad.left_bumper);
        telemetry.addData("lefttrigger", currentGamepad.left_trigger);
        telemetry.addData("dpadup", currentGamepad.dpad_up);
        telemetry.addData("dpaddown", currentGamepad.dpad_down);
        telemetry.addData("dpadleft", currentGamepad.dpad_left);
        telemetry.addData("dpadright", currentGamepad.dpad_right);
        telemetry.addData("joyleftX", currentGamepad.dpad_up);
        telemetry.addData("joyrightX", currentGamepad.dpad_down);
        telemetry.addData("joyleftY", currentGamepad.dpad_left);
        telemetry.addData("joyrightX", currentGamepad.dpad_right);
        telemetry.addData("a",currentGamepad.a);
        telemetry.addData("b",currentGamepad.b);
        telemetry.addData("x",currentGamepad.x);
        telemetry.addData("y",currentGamepad.y);

    }
}
