package org.firstinspires.ftc.teamcode.drive.opmode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.CENTERSTAGEROBOT;

public class CENTERSTAGETELEOP extends OpMode {
    Gamepad previousGamepad;
    Gamepad currentGamepad;
    CENTERSTAGEROBOT drive = new CENTERSTAGEROBOT(hardwareMap);

    //DcMotor slide=drive.getSlide(hardwareMap);
    //Servo v4b=drive.getV4b(hardwareMap);
    float slidePower =0;
    float v4bPos=90;
    Trajectory forward = drive.trajectoryBuilder(new Pose2d())
            .forward(5)
            .build();
    Trajectory backward = drive.trajectoryBuilder(new Pose2d())
            .back(5)
            .build();
    Trajectory right = drive.trajectoryBuilder(new Pose2d())
            .strafeRight(5)
            .build();
    Trajectory left = drive.trajectoryBuilder(new Pose2d())
            .strafeLeft(5)
            .build();
    @Override

    public void init() {




    }

    @Override
    public void loop() {
        previousGamepad=currentGamepad;
        currentGamepad=gamepad1;

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
        if(!previousGamepad.dpad_up && currentGamepad.dpad_up){
            drive.followTrajectory(forward);

        } else if (!previousGamepad.dpad_down && currentGamepad.dpad_down){

            drive.followTrajectory(backward);

        }else if (!previousGamepad.dpad_left && currentGamepad.dpad_left){

            drive.followTrajectory(left);

        }else if (!previousGamepad.dpad_right && currentGamepad.dpad_right){

            drive.followTrajectory(right);

        }



    }
}
