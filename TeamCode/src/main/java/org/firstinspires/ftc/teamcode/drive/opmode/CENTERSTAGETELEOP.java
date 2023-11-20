package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    SampleMecanumDrive drive;
    Trajectory forward;
    Trajectory backward;
    Trajectory left;
    Trajectory right;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        previousGamepad=gamepad1;
        currentGamepad=gamepad1;

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
        drive.followTrajectory(forward);
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
