package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;


@Config
@Autonomous
public class Autored extends LinearOpMode {




    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, 50, Math.toRadians(-54));

        Shooter shooter = new Shooter(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);




        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);



        TrajectoryActionBuilder path1 =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-30, 25), Math.toRadians(-42))
                .waitSeconds(0.85)
                .strafeToLinearHeading(new Vector2d(-30, 20), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(45)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-17, 56), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30, 25, Math.toRadians(-42)), Math.toRadians(90))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(-5, 20), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(50)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, 25), Math.toRadians(-42))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15, 20), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(50)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, 25), Math.toRadians(-42))
                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(10, 30), Math.toRadians(0));

        Action path2 = path1.build();

        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(2850),
                        transfer.run(),
                        new SequentialAction(
                                new SleepAction(1.15),
                                transfer.fullLoad(),
                                new SleepAction(2.8),
                                transfer.run(),
                                new SleepAction(5.0),
                                transfer.fullLoad(),
                                new SleepAction(2.8),
                                transfer.run(),
                                new SleepAction(3),
                                transfer.fullLoad(),
                                new SleepAction(3.3),
                                transfer.run(),
                                new SleepAction(3),
                                transfer.fullLoad()


                        ),

                        path2
                )

        ));





    }

}
