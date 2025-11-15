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
public class Auto_pathing_far_side extends LinearOpMode {





    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(60, -16, Math.toRadians(0));
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        TrajectoryActionBuilder path2 = drive.actionBuilder(new Pose2d(64, -17, Math.toRadians(0))) // green purple purple, only for specific cases
                .strafeToLinearHeading(new Vector2d(0,-9), Math.toRadians(39))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-2, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-2, -53), Math.toRadians(-80))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(0,-9), Math.toRadians(39))
                .waitSeconds(1.3)
                .strafeToLinearHeading(new Vector2d(21, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-54)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(0, -9), Math.toRadians(39))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(40, -40), Math.toRadians(-50))
                .strafeToLinearHeading(new Vector2d(55, -62), Math.toRadians(-60))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-1, -12), Math.toRadians(39))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(10, -15), Math.toRadians(0));
        Action path22 = path2.build();



        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(3460),
                        new SequentialAction(
                                intake.run(),
                                new SleepAction(2.0),
                                transfer.farfullLoad(),
                                new SleepAction(0.5),
                                transfer.run(),
                                new SleepAction(2.8),
                                transfer.farfullLoad(),
                                new SleepAction(0.6),
                                transfer.run(),
                                new SleepAction(3.2),
                                transfer.farfullLoad(),
                                new SleepAction(0.5),
                                transfer.run(),
                                new SleepAction(3.3),
                                transfer.farfullLoad()

                        ),


                        path22

                )));
    }}
