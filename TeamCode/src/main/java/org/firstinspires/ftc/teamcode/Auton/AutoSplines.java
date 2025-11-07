package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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
public class AutoSplines extends LinearOpMode {




    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));

        Shooter shooter = new Shooter(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);




        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);



        TrajectoryActionBuilder path1 =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39))
                .waitSeconds(1.5)
                .splineToSplineHeading(new Pose2d(-16.5, -20, Math.toRadians(-90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-17.5,-60), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20, -20), Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(-4, -60), Math.toRadians(30))
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39))
                .waitSeconds(0.8)
                .splineToSplineHeading(new Pose2d(1.5, -24, Math.toRadians(-90)), Math.toRadians(-75))
                .lineToY(-62)
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(39)), Math.toRadians(-30))
                .waitSeconds(1.6)
                .splineToSplineHeading(new Pose2d(23, -24, Math.toRadians(-90)), Math.toRadians(-75))
                .lineToY(-60)
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(39)), Math.toRadians(-75))
                .waitSeconds(1);

        Action path2 = path1.build();

        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(2860),
                        new SequentialAction(
                                new SleepAction(1.05),
                                transfer.fullLoad(),
                                new SleepAction(3.3),
                                transfer.run(),
                                new SleepAction(3.5),
                                transfer.fullLoad(),
                                new SleepAction(2.0),
                                transfer.run(),
                                new SleepAction(3.4),
                                transfer.fullLoad(),
                                new SleepAction(3.3),
                                transfer.run(),
                                new SleepAction(3.35),
                                transfer.fullLoad()

                        ),

                        path2
                )

        ));





    }

}
