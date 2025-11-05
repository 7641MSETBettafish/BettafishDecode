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
public class Auto extends LinearOpMode {




    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));

        Shooter shooter = new Shooter(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap, shooter);
        Intake intake = new Intake(hardwareMap);




        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);



        TrajectoryActionBuilder path1 =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(35))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-16.5, -30), Math.toRadians(-90))
                .waitSeconds(0.3)
                .lineToY(-50, null, new ProfileAccelConstraint(-30.0, 50.0))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(9, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(35))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(30, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(35));

        Action path2 = path1.build();

        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(3150),
                        new SequentialAction(
                                new SleepAction(1),
                                transfer.load(),
                                new SleepAction(1),
                                transfer.run(),
                                new SleepAction(1.2),
                                transfer.load()


                                ),

                        path2
                )

        ));





    }

}
