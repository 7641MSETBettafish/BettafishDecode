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
public class Autonosplinesnotiming extends LinearOpMode {




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
                .splineToSplineHeading(new Pose2d(-18.5, -20, Math.toRadians(-90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20,-60), Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-20, -50, Math.toRadians(0)), Math.toRadians(-30), null, new ProfileAccelConstraint(-30.0, 40.0))
                .strafeToLinearHeading(new Vector2d(-9, -70), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39))
                .waitSeconds(0.8)
                .splineToSplineHeading(new Pose2d(1.5, -24, Math.toRadians(-90)), Math.toRadians(-75))
                .lineToY(-60)
                .splineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(39)), Math.toRadians(-30))
                .waitSeconds(1.6)
                .splineToSplineHeading(new Pose2d(23, -24, Math.toRadians(-90)), Math.toRadians(-75))
                .lineToY(-60)
                .splineToSplineHeading(new Pose2d(-30, -25, Math.toRadians(39)), Math.toRadians(-75))
                .waitSeconds(1);

        TrajectoryActionBuilder preload =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39))

                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(39))
                .waitSeconds(1);


        TrajectoryActionBuilder firstrow =  drive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(39)))
                .strafeToLinearHeading(new Vector2d(-16.5, -24), Math.toRadians(-90))
                .waitSeconds(0.1)
                .lineToY(-52);


        TrajectoryActionBuilder shootfirst =  drive.actionBuilder(new Pose2d(-16.5, -52, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39));

        TrajectoryActionBuilder secondrow =  drive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(39)))
                .strafeToLinearHeading(new Vector2d(4, -24), Math.toRadians(-90))
                .lineToY(-60);

        TrajectoryActionBuilder shootsecond =  drive.actionBuilder(new Pose2d(4, -60, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39));

        TrajectoryActionBuilder thirdrow =  drive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(39)))
                .strafeToLinearHeading(new Vector2d(26, -24), Math.toRadians(-90))
                .lineToY(-62);

        TrajectoryActionBuilder shootthird =  drive.actionBuilder(new Pose2d(26, -63, Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(39));

        TrajectoryActionBuilder getout =  drive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(39)))
                .strafeToLinearHeading(new Vector2d(-5, -25), Math.toRadians(0));





        Action preloadpath = preload.build();
        Action firstrowpath = firstrow.build();
        Action firstshootpath = shootfirst.build();
        Action secondrowpath = secondrow.build();
        Action secondshootpath = shootsecond.build();
        Action thirdrowpath = thirdrow.build();
        Action thirdshootpath = shootthird.build();
        Action getoutpath = getout.build();

        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(3100),
                        preloadpath
                ),
                transfer.fullLoad(),
                new ParallelAction(
                        firstrowpath,
                        transfer.run()
                ),
                firstshootpath,
                transfer.fullLoad(),
                new ParallelAction(
                        secondrowpath,
                        transfer.run()
                ),
                secondshootpath,
                transfer.fullLoad(),
                new ParallelAction(
                        thirdrowpath,
                        transfer.run()
                ),
                thirdshootpath,
                transfer.fullLoad(),
                getoutpath





        ));






    }

}
