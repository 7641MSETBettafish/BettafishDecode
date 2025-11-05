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
public class Auto extends LinearOpMode {




    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));

        Shooter shooter = new Shooter(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap, shooter);




        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);



        TrajectoryActionBuilder path1 =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(45))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(9, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .strafeToLinearHeading(new Vector2d(-30, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(30, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45));

        Action path2 = path1.build();

        waitForStart();

        telemetry.addData("transfer ticks", transfer.transferMotor.getCurrentPosition());
        telemetry.update();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                shooter.run(3300),
                                new SleepAction(0.5),
                                transfer.load()
                                ),

                        path2
                )

        ));




    }

}
