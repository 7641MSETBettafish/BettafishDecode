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

import java.util.Vector;


@Config
@Autonomous(preselectTeleOp = "Teleop")
public class Autored extends LinearOpMode {




    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-58, 58, Math.toRadians(-52));

        Shooter shooter = new Shooter(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);




        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);



        TrajectoryActionBuilder path1 =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-32, 20), Math.toRadians(-53))
                .waitSeconds(0.85)
                .strafeToLinearHeading(new Vector2d(0, 28), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(67)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(0, 80), Math.toRadians(180))
                .waitSeconds(0.01)
                .splineToLinearHeading(new Pose2d(-32, 20, Math.toRadians(-53)), Math.toRadians(-10))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(25, 35), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(70)
                .waitSeconds(0.01)
                .splineToLinearHeading(new Pose2d(-32, 20, Math.toRadians(-53)), Math.toRadians(135))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(50, 35), Math.toRadians(90))
                .waitSeconds(0.01)
                .lineToY(70)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-32, 20), Math.toRadians(-53))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(0, 30), Math.toRadians(0));


        Action path2 = path1.build();

        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(3175),
                        Context.updatePosition(drive, 0),
                        new SequentialAction(
                                intake.run(),
                                new SleepAction(1.2),
                                transfer.fullLoad(),
                                new SleepAction(2.4),
                                transfer.run(),
                                new SleepAction(4),
                                transfer.fullLoad(),
                                new SleepAction(2.0),
                                transfer.run(),
                                new SleepAction(3.3),
                                transfer.fullLoad(),
                                new SleepAction(2.5),
                                transfer.run(),
                                new SleepAction(3.2),
                                transfer.fullLoad()


                        ),

                        path2
                )

        ));





    }

}
