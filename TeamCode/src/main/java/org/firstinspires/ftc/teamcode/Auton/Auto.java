package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


@Config
@Autonomous
public class Auto extends LinearOpMode {


    private static final Logger log = LoggerFactory.getLogger(Auto.class);
    Intake intake = new Intake(hardwareMap);
    Transfer transfer = new Transfer(hardwareMap);

    double distance = 0;

    Shooter shooter = new Shooter(hardwareMap);


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(60, -16, Math.toRadians(0));
        Pose2d startPose2 = new Pose2d(-16, -16, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);



        TrajectoryActionBuilder preload =  drive.actionBuilder(startPose1)

                .lineToX(-16)
                // first set
                .strafeToLinearHeading(new Vector2d(38, -30), Math.toRadians(90))
                .lineToY(-52)
                .strafeToLinearHeading(new Vector2d(-16, -16), Math.toRadians(0))
                // second set
                .strafeToLinearHeading(new Vector2d(11, -30), Math.toRadians(90))
                .lineToY(-52)
                .strafeToLinearHeading(new Vector2d(-16, -16), Math.toRadians(90))
                // third set
                .strafeToLinearHeading(new Vector2d(-15,-30) , Math.toRadians(90))
                .lineToY(-52)
                .strafeToLinearHeading(new Vector2d(-16,-16) , Math.toRadians((90));

        Action path1 = preload.build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        intake.run(),
                        transfer.load(),
                        intake.stop(),
                        path1
                )

        ));


    }

}
