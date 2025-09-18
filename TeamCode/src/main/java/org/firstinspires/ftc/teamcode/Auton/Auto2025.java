package org.firstinspires.ftc.teamcode.Auton;

import android.graphics.Color;

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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;
/*
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.ExtendoV2;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.SlidesV3;
import org.firstinspires.ftc.teamcode.mechanisms.SweeperSample;
*/

@Config
@Autonomous(preselectTeleOp = "ABlueTeleop")
public class Auto2025 extends LinearOpMode {

    Camera camera = new Camera(hardwareMap);

    public static double aFarLaunchZoneX = 72;
    public static double aFarLaunchZoneY = 12;

    /*
    public static double apreloadX = -17.97;
    public static double apreloadY = 12.644;
    public static double apreloadH = 68.3;
    public static double bfirstsampleX = -8; //-18.5;
    public static double bfirstsampleY = 12.5; //20;
    public static double bfirstsampleH = 90; //76;
    public static double cfirstsampleintakeH = 69.8;
    public static double cfirstsampleintakex = -15;
    public static double cfirstsampleintakey = 20;
    public static double dfirstsampledepositX = -16.5;
    public static double dfirstsampledepositY = 12;
    public static double dfirstsampledepositH = 58;
    public static double esecondsampleH = 90;
    public static double esecondsamplex = -17;
    public static double esecondsampley = 13.3;
    public static double fsecondsampleintakeh = 95;
    public static double fsecondsampleintakex = -16.94;
    public static double fsecondsampleintakey = 21;
    public static double gsecondsampledepositX = -16.75;
    public static double gsecondsampledepositY = 13;
    public static double gsecondsampledepositH = 58;
    public static double hthirdsampleH = 140;
    public static double hthirdsamplex = -7.5;
    public static double hthirdsampley = 23.3;
    public static double ithirdsamplealignH = 123.35;
    public static double ithirdsamplealignx = -15.75;
    public static double ithirdsamplealigny = 15.9;
    public static double jthirdsampleintakeh = 129;
    public static double jthirdsampleintakex = -18;
    public static double jthirdsampleintakey = 24.6;
    public static double ksubmersibleintakex = 15;
    public static double ksubmersibleintakey = 55;
    public static double ksubmersibleintakeh = 0;
    public static double parkX = 10;

    public static double parkY = 67.07;

    public static double parkHead1 = 180;

    public static double parkHead2 = 0;
    */



    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(24, 12, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);


//        SlidesV3 slides = new SlidesV3(hardwareMap, true);
//        ExtendoV2 extendo = new ExtendoV2(hardwareMap);
//        Claw claw = new Claw(hardwareMap);
//        Intaker intake = new Intaker(hardwareMap);
//        SweeperSample sampleSweeper = new SweeperSample(hardwareMap);


        TrajectoryActionBuilder path1 = drive.actionBuilder(startPose1)
                .strafeToConstantHeading(new Vector2d(aFarLaunchZoneX, aFarLaunchZoneY));


        Action path = path1.build();

        waitForStart();

}