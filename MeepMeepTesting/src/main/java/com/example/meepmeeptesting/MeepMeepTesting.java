package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-58, -55, 45))
                .strafeToLinearHeading(new Vector2d(-45, -30), 45)
                .waitSeconds(1) //launch balls
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(45)), Math.toRadians(-90)) //launch balls
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}