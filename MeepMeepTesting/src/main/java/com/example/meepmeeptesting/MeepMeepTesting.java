package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.25, 17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(-3.5, 62.5, Math.toRadians(-90)))
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-3.5, 62.5, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(-3.5, 32.5, Math.toRadians(-90)))
                        /*.splineToSplineHeading(new Pose2d(-3.5, 40, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-20, 50, Math.toRadians(90)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-36, 61, Math.toRadians(90)), Math.toRadians(90))*/
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}