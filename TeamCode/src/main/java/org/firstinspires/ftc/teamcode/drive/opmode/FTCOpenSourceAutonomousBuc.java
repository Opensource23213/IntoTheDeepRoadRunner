

package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * FFTCOpenSourceAutonomouss Example for only vision detection using tensorflow and park
 */

@Autonomous(name = "FTCOpenSourceAutonomousBuc", group = "00-Autonomous", preselectTeleOp = "BucDays")
public class FTCOpenSourceAutonomousBuc extends LinearOpMode {

    public static String TEAM_NAME = "Open Source Robotics";
    public static int TEAM_NUMBER = 23213;
    private static final String TFOD_MODEL_FILE = "myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue Prop", "Red Prop"
    };
    private AprilTagProcessor aprilTag;
    private CRServo spinny;

    public SampleMecanumDrive drive;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private Servo elbow1;
    private Servo wristy;
    IMU imu;
    private Servo grabby;
    private Servo flippy;
    private DcMotor ArmPos = null;
    private Servo mustaches;
    private Servo ilifty;
    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistance2;
    private DistanceSensor flippydis;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor rear_left = null;
    private DcMotor rear_right = null;
    int use_tfod = 1;
    int use_apriltag = 2;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private VisionPortal myVisionPortal;

    //Define and declare Robot Starting Locations
    public enum THRU{
        TRUSS,
        WALL
    }
    public enum START_POSITION{
        BLUE_NEAR,
        BLUE_FAR,
        RED_FAR,
        RED_NEAR
    }
    public static THRU thruPosition;
    public static THRU parkPosition;
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    int sleeps = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        drive = new SampleMecanumDrive(hardwareMap);  //Moved from void automation//Moved from void automation
        spinny = hardwareMap.get(CRServo.class, "spinny");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        elbow1 = hardwareMap.get(Servo.class, "elbow1");
        grabby = hardwareMap.get(Servo.class, "grabby");
        wristy = hardwareMap.get(Servo.class, "wristy");
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        flippy = hardwareMap.get(Servo.class, "flippy");
        mustaches = hardwareMap.get(Servo.class, "mustaches");
        ilifty = hardwareMap.get(Servo.class, "lifty2");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensorDistance2");
        flippydis = hardwareMap.get(DistanceSensor.class, "flippydis");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) sensorDistance2;
        Rev2mDistanceSensor sensorTimeOfFlight3 = (Rev2mDistanceSensor) flippydis;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d poseEstimate = drive.getPoseEstimate();
        Arm1.setDirection(DcMotor.Direction.FORWARD);
        Arm2.setDirection(DcMotor.Direction.REVERSE);
        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow1.setDirection(Servo.Direction.REVERSE);
        ilifty.setPosition(.3);
        initDoubleVision();
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        myVisionPortal.setProcessorEnabled(tfod, true);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        double ticks = 22.76;
        flippy.setPosition(.5);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence traj13 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        double armAngle = ArmPos.getCurrentPosition() / ticks - 25;
        double serAdjust = -7;

        double stop = selectStartingPosition();
        grabby.setDirection(Servo.Direction.REVERSE);
        if (startPosition == START_POSITION.BLUE_NEAR || startPosition == START_POSITION.RED_NEAR){
            grabby.setPosition(.6);
        }else {
            grabby.setPosition(.97);
        }
        //Key Pay inputs to selecting Starting Position of robot

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData("|>", "Touch Play to start OpMode");
        telemetry.update();
        //
        double hey = 1;
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            myVisionPortal.setProcessorEnabled(aprilTag, false);
            myVisionPortal.setProcessorEnabled(tfod, true);
            runTfodTensorFlow();
            //identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;  //REMOVE!!!!
            if(gamepad1.dpad_up || gamepad1.dpad_down){
                hey = 2;
            }
            if (!gamepad1.dpad_down && hey == 2){
                hey = 1;
                sleeps -= .5;
            }
            if (!gamepad1.dpad_up && hey == 2){
                hey = 1;
                sleeps += .5;
            }
            telemetry.addData("Total time", stop + 24);
            telemetry.addData("Time before slide", stop + 16);
            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }
        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            double pix = 10;
            elbow1.setPosition(.91);
            double grabIn = 1;
            float wristIn = (float) ((float) .6 + serAdjust/355);

            if (startPosition == START_POSITION.BLUE_NEAR || startPosition == START_POSITION.RED_NEAR){
                grabby.setPosition(.6);
            }else {
                grabby.setPosition(.97);
            }
            safeWaitSeconds(.25);
            wristy.setPosition((.65 + 3/355)*.22);
            double grabOut = .79;
            float servoDegree = 1 / 355;
            Pose2d startbf = new Pose2d(-35,62, Math.toRadians(-90));
            Pose2d startrf = new Pose2d(-35, -62, Math.toRadians(90));
            Pose2d startrn = new Pose2d(12, -62, Math.toRadians(90));
            Pose2d startbn = new Pose2d(12, 62, Math.toRadians(-90));
            armpose(-10);
            //Build parking trajectory based on last detected target by vision
            myVisionPortal.setProcessorEnabled(aprilTag, true);
            myVisionPortal.setProcessorEnabled(tfod, false);
            while(opModeIsActive()) {
                switch (thruPosition) {
                    case WALL:
                        switch (startPosition) {
                            case BLUE_NEAR:
                                drive.setPoseEstimate(startbn);
                                //drive = new SampleMecanumDrive(hardwareMap);
                                switch (identifiedSpikeMarkLocation) {
                                    case LEFT:
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startbn)
                                                .splineToSplineHeading(new Pose2d(48, 44, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41, 44, Math.toRadians(0)))
                                                .addTemporalMarker(.3, () -> {
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(21, 40, Math.toRadians(-90)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(21, 47, Math.toRadians(-90)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.28);
                                                })
                                                .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(180)))

                                                .lineToConstantHeading(new Vector2d(-57, 60))

                                                .lineToConstantHeading(new Vector2d(-57, 35))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.68);
                                                })
                                                .lineToLinearHeading(new Pose2d(-62, 35, Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    spinny.setPower(1);
                                                    mustaches.setPosition(.42);
                                                    ilifty.setPosition(.3);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(-.09, () -> {
                                                    sleep(3000);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                                                    mustaches.setPosition(.28);
                                                    ilifty.setPosition(.5);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                                    spinny.setPower(0);
                                                    ilifty.setPosition(.5);
                                                    flippy.setPosition(.18);
                                                    double flipdis = flippydis.getDistance(DistanceUnit.INCH);
                                                    telemetry.addData("flippydis: ", flipdis);
                                                    telemetry.update();
                                                    grabby.setPosition(.97);
                                                    wristy.setPosition((float) (float) .67 * .22);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.85, () -> {
                                                    elbow1.setPosition(1);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.35, () -> {
                                                    grabby.setPosition(.6);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                                                    flippy.setPosition(.5);
                                                    elbow1.setPosition(.91);
                                                })
                                                .lineToLinearHeading(new Pose2d(-50, 60, Math.toRadians(0)))
                                                .splineToConstantHeading(new Vector2d(12, 60), Math.toRadians(0))
                                                .UNSTABLE_addTemporalMarkerOffset(.0001, () -> {
                                                    armpose(0);
                                                    elbow1.setPosition(0);
                                                    wristy.setPosition(((68 + 0 * 1.4 + 2 * 1.4) / 355) * .22);
                                                })
                                                .splineToConstantHeading(new Vector2d(48, 38), Math.toRadians(0))


                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        drop("frontr", "red", 99);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .splineToConstantHeading(new Vector2d(44, 44), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(44, 66), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .lineToLinearHeading(new Pose2d(60, 10, Math.toRadians(180)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }
                                        mustaches.setPosition(.28);
                                        safeWaitSeconds(1);
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        // purple drop
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startbn)
                                                .splineToSplineHeading(new Pose2d(48, 37.2, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41, 37, Math.toRadians(0)))
                                                .addTemporalMarker(.5, () -> {
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(12, 32.5, Math.toRadians(-90)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToConstantHeading(new Vector2d(12, 39.5))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.28);
                                                })
                                                .lineToLinearHeading(new Pose2d(20, 60, Math.toRadians(180)))
                                                .lineToConstantHeading(new Vector2d(-57, 60))

                                                .lineToConstantHeading(new Vector2d(-57, 35))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                   mustaches.setPosition(.68);
                                                })
                                                .lineToLinearHeading(new Pose2d(-62, 35, Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    spinny.setPower(1);
                                                    mustaches.setPosition(.42);
                                                    ilifty.setPosition(.3);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(-.09, () -> {
                                                    sleep(3000);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                                                    mustaches.setPosition(.28);
                                                    ilifty.setPosition(.5);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                                    spinny.setPower(0);
                                                    ilifty.setPosition(.5);
                                                    flippy.setPosition(.18);
                                                    double flipdis = flippydis.getDistance(DistanceUnit.INCH);
                                                    telemetry.addData("flippydis: ", flipdis);
                                                    telemetry.update();
                                                    grabby.setPosition(.97);
                                                    wristy.setPosition((float) (float) .67 * .22);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.85, () -> {
                                                    elbow1.setPosition(1);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.35, () -> {
                                                    grabby.setPosition(.6);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                                                    flippy.setPosition(.5);
                                                    elbow1.setPosition(.91);
                                                })
                                                .lineToLinearHeading(new Pose2d(-50, 60, Math.toRadians(0)))
                                                .splineToConstantHeading(new Vector2d(12, 60), Math.toRadians(0))
                                                .UNSTABLE_addTemporalMarkerOffset(.0001, () -> {
                                                    armpose(0);
                                                    elbow1.setPosition(0);
                                                    wristy.setPosition(((68 + 0 * 1.4 + 2 * 1.4) / 355) * .22);
                                                })
                                                .splineToConstantHeading(new Vector2d(48, 43), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        drop("frontr", "red", 99);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .splineToConstantHeading(new Vector2d(44, 43), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(44, 64), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .lineToLinearHeading(new Pose2d(60, 10, Math.toRadians(180)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }
                                        mustaches.setPosition(.28);
                                        safeWaitSeconds(1);


                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case RIGHT:
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startbn)
                                                .splineToSplineHeading(new Pose2d(48, 30, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41, 30, Math.toRadians(0)))
                                                .addTemporalMarker(.5, () -> {
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(8, 32, Math.toRadians(180)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                       traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToConstantHeading(new Vector2d(21, 32))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.28);
                                                })
                                                .lineToConstantHeading(new Vector2d(12, 59))
                                                .lineToConstantHeading(new Vector2d(-57, 58))

                                                .lineToConstantHeading(new Vector2d(-57, 32))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                   mustaches.setPosition(.68);
                                                })
                                                .lineToLinearHeading(new Pose2d(-62, 32 , Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    spinny.setPower(1);
                                                    mustaches.setPosition(.42);
                                                    ilifty.setPosition(.3);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(-.09, () -> {
                                                    sleep(3000);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                                                    mustaches.setPosition(.28);
                                                    ilifty.setPosition(.5);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                                    spinny.setPower(0);
                                                    ilifty.setPosition(.5);
                                                    flippy.setPosition(.18);
                                                    double flipdis = flippydis.getDistance(DistanceUnit.INCH);
                                                    telemetry.addData("flippydis: ", flipdis);
                                                    telemetry.update();
                                                    grabby.setPosition(.97);
                                                    wristy.setPosition((float) (float) .67 * .22);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.85, () -> {
                                                    elbow1.setPosition(1);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.35, () -> {
                                                    grabby.setPosition(.6);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                                                    flippy.setPosition(.5);
                                                    elbow1.setPosition(.91);
                                                })
                                                .lineToLinearHeading(new Pose2d(-50, 60, Math.toRadians(0)))
                                                .splineToConstantHeading(new Vector2d(12, 60), Math.toRadians(0))
                                                .UNSTABLE_addTemporalMarkerOffset(.0001, () -> {
                                                    armpose(0);
                                                    elbow1.setPosition(0);
                                                    wristy.setPosition(((68 + 0 * 1.4 + 2 * 1.4) / 355) * .22);
                                                })
                                                .splineToConstantHeading(new Vector2d(48, 43), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        drop("frontr", "red", 99);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .splineToConstantHeading(new Vector2d(44, 43), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(44, 64), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .lineToLinearHeading(new Pose2d(60, 10, Math.toRadians(180)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }
                                        mustaches.setPosition(.28);
                                        safeWaitSeconds(1);

                                        break;
                                }
                                break;

                            case RED_NEAR:
                                drive.setPoseEstimate(startrn);
                                //drive = new SampleMecanumDrive(hardwareMap);
                                switch (identifiedSpikeMarkLocation) {
                                    case RIGHT:
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startrn)
                                                .splineToSplineHeading(new Pose2d(48, -44, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41, -44, Math.toRadians(0)))
                                                .addTemporalMarker(.3, () -> {
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(21, -40, Math.toRadians(90)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(21, -47, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(12, -60, Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.28);
                                                })
                                                .lineToConstantHeading(new Vector2d(-55, -60))

                                                .lineToConstantHeading(new Vector2d(-55, -35))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.68);
                                                })
                                                .lineToLinearHeading(new Pose2d(-62, -35, Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    spinny.setPower(1);
                                                    mustaches.setPosition(.42);
                                                    ilifty.setPosition(.3);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(-.09, () -> {
                                                    sleep(3000);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                                                    mustaches.setPosition(.28);
                                                    ilifty.setPosition(.5);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                                    spinny.setPower(0);
                                                    ilifty.setPosition(.5);
                                                    flippy.setPosition(.18);
                                                    double flipdis = flippydis.getDistance(DistanceUnit.INCH);
                                                    telemetry.addData("flippydis: ", flipdis);
                                                    telemetry.update();
                                                    grabby.setPosition(.97);
                                                    wristy.setPosition((float) (float) .67 * .22);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.85, () -> {
                                                    elbow1.setPosition(1);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.35, () -> {
                                                    grabby.setPosition(.6);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                                                    flippy.setPosition(.5);
                                                    elbow1.setPosition(.91);
                                                })
                                                .lineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(0.0001)))
                                                .splineToConstantHeading(new Vector2d(20, -60), Math.toRadians(0))
                                                .UNSTABLE_addTemporalMarkerOffset(.0001, () -> {
                                                    armpose(0);
                                                    elbow1.setPosition(0);
                                                    wristy.setPosition(((68 + 0 * 1.4 + 2 * 1.4) / 355) * .22);
                                                })
                                                .splineToConstantHeading(new Vector2d(48, -37), Math.toRadians(0))


                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        drop("frontr", "red", 99);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .splineToConstantHeading(new Vector2d(44, -43), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(44, -62), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(180)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }
                                        mustaches.setPosition(.28);
                                        safeWaitSeconds(1);
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        // purple drop
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startrn)
                                                .splineToSplineHeading(new Pose2d(48, -37.2, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41, -37, Math.toRadians(0)))
                                                .addTemporalMarker(.5, () -> {
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(12, -32.5, Math.toRadians(90)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToConstantHeading(new Vector2d(12, -39.5))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.28);
                                                })
                                                .lineToLinearHeading(new Pose2d(20, -60, Math.toRadians(180)))
                                                .lineToConstantHeading(new Vector2d(-55, -60))

                                                .lineToConstantHeading(new Vector2d(-55, -35))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.68);
                                                })
                                                .lineToLinearHeading(new Pose2d(-62, -35, Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    spinny.setPower(1);
                                                    mustaches.setPosition(.42);
                                                    ilifty.setPosition(.3);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(-.09, () -> {
                                                    sleep(3000);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                                                    mustaches.setPosition(.28);
                                                    ilifty.setPosition(.5);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                                    spinny.setPower(0);
                                                    ilifty.setPosition(.5);
                                                    flippy.setPosition(.18);
                                                    double flipdis = flippydis.getDistance(DistanceUnit.INCH);
                                                    telemetry.addData("flippydis: ", flipdis);
                                                    telemetry.update();
                                                    grabby.setPosition(.97);
                                                    wristy.setPosition((float) (float) .67 * .22);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.85, () -> {
                                                    elbow1.setPosition(1);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.35, () -> {
                                                    grabby.setPosition(.6);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                                                    flippy.setPosition(.5);
                                                    elbow1.setPosition(.91);
                                                })
                                                .lineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(0.0001)))
                                                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(0))
                                                .UNSTABLE_addTemporalMarkerOffset(.0001, () -> {
                                                    armpose(0);
                                                    elbow1.setPosition(0);
                                                    wristy.setPosition(((68 + 0 * 1.4 + 2 * 1.4) / 355) * .22);
                                                })
                                                .splineToConstantHeading(new Vector2d(48, -37), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        drop("frontr", "red", 99);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .splineToConstantHeading(new Vector2d(44, -43), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(44, -64), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(180)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }
                                        mustaches.setPosition(.28);
                                        safeWaitSeconds(1);


                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case LEFT:
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startrn)
                                                .splineToSplineHeading(new Pose2d(48, -30, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41, -30, Math.toRadians(0)))
                                                .addTemporalMarker(.5, () -> {
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(8, -32, Math.toRadians(180)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToConstantHeading(new Vector2d(21, -32))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.28);
                                                })
                                                .lineToConstantHeading(new Vector2d(12, -60))
                                                .lineToConstantHeading(new Vector2d(-57, -60))

                                                .lineToConstantHeading(new Vector2d(-55, -37))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    mustaches.setPosition(.68);
                                                })
                                                .lineToLinearHeading(new Pose2d(-63, -37, Math.toRadians(180)))
                                                .UNSTABLE_addTemporalMarkerOffset(-.1, () -> {
                                                    spinny.setPower(1);
                                                    mustaches.setPosition(.42);
                                                    ilifty.setPosition(.3);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(-.09, () -> {
                                                    sleep(3000);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                                                    mustaches.setPosition(.28);
                                                    ilifty.setPosition(.5);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                                    spinny.setPower(0);
                                                    ilifty.setPosition(.5);
                                                    flippy.setPosition(.18);
                                                    double flipdis = flippydis.getDistance(DistanceUnit.INCH);
                                                    telemetry.addData("flippydis: ", flipdis);
                                                    telemetry.update();
                                                    grabby.setPosition(.97);
                                                    wristy.setPosition((float) (float) .67 * .22);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(2.85, () -> {
                                                    elbow1.setPosition(1);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.35, () -> {
                                                    grabby.setPosition(.6);
                                                })
                                                .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                                                    flippy.setPosition(.5);
                                                    elbow1.setPosition(.91);
                                                })
                                                .lineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(.0001)))
                                                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(0))
                                                .UNSTABLE_addTemporalMarkerOffset(.0001, () -> {
                                                    armpose(0);
                                                    elbow1.setPosition(0);
                                                    wristy.setPosition(((68 + 0 * 1.4 + 2 * 1.4) / 355) * .22);
                                                })
                                                .splineToConstantHeading(new Vector2d(48, -38), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        drop("frontr", "red", 99);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .splineToConstantHeading(new Vector2d(44, -44), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(44, -62), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj3.end())
                                                    .lineToLinearHeading(new Pose2d(60, -10, Math.toRadians(180)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }
                                        mustaches.setPosition(.28);
                                        safeWaitSeconds(1);

                                        break;
                                }
                                break;

                            case BLUE_FAR:
                                //drive = new SampleMecanumDrive(hardwareMap);
                                drive.setPoseEstimate(startbf);
                                switch (identifiedSpikeMarkLocation) {
                                    case LEFT:
                                        skip(stop - 1);
                                        traj1 = drive.trajectorySequenceBuilder(startbf)
                                                .splineTo(new Vector2d(-31, 35), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineTo(new Vector2d(-40, 35))
                                                .splineToSplineHeading(new Pose2d(-52, 31, Math.toRadians(180)), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(-61, 34), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(-52, 39, Math.toRadians(180)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(0)))
                                                .addTemporalMarker(1, () -> {
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);

                                        pix = pick();


                                        traj3 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(9, 60), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(37, 48), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(getaprilpose(1, 37, 49, 0, 43, 44))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontr", "blue", pix);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, 63))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, 11))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        // purple drop
                                        skip(stop - 1);
                                        traj1 = drive.trajectorySequenceBuilder(startbf)
                                                .lineToLinearHeading(new Pose2d(-39, 31, Math.toRadians(-90)))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(-39, 44, Math.toRadians(-90)))
                                                .lineToLinearHeading(new Pose2d(-52, 34, Math.toRadians(180)))
                                                .lineToConstantHeading(new Vector2d(-61, 36))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(-52, 39, Math.toRadians(180)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(-35, 58, Math.toRadians(0)))
                                                .addTemporalMarker(1, () -> {
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);

                                        pix = pick();


                                        traj3 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(9, 58), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(37, 41), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(getaprilpose(2, 37, 41, 0, 37, 44))

                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontr", "blue", pix);
                                        if (parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, 63))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, 11))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case RIGHT:
                                        skip(stop - 1);
                                        traj1 = drive.trajectorySequenceBuilder(startbf)
                                                .splineToConstantHeading(new Vector2d(-54.5, 50), Math.toRadians(-90))
                                                .splineToConstantHeading(new Vector2d(-54.5, 38), Math.toRadians(-90))
                                                .splineToConstantHeading(new Vector2d(-44, 32), Math.toRadians(-90))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(-46, 48, Math.toRadians(-90)))
                                                .lineToLinearHeading(new Pose2d(-52, 36, Math.toRadians(180)))
                                                .lineToConstantHeading(new Vector2d(-61, 36))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(-52, 36, Math.toRadians(180)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(0)))
                                                .addTemporalMarker(1, () -> {
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);

                                        pix = pick();


                                        traj3 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(9, 60), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(37, 34), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(getaprilpose(3, 37, 38, 0, 30, 43))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontl", "blue", pix);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, 63))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, 11))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                }
                                safeWaitSeconds(1);
                /*midwayPose1a = new Pose2d(30, -18, Math.toRadians(90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(90));
                midwayPose2 = new Pose2d(47, -23, Math.toRadians(0));
                midwayPose3 = new Pose2d(48, 0, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board*/
                                break;

                            case RED_FAR:
                                //drive = new SampleMecanumDrive(hardwareMap);
                                drive.setPoseEstimate(startrf);
                                switch (identifiedSpikeMarkLocation) {
                                    case RIGHT:
                                        skip(stop - 1);
                                        traj1 = drive.trajectorySequenceBuilder(startrf)
                                                .splineTo(new Vector2d(-31, -35), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToConstantHeading(new Vector2d(-40, -35))
                                                .splineToSplineHeading(new Pose2d(-52, -37, Math.toRadians(180)), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(-61, -37), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(-52, -39, Math.toRadians(180)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(-35, -57, Math.toRadians(0)))
                                                .addTemporalMarker(1, () -> {
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);

                                        pix = pick();


                                        traj3 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(28, -57), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(37, -31), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(getaprilpose(6, 38, -31, 0, -38, 44))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontl", "red", pix);
                                        if (parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, -63))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, -11))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        skip(stop - 1);
                                        // purple drop
                                        traj1 = drive.trajectorySequenceBuilder(startrf)
                                                .lineToLinearHeading(new Pose2d(-39, -31, Math.toRadians(90)))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(-39, -39, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(-52, -36, Math.toRadians(180)))
                                                .lineToConstantHeading(new Vector2d(-61, -36))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(-52, -39, Math.toRadians(180)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(-35, -58, Math.toRadians(0)))
                                                .addTemporalMarker(1, () -> {
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);

                                        pix = pick();


                                        traj3 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(15, -58), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(37, -25), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(getaprilpose(5, 37, -26.5f, 0, -30, 44))

                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("front", "red", pix);
                                        if(parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, -63))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, -11))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case LEFT:
                                        skip(stop - 1);
                                        traj1 = drive.trajectorySequenceBuilder(startrf)
                                                .splineToConstantHeading(new Vector2d(-52.5, -50), Math.toRadians(90))
                                                .splineToConstantHeading(new Vector2d(-52.5, -38), Math.toRadians(90))
                                                .splineToConstantHeading(new Vector2d(-44, -32), Math.toRadians(90))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(-46, -48, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(-52, -36, Math.toRadians(180)))
                                                .lineToConstantHeading(new Vector2d(-61, -36))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(-52, -36, Math.toRadians(180)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(-35, -57, Math.toRadians(0)))
                                                .addTemporalMarker(1, () -> {
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);

                                        pix = pick();


                                        traj3 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(9, -57), Math.toRadians(0))
                                                .splineToConstantHeading(new Vector2d(37, -21), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(getaprilpose(4, 37, -14.5F, 0, -26 , 42))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontr", "red", pix);
                                        if (parkPosition == THRU.WALL) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, -63))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToConstantHeading(new Vector2d(42, -11))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                }
                                safeWaitSeconds(1);
                                break;
                        }
                        break;
                    case TRUSS:
                        switch (startPosition) {
                            case BLUE_NEAR:
                                //drive = new SampleMecanumDrive(hardwareMap);
                                switch (identifiedSpikeMarkLocation) {
                                    case RIGHT:
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .splineTo(new Vector2d(27, -4), Math.toRadians(-90))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(27, 5, Math.toRadians(-90)))
                                                .lineToLinearHeading(new Pose2d(25, 18, Math.toRadians(90)))
                                                .build();

                                        drive.followTrajectorySequence(traj2);
                                        mustaches.setPosition(.28);
                                        pix = pick();
                                        armout(2);
                                        traj5 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(getaprilpose(3, 30,15,90, 10, 23))
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        drop("frontl", "blue", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                    .lineToLinearHeading(new Pose2d(0, 37, Math.toRadians(90)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }else{

                                        }
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .lineTo(new Vector2d(31, 5))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(23, 5, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(23, 19, Math.toRadians(90)))
                                                .build();

                                        drive.followTrajectorySequence(traj2);
                                        mustaches.setPosition(.28);
                                        pix = pick();
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(getaprilpose(2, 23,16,90, 17, 23))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("front", "blue", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, 39, Math.toRadians(90)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }else{

                                        }

                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case LEFT:
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .lineTo(new Vector2d(20, 11))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(12, 11, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(15, 25, Math.toRadians(90)))
                                                .build();

                                        drive.followTrajectorySequence(traj2);
                                        mustaches.setPosition(.28);
                                        pix = pick();
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(getaprilpose(1, 14,22,90, 23, 15))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontr", "blue", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, 39, Math.toRadians(90)))
                                                    .build();

                                            drive.followTrajectorySequence(traj7);
                                        }else{

                                        }

                            /*traj7 = drive.trajectorySequenceBuilder(traj2.end())//check distance to wall in front
                                    .forward(distancechange(4, "frontl"))
                                    .build();*/


                                        break;
                                }

                                //waitSecondsBeforeDrop = 2; //TODO: Adjust time to  wait for alliance partner to move from board
                                break;

                            case RED_NEAR:
                                drive.setPoseEstimate(startrn);
                                //drive = new SampleMecanumDrive(hardwareMap);
                                switch (identifiedSpikeMarkLocation) {
                                    case RIGHT:
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startrn)
                                                .splineToSplineHeading(new Pose2d(48,-44, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41,-37, Math.toRadians(0)))
                                                .addTemporalMarker(.5,()->{
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(19,-36, Math.toRadians(90)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj8 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToConstantHeading(new Vector2d(19,-45))
                                                .lineToLinearHeading(new Pose2d(20,-62, Math.toRadians(180)))
                                                .splineToConstantHeading(new Vector2d(-35,-62), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(-61,-42), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        stack1();
                                        safeWaitSeconds(.2);
                                        stack2();
                                        in();
                                        traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                .lineToLinearHeading(new Pose2d(-35,-59,  Math.toRadians(180)))
                                                .addTemporalMarker(1.2,()->{
                                                    stopp();
                                                })
                                                .splineToConstantHeading(new Vector2d(9,-59), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        if (twopix()) {
                                            pick();
                                            traj7 = drive.trajectorySequenceBuilder(traj7.end())
                                                    .splineToConstantHeading(new Vector2d(12, -59), Math.toRadians(0))
                                                    .addTemporalMarker(.8, () -> {
                                                        armout(2);
                                                    })
                                                    .splineToSplineHeading(new Pose2d(36, -59, Math.toRadians(1)), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(49, -37), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                            if (twopix()){
                                                b();
                                                safeWaitSeconds(1.2);
                                                pick();
                                                armout(2);
                                                safeWaitSeconds(1);
                                            }
                                            drop("frontl", "red", 211);
                                            if (parkPosition == THRU.TRUSS) {
                                                traj13 = drive.trajectorySequenceBuilder(traj7.end())
                                                        .lineToConstantHeading(new Vector2d(46, -63))
                                                        .build();
                                                drive.followTrajectorySequence(traj13);
                                            }else{

                                            }
                                        }else {
                                            traj7 = drive.trajectorySequenceBuilder(traj7.end())
                                                    .lineToConstantHeading(new Vector2d(55, -59))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                            release();
                                            if (parkPosition == THRU.TRUSS) {
                                                traj13 = drive.trajectorySequenceBuilder(traj7.end())
                                                        .lineToConstantHeading(new Vector2d(60, -63))
                                                        .build();
                                                drive.followTrajectorySequence(traj13);
                                            }else{

                                            }

                                        }
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        // purple drop
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startrn)
                                                .splineToSplineHeading(new Pose2d(48,-37, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41,-37, Math.toRadians(0)))
                                                .addTemporalMarker(.5,()->{
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(12,-33, Math.toRadians(90)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj8 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToConstantHeading(new Vector2d(12,-38))
                                                .lineToLinearHeading(new Pose2d(20,-61, Math.toRadians(180)))
                                                .splineToConstantHeading(new Vector2d(-35,-61), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(-61,-39), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        stack1();
                                        safeWaitSeconds(.2);
                                        stack2();
                                        in();
                                        traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                .lineToLinearHeading(new Pose2d(-35,-58,  Math.toRadians(180)))
                                                .addTemporalMarker(1.2,()->{
                                                    stopp();
                                                })
                                                .splineToConstantHeading(new Vector2d(9,-56), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        if (twopix()) {
                                            pick();
                                            traj7 = drive.trajectorySequenceBuilder(traj7.end())
                                                    .splineToConstantHeading(new Vector2d(12, -59), Math.toRadians(0))
                                                    .addTemporalMarker(.8, () -> {
                                                        armout(2);
                                                    })
                                                    .splineToSplineHeading(new Pose2d(36, -59, Math.toRadians(1)), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(49, -37), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                            if (twopix()){
                                                b();
                                                safeWaitSeconds(1.2);
                                                pick();
                                                armout(2);
                                                safeWaitSeconds(1);
                                            }
                                            drop("frontl", "red", 211);
                                            if (parkPosition == THRU.TRUSS) {
                                                traj13 = drive.trajectorySequenceBuilder(traj7.end())
                                                        .lineToConstantHeading(new Vector2d(46, -63))
                                                        .build();
                                                drive.followTrajectorySequence(traj13);
                                            }else{

                                            }
                                        }else {
                                            traj7 = drive.trajectorySequenceBuilder(traj7.end())
                                                    .lineToConstantHeading(new Vector2d(55, -59))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                            release();
                                            if (parkPosition == THRU.TRUSS) {
                                                traj13 = drive.trajectorySequenceBuilder(traj7.end())
                                                        .lineToConstantHeading(new Vector2d(60, -63))
                                                        .build();
                                                drive.followTrajectorySequence(traj13);
                                            }else{

                                            }

                                        }
                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case LEFT:
                                        armout(1);
                                        traj1 = drive.trajectorySequenceBuilder(startrn)
                                                .splineToSplineHeading(new Pose2d(48,-32, Math.toRadians(0)), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        grabby.setPosition(.97);
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(41,-32, Math.toRadians(0)))
                                                .addTemporalMarker(0,()->{
                                                    b();
                                                })
                                                .splineToSplineHeading(new Pose2d(8,-35, Math.toRadians(180)), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        release();
                                        safeWaitSeconds(.2);
                                        traj8 = drive.trajectorySequenceBuilder(traj2.end())
                                                .splineToConstantHeading(new Vector2d(16,-33), 0)
                                                .splineToConstantHeading(new Vector2d(20,-12), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(-35,-14), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(-62,-12), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        stack1();
                                        safeWaitSeconds(.2);
                                        stack2();
                                        in();
                                        traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                .lineToLinearHeading(new Pose2d(-35,-15,  Math.toRadians(180)))
                                                .addTemporalMarker(1.2,()->{
                                                    stopp();
                                                })
                                                .splineToConstantHeading(new Vector2d(9,-12), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        if (twopix()) {
                                            pick();

                                            traj7 = drive.trajectorySequenceBuilder(traj7.end())
                                                    .splineToConstantHeading(new Vector2d(12, -12), Math.toRadians(0))
                                                    .addTemporalMarker(.8, () -> {
                                                        armout(2);
                                                    })
                                                    .splineToSplineHeading(new Pose2d(36, -19, Math.toRadians(1)), Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(49, -27), Math.toRadians(0))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                            if (twopix()){
                                                b();
                                                safeWaitSeconds(1.2);
                                                pick();
                                                armout(2);
                                                safeWaitSeconds(1);
                                            }
                                            drop("frontl", "red", 211);
                                            if (parkPosition == THRU.TRUSS) {
                                                traj13 = drive.trajectorySequenceBuilder(traj7.end())
                                                        .lineToConstantHeading(new Vector2d(46, -12))
                                                        .build();
                                                drive.followTrajectorySequence(traj13);
                                            }else{

                                            }

                                        }else {
                                            traj7 = drive.trajectorySequenceBuilder(traj7.end())
                                                    .lineToConstantHeading(new Vector2d(55, -9))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                            release();
                                            if (parkPosition == THRU.TRUSS) {
                                                traj13 = drive.trajectorySequenceBuilder(traj7.end())
                                                        .lineToConstantHeading(new Vector2d(60, -9))
                                                        .build();
                                                drive.followTrajectorySequence(traj13);
                                            }

                                        }
                                        break;
                                }
                                break;

                            case BLUE_FAR:
                                //drive = new SampleMecanumDrive(hardwareMap);
                                switch (identifiedSpikeMarkLocation) {
                                    case LEFT:
                                        skip(stop - 1);
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .splineTo(new Vector2d(27, 4), Math.toRadians(90))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(27, -5, Math.toRadians(90)))
                                                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(270)))
                                                .lineToLinearHeading(new Pose2d(39, -26, Math.toRadians(270)))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(41, -17, Math.toRadians(270)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(49, -5, Math.toRadians(90)))
                                                .addTemporalMarker(1,()->{
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        pix = pick();

                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(31, 76), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(10, 72), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(getaprilpose(1,12,71,92, 71, 19))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontr", "blue", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(50, 76, Math.toRadians(90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                        }else{
                                            traj7 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, 76, Math.toRadians(90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj7);
                                        }
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        skip(stop);
                                        // purple drop
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .lineTo(new Vector2d(31, -5))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(21, -5, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(21, -17, Math.toRadians(270)))
                                                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(270)))
                                                .lineToLinearHeading(new Pose2d(41, -26, Math.toRadians(270)))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(41, -17, Math.toRadians(270)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(52, -5, Math.toRadians(90)))
                                                .addTemporalMarker(1,()->{
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        pix = pick();

                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(28.1, 76), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(15, 71.5), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(getaprilpose(2,16.5F,70,90, 72, 21))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("front", "blue", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(49, 78, Math.toRadians(90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, 78, Math.toRadians(90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case RIGHT:
                                        skip(stop);
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .lineTo(new Vector2d(27, -12))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineTo(new Vector2d(16, -12))
                                                .lineTo(new Vector2d(17, 1))
                                                .build();

                                        drive.followTrajectorySequence(traj2);
                                        traj5 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineTo(new Vector2d(46, 1))
                                                .splineTo(new Vector2d(50, -27), Math.toRadians(270))
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .lineToLinearHeading(new Pose2d(50, -17, Math.toRadians(270)))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(new Pose2d(50, -5, Math.toRadians(90)))
                                                .addTemporalMarker(1,()->{
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        pix = pick();

                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(50, 50), Math.toRadians(90))
                                                .splineToConstantHeading(new Vector2d(26, 72), Math.toRadians(0))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(getaprilpose(3,24,71,90, 72, 32))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontl", "blue", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(49, 78, Math.toRadians(90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, 78, Math.toRadians(90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                }safeWaitSeconds(1);
                /*midwayPose1a = new Pose2d(30, -18, Math.toRadians(90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(90));
                midwayPose2 = new Pose2d(47, -23, Math.toRadians(0));
                midwayPose3 = new Pose2d(48, 0, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board*/
                                break;

                            case RED_FAR:
                                //drive = new SampleMecanumDrive(hardwareMap);
                                switch (identifiedSpikeMarkLocation) {
                                    case RIGHT:
                                        skip(stop);
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .splineTo(new Vector2d(27, -4), Math.toRadians(-90))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(27, 5, Math.toRadians(-90)))
                                                .lineToLinearHeading(new Pose2d(37, 17, Math.toRadians(-270)))
                                                .lineToLinearHeading(new Pose2d(37, 27, Math.toRadians(-270)))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(38, 17, Math.toRadians(-270)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(51, 5, Math.toRadians(-90)))
                                                .addTemporalMarker(1,()->{
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        pix = pick();

                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(31, -76), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(29, -72), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(getaprilpose(6, 30,-72,-90, -72, 25))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontl", "red", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(57, -78, Math.toRadians(-90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, -78, Math.toRadians(-90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;

                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//

                                    case MIDDLE:
                                        skip(stop);
                                        // purple drop
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .lineTo(new Vector2d(31, 5))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineToLinearHeading(new Pose2d(21, 5, Math.toRadians(0)))
                                                .lineToLinearHeading(new Pose2d(21, 17, Math.toRadians(-270)))
                                                .lineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-270)))
                                                .lineToLinearHeading(new Pose2d(52, 26, Math.toRadians(-270)))
                                                .build();
                                        drive.followTrajectorySequence(traj2);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineToLinearHeading(new Pose2d(52, 17, Math.toRadians(-270)))
                                                .build();
                                        drive.followTrajectorySequence(traj3);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj3.end())
                                                .lineToLinearHeading(new Pose2d(52, 5, Math.toRadians(-90)))
                                                .addTemporalMarker(1,()->{
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        pix = pick();

                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(30.1, -76), Math.toRadians(180))
                                                .splineToConstantHeading(new Vector2d(36, -72), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(getaprilpose(5,36.5F,-72,-90, -71, 21))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("front", "red", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(49, -78, Math.toRadians(-90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, -78, Math.toRadians(-90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                    //----------------------------------------------------------------------------//
                                    ////////////////////////////////////////////////////////////////////////////////
                                    // ---------------------------------------------------------------------------//
                                    case LEFT:
                                        skip(stop);
                                        traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                                                .lineTo(new Vector2d(27, 12))
                                                .build();
                                        drive.followTrajectorySequence(traj1);
                                        release();
                                        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                                                .lineTo(new Vector2d(16, 12))
                                                .lineTo(new Vector2d(17, -1))
                                                .build();

                                        drive.followTrajectorySequence(traj2);
                                        traj5 = drive.trajectorySequenceBuilder(traj2.end())
                                                .lineTo(new Vector2d(46, -1))
                                                .splineTo(new Vector2d(49, 26), Math.toRadians(-270))
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        stack1();
                                        safeWaitSeconds(.5);
                                        stack2();
                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .lineToLinearHeading(new Pose2d(50, 15, Math.toRadians(-270)))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        in();
                                        traj5 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(new Pose2d(52, 5, Math.toRadians(-90)))
                                                .addTemporalMarker(1,()->{
                                                    stopp();
                                                })
                                                .build();
                                        drive.followTrajectorySequence(traj5);
                                        pix = pick();
                                        stop -= 1;

                                        traj7 = drive.trajectorySequenceBuilder(traj5.end())
                                                .splineToConstantHeading(new Vector2d(52, -50), Math.toRadians(90))
                                                .splineToConstantHeading(new Vector2d(41, -72), Math.toRadians(180))
                                                .build();
                                        drive.followTrajectorySequence(traj7);
                                        armout(2);
                                        traj8 = drive.trajectorySequenceBuilder(traj7.end())
                                                .lineToLinearHeading(getaprilpose(4, 44,-70,-90, -72, 32))
                                                .build();
                                        drive.followTrajectorySequence(traj8);
                                        drop("frontr", "red", pix);
                                        if (parkPosition == THRU.TRUSS) {
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(57, -78, Math.toRadians(-90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        else{
                                            traj13 = drive.trajectorySequenceBuilder(traj8.end())
                                                    .lineToLinearHeading(new Pose2d(0, -78, Math.toRadians(-90)))
                                                    .build();
                                            drive.followTrajectorySequence(traj13);
                                        }
                                        break;
                                } safeWaitSeconds(2);
                                break;
                        }
                }
                    break;
            }
        }
    }   // end runOpMode // end runOpMode()

    private void stack1() {
        spinny.setPower(1);
        flippy.setPosition(.5);
        mustaches.setPosition(.42);
        safeWaitSeconds(.6);
    }
    public void stack2() {
        mustaches.setPosition(.68);
        spinny.setPower(0);
    }
    public void in() {
        spinny.setPower(1);
        mustaches.setPosition(.28);
    }
    public void stopp() {
        spinny.setPower(0);
        ilifty.setPosition(.5);
        flippy.setPosition(.23);
    }
    public void release() {
        ilifty.setPosition(.3);
        mustaches.setPosition(.68);
        safeWaitSeconds(.1);
    }
    public void b(){
        elbow1.setPosition(.91);
        wristy.setPosition((float) ((float) .67 * .22));
        if(startPosition == START_POSITION.BLUE_FAR || startPosition == START_POSITION.RED_FAR){
            flippy.setPosition(.28);
        }

        armpose(-10);
    }
    public double pick(){
        double grabIn = 1;
        ilifty.setPosition(.5);
        flippy.setPosition(.18);
        double flipdis = flippydis.getDistance(DistanceUnit.INCH);
        telemetry.addData("flippydis: ", flipdis);
        telemetry.update();
        grabby.setPosition(.97);
        wristy.setPosition((float) ((float) .67 * .22));
        safeWaitSeconds(.25);//
        elbow1.setPosition(1);
        safeWaitSeconds(.5);//
        grabby.setPosition(.6);
        safeWaitSeconds(.2);//
        flippy.setPosition(.5);
        elbow1.setPosition(.91);
        return flipdis;
    }
    public Pose2d getaprilpose(int id, float x, float y, int angle, int y2, int x2) {
        if (thruPosition == THRU.TRUSS) {
            float xpose = 0;
            float ypose = 0;
            ElapsedTime slee = new ElapsedTime(SECONDS);
            slee.reset();
            AprilTagDetection desiredTag = null;
            while (xpose == 0 && slee.time() < 1) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((detection.id == id)) {
                            // Yes, we want to use this tag.
                            desiredTag = detection;
                            telemetry.addData("dis", String.valueOf(desiredTag.ftcPose.y), desiredTag.ftcPose.x);
                            telemetry.update();
                            xpose = (int) desiredTag.ftcPose.x;
                            ypose = (int) desiredTag.ftcPose.y;
                            xpose += 7;
                            ypose -= 5.5;
                            if (id <= 3) {
                                y += ypose;
                                x += xpose;
                            } else {
                                y -= ypose;
                                x -= xpose;
                            }

                        } else {
                            telemetry.addLine("Didn't read");
                            telemetry.update();
                        }
                    }

                }
            }
            if (xpose == 0){
                if (id == 1){
                    y2 += (int) distancechange( 7,"frontr");
                }
                if (id == 4){
                    y2 -= (int) distancechange( 7,"frontr");
                }
                if (id == 5){
                    y2 -= (int) distancechange( 7,"front");
                }
                if (id == 6){
                    y2 -= (int) distancechange( 7,"frontl");
                }
                if (id == 2){
                    y2 += (int) distancechange( 7,"front");
                }
                if (id == 3){
                    y2 += (int) distancechange( 7,"frontl");
                }
                y = y2;
                x = x2;
            }
        }else{
            float xpose = 0;
            float ypose = 0;
            ElapsedTime slee = new ElapsedTime(SECONDS);
            slee.reset();
            AprilTagDetection desiredTag = null;
            while (xpose == 0 && slee.time() < 1) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((detection.id == id)) {
                            // Yes, we want to use this tag.
                            desiredTag = detection;
                            telemetry.addData("dis", String.valueOf(desiredTag.ftcPose.y), desiredTag.ftcPose.x);
                            telemetry.update();
                            ypose = (int) desiredTag.ftcPose.y;
                            xpose = (int) desiredTag.ftcPose.x;
                            xpose -= 5.5;
                            ypose -= 7;
                            if (id <= 3) {
                                x += ypose;
                                y += xpose;
                            } else {
                                x += ypose;
                                y += xpose;
                            }
                        } else {
                            telemetry.addLine("Didn't read");
                            telemetry.update();
                        }
                    }

                }

            }
            if (xpose == 0) {
                if (id == 1) {
                    x2 += (int) distancechange(7, "frontr");
                }
                if (id == 4) {
                    x2 += (int) distancechange(7, "frontr");
                }
                if (id == 5) {
                    x2 += (int) distancechange(7, "front");
                }
                if (id == 6) {
                    x2 += (int) distancechange(7, "frontl");
                }
                if (id == 2) {
                    x2 += (int) distancechange(7, "front");
                }
                if (id == 3) {
                    x2 += (int) distancechange(7, "frontl");
                }
                x = x2;
                y = y2;
            }
        }
        return new Pose2d(x, y, Math.toRadians(angle));
    }
    public double dis(String side) {
        double dis = 1;
        if (side == "front") {
            dis = (sensorDistance.getDistance(DistanceUnit.INCH) + sensorDistance2.getDistance(DistanceUnit.INCH)) / 2;
        }
        if (side == "frontr") {
            dis = sensorDistance2.getDistance(DistanceUnit.INCH);
        }
        if (side == "frontl") {
            dis = sensorDistance.getDistance(DistanceUnit.INCH);
        }


        return dis;
    }

    public double distancechange(float num, String side){
        double dis = dis(side);
        dis = dis - num;
        return dis;
    }
    public double distancechanges(float num, String side){
        double dis = dis(side);
        if (dis < 10){
             dis = 0;
        }
        dis = dis - num;
        return dis;
    }
    public boolean twopix() {
        boolean pix = flippydis.getDistance(DistanceUnit.INCH) < 2;
        return pix;
    }
    public void drop(String side, String colr, double pix){
        if (pix == 99){
            grabby.setPosition(.75);
            front_left.setPower(-.5);
            front_right.setPower(-.5);
            rear_left.setPower(-.5);
            rear_right.setPower(-.5);

            sleep(180);
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
            armpose(2);
            front_left.setPower(.5);
            front_right.setPower(.5);
            rear_left.setPower(.5);
            rear_right.setPower(.5);

            sleep(120);
            grabby.setPosition(.97);
            sleep(100);
            front_left.setPower(-.5);
            front_right.setPower(-.5);
            rear_left.setPower(-.5);
            rear_right.setPower(-.5);
            sleep(200);
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
            b();
        } else if (pix < 2) {
            grabby.setPosition(.75);
            front_left.setPower(-.5);
            front_right.setPower(-.5);
            rear_left.setPower(-.5);
            rear_right.setPower(-.5);

            sleep(180);
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
            armpose(2);
            if (side == "frontr" || side == "front") {
                front_left.setPower(.71);
                front_right.setPower(-.71);
                rear_left.setPower(-.7);
                rear_right.setPower(.7);
                sleep(150);
            } else {
                front_left.setPower(-.71);
                front_right.setPower(.71);
                rear_left.setPower(.7);
                rear_right.setPower(-.7);
                sleep(150);
            }
            front_left.setPower(.5);
            front_right.setPower(.5);
            rear_left.setPower(.5);
            rear_right.setPower(.5);

            sleep(160);
            grabby.setPosition(.97);
            sleep(100);
            front_left.setPower(-.5);
            front_right.setPower(-.5);
            rear_left.setPower(-.5);
            rear_right.setPower(-.5);
            sleep(200);
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
            b();
        }else{
            grabby.setPosition(.97);
            front_left.setPower(-.5);
            front_right.setPower(-.5);
            rear_left.setPower(-.5);
            rear_right.setPower(-.5);

            sleep(180);
            front_left.setPower(0);
            front_right.setPower(0);
            rear_left.setPower(0);
            rear_right.setPower(0);
            b();
        }
    }
    public void skip(double stop){
        if (stop == 0){

        }else{
            safeWaitSeconds(stop);
        }
    }

    public void armout(int height){
        if (thruPosition == THRU.TRUSS && startPosition == START_POSITION.BLUE_FAR || startPosition == START_POSITION.RED_FAR) {
            if(startPosition == START_POSITION.BLUE_NEAR || startPosition == START_POSITION.RED_NEAR){
                armpose(-5);
            }
            else if(identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE){
                armpose(-1.9F);
            }
            else {
                armpose(-2.7F);
            }
            elbow1.setPosition(0);
            wristy.setPosition(((68 + -5 * 1.4 + 2 * 1.4) / 355) * .22);
        }else {

            if (height == 1) {
                armpose(-6);
            } else if (identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE) {
                armpose(-1.9F);
            } else if (startPosition == START_POSITION.BLUE_FAR || startPosition == START_POSITION.RED_FAR) {
                armpose(-2.7F);
            } else if (height == 2 || startPosition == START_POSITION.BLUE_NEAR || startPosition == START_POSITION.RED_NEAR) {
                armpose(2);
            }
            elbow1.setPosition(0);
            wristy.setPosition(((68 + -5 * 1.4 + 2 * 1.4) / 355) * .22);
        }
    }
    public double distancechange(int now, int later, String sensor){
        if(now - 7 - 8.375 > dis(sensor)){
            later -= (int) (now - (dis(sensor) + 8.375));
            later = (int) distancechange(later, sensor);
        }else{
            later = (int) distancechange(later, sensor);
        }

        return later;
    }
    public void armpose(float pose){
        double ticks = 22.76;
        double topose = 2;
        double counter8 = 0;
        double armAngle = ArmPos.getCurrentPosition() / ticks - 24;
        while (topose == 2) {
            armAngle = ArmPos.getCurrentPosition() / ticks - 24;
            if (pose < armAngle + 1 && pose > armAngle - 1) {// Stop arm movement within a 4 degree range
                Arm1.setPower(0);
                Arm2.setPower(0);
                if (counter8 < 14){
                    counter8 += 1;
                }else{
                    topose = 1;
                }
                break;

            } else if (pose > armAngle + 8 || pose < armAngle - 8) {//  Far and fast arm move into position within an infinite range
                if (pose < armAngle) {
                    Arm1.setPower(1);
                    Arm2.setPower(1);
                }
                if (pose > armAngle) {
                    Arm1.setPower(-1);
                    Arm2.setPower(-1);
                }

            } else { //Close and slow arm move into position if arm is in a 16 degree range
                if (pose < armAngle) {
                    Arm1.setPower(.2);
                    Arm2.setPower(.2);
                }
                if (pose > armAngle) {
                    Arm1.setPower(-.2);
                    Arm2.setPower(-.2);
                }

            }
        }
    }




    //Method to select starting position using X, Y, A, B buttons on gamepad
    public double selectStartingPosition() {
        double stop = 4.5;
        double hey = 0;
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        double rey = 1;
        double yyy = 1;
        double xyy = 1;
        double ayy = 1;
        double byy = 1;

        //******select start pose*****
        while(!isStopRequested()){

            telemetry.addData("Initializing FFTCOpenSourceAutonomousourceAutonomousnomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            while (rey < 4 && !isStopRequested()){
                telemetry.addData("Select Through Position using XY on Logitech on gamepad 1 and 2:","");
                telemetry.addData("    Code goes through the middle   ", "(X)");
                telemetry.addData("    Code goes near the wall ", "(Y)");
                telemetry.addData("","");
                telemetry.addData("Select Parking Position using AB on Logitech on gamepad 1 and 2:","");
                telemetry.addData("    Code parks near the middle   ", "(B)");
                telemetry.addData("    Code parks near the wall ", "(A)");
                if(gamepad1.x){
                    xyy = 2;
                }

                if(gamepad1.y){
                    yyy = 2;
                }
                if(gamepad1.a){
                    ayy = 2;
                }
                if(gamepad1.b ){
                    byy = 2;
                }
                if (byy == 2 && !gamepad1.b){
                    parkPosition = THRU.TRUSS;
                    rey += 1;
                    byy = 3;
                }
                if (ayy == 2 && !gamepad1.a){
                    parkPosition = THRU.WALL;
                    rey += 1;
                    ayy = 3;
                }
                if (xyy == 2 && !gamepad1.x){
                    thruPosition = THRU.TRUSS;
                    rey += 1;
                    xyy = 3;
                }
                if (yyy == 2 && !gamepad1.y){
                    thruPosition = THRU.WALL;
                    rey += 1;
                    yyy = 3;
                }
                if (rey == 3){
                    telemetry.clearAll();
                    rey = 4;
                }
                telemetry.update();
            }
            telemetry.addData("Parking near the ",parkPosition);
            telemetry.addData("Going through the obstacle near the ",thruPosition);
            telemetry.addData("To overide these values, press: ","left bumper and right bumper on both controllers");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Near   ", "(X / ▢)");
            telemetry.addData("    Blue Far ", "(Y / Δ)");
            telemetry.addData("    Red Far    ", "(B / O)");
            telemetry.addData("    Red Near  ", "(A / X)");
            telemetry.addData("    Time at slide  ", 18 + stop);
            telemetry.addData("    Time at fold  ", 24 + stop);
            if(stop == 6){
                telemetry.addData("This is very dangerous, are you sure?!!!", 24 + stop);
            }
            if(gamepad1.right_bumper){
                stop = 3;
            }
            if (gamepad1.left_bumper){
                stop = 0;
            }
            if (gamepad1.dpad_up){
                hey = 1;
            }
            if (gamepad1.dpad_down){
                hey = 2;
            }
            if (!gamepad1.dpad_down && hey == 2 && stop > 0){
                stop -= 1;
                hey = 0;
            }
            if (!gamepad1.dpad_up && hey == 1 && stop < 6){
                stop += 1;
                hey = 0;
            }
            if(gamepad1.x && gamepad2.x){
                startPosition = START_POSITION.BLUE_NEAR;
                break;
            }
            if(gamepad1.y && gamepad2.y){
                startPosition = START_POSITION.BLUE_FAR;
                break;
            }
            if(gamepad1.b && gamepad2.b){
                startPosition = START_POSITION.RED_FAR;
                break;
            }
            if(gamepad1.a && gamepad2.a){
                startPosition = START_POSITION.RED_NEAR;
                break;
            }
            if(gamepad2.left_bumper && gamepad2.right_bumper){
                telemetry.clearAll();
                ayy = 1;
                byy = 1;
                xyy = 1;
                yyy = 1;
                rey = 1;
            }
            telemetry.update();
        }
        telemetry.clearAll();
        return stop;
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)

                .build();
        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
        tfod.setMinResultConfidence(.80f);
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void setCamMode() {
        if (use_apriltag == 1) {
            myVisionPortal.setProcessorEnabled(aprilTag, true);
        } else {
            myVisionPortal.setProcessorEnabled(aprilTag, false);
        }
        if (use_tfod == 1) {
            myVisionPortal.setProcessorEnabled(tfod, true);
        } else {
            myVisionPortal.setProcessorEnabled(tfod, false);
        }
    }   // end method doCameraSwitching()

    // end class
    private void runTfodTensorFlow() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        //Camera placed between Left and Right Spike Mark on RED_FAR and BLUE_REAR If pixel not visible, assume Right spike Mark
        if (startPosition == START_POSITION.RED_FAR || startPosition == START_POSITION.BLUE_NEAR) {
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        } else { //RED_NEAR or BLUE_FAR
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        }
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (recognition.getLabel() == "Blue Prop" || recognition.getLabel() == "Red Prop") {
                if (x < 350) {
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                } else {
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                }
            } else {
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            }
        }// end for() loop

    }
}   // end class
