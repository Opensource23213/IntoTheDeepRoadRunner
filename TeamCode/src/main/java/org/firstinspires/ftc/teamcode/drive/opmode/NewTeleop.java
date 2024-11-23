package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.AsyncFollowingFSM;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Codethatworks", group="ABC Opmode")
//@Disabled
public class NewTeleop extends OpMode {
    private PIDController controller;
    private PIDController armcontroller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int slidestarget = 0;
    public static double armp = 0.01, armi = 0, armd = 0;

    public static double armf = 0.01;

    public static int armtarget = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private AnalogInput ArmPos = null;
    private Servo wristy = null;
    private Servo twisty = null;
    private CRServo gripspinny = null;

    double mode = 1;
    double county = 2;
    public double basketmove =1;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;

    double bottomlimit = .25;
    double slidebasket = 1800 ;
    double armbasket = 1725;
    double twistbasket = .5;
    double wristbasket = .2;
    double slidespecimen = .5;
    double armspecimen = 1380;
    double wristspecimen = .3;
    double twistspecimen = .5;
    double armspecimenpickup = 60;
    double wristspecimenpickup = .51;
    double xpress = 1;
    double times = 0;
    public Button1 buttons1 = null;
    public Button2 buttons2 = null;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    double rpress = 1;

    public RevTouchSensor limitfront;
    public RevTouchSensor limitfront2;
    public DigitalChannel limitwrist1;
    public DigitalChannel limitwrist2;
    public DigitalChannel limitwrist3;
    public double r1press = 1;
    public double armPose = 0;
    double press = 1;
    double slidesPose = 0;
    double offset = 0;
    double many = 1;
    double newpos = -312;
    double ypress = 1;
    double check = 1;
    double oldangle = 0;
    double oldtarget = 0;
    double dpress = 1;
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        TRAJECTORY_4,         // Then we're gonna wait a second
        TRAJECTORY_5,         // Finally, we're gonna turn again
        TRAJECTORY_6,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    AsyncFollowingFSM.State currentState = AsyncFollowingFSM.State.IDLE;
    double front = 0;
    double scored = 1;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-37, 61, Math.toRadians(90));
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    SampleMecanumDrive drive = null;
    double just = 0;
    boolean ready = false;
    double aapress = 1;
    ElapsedTime drivetime = new ElapsedTime();
    Trajectory trajectory1 = null;
    Trajectory turn = null;
    TrajectorySequence trajectory2 = null;
    TrajectorySequence trajectory3 = null;
    TrajectorySequence trajectory4 = null;
    TrajectorySequence trajectory5 = null;
    TrajectorySequence trajectory6 = null;
    TrajectorySequence trajectory7 = null;
    TrajectorySequence trajectory8 = null;
    TrajectorySequence trajectory12 = null;
    TrajectorySequence trajectory13 = null;
    Pose2d poseEstimate = null;
    double ticks = .002866;
    double conversion = ticks * armticks;
    public double inta = 1;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        gripspinny = hardwareMap.get(CRServo.class, "gripspinny");
        wristy = hardwareMap.get(Servo.class, "wrist");
        twisty = hardwareMap.get(Servo.class, "twist");
        imu = hardwareMap.get(IMU.class, "imu");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        limitwrist1 = hardwareMap.get(DigitalChannel.class, "limitwrist1");
        limitwrist2 = hardwareMap.get(DigitalChannel.class, "limitwrist2");
        limitwrist3 = hardwareMap.get(DigitalChannel.class, "limitwrist3");
        limitfront = hardwareMap.get(RevTouchSensor.class, "limitfront");
        limitfront2 = hardwareMap.get(RevTouchSensor.class, "limitfront2");
        gripspinny.setPower(0);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripspinny.setDirection(DcMotorSimple.Direction.REVERSE);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        buttons1 = new Button1();
        buttons2 = new Button2();
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        // Let's define our trajectories

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-35.5, 61), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(1, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-1, 29.5), Math.toRadians(-90))
                .build();
        trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .splineToConstantHeading(new Vector2d(3, 39), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-20, 48, Math.toRadians(95)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38.8, 61, Math.toRadians(95)), Math.toRadians(90))
                .build();
        trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
                .splineToConstantHeading(new Vector2d(-38.7, 61), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-1, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-2, 29.5), Math.toRadians(-90))
                .build();
        trajectory6 = drive.trajectorySequenceBuilder(trajectory5.end())
                .splineToConstantHeading(new Vector2d(3, 39), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-20, 48, Math.toRadians(100)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-41, 61, Math.toRadians(100)), Math.toRadians(90))
                .build();
        trajectory7 = drive.trajectorySequenceBuilder(trajectory6.end())
                .splineToConstantHeading(new Vector2d(-38.7, 61), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-1, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-2, 29.5), Math.toRadians(-90))
                .build();
        trajectory12 = drive.trajectorySequenceBuilder(new Pose2d(-40, 61, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-40, 44, Math.toRadians(90)))
                .build();
        trajectory13 = drive.trajectorySequenceBuilder(trajectory12.end())
                .lineToLinearHeading(new Pose2d(-40, 61, Math.toRadians(90)))
                .build();
        turn = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 0.1, Math.toRadians(180)))
                .build();
        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        boolean ready = false;
        armtarget = (int) ((1 - ArmPos.getVoltage() - .2) / ticks * armticks);
        slidestarget = 0;

    }
    double wristpose;
    double twistpose = .5;
    double lasttraj = 0;
    @Override
    public void loop() {
        arm();
        drop();
        spit();
        extra_in();
        basket();
        dropoff();
        eddy();
        basketdrop();
        spit2();
        check();
        intake();
        if(gamepad1.a){
            currentState = AsyncFollowingFSM.State.TURN_1;
            drive.followTrajectoryAsync(turn);
            turn = drive.trajectoryBuilder(poseEstimate)
                    .lineToLinearHeading(new Pose2d(poseEstimate.getX(), poseEstimate.getY() + .1, Math.toRadians(0)))
                    .build();

        }
        switch (currentState) {
            case TRAJECTORY_2:

                if(drivetime.time(TimeUnit.MILLISECONDS) > 800){
                    drive.followTrajectorySequenceAsync(trajectory13);
                    if(lasttraj == 4){
                        currentState = AsyncFollowingFSM.State.TRAJECTORY_4;
                    }
                    if(lasttraj == 6){
                        currentState = AsyncFollowingFSM.State.TRAJECTORY_6;
                    }
                }
                break;
            case TRAJECTORY_3:
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                if(drivetime.time(TimeUnit.MILLISECONDS) > 500 && (limitfront.isPressed() || limitfront2.isPressed()) && front == 0){
                    front = 1;
                    drivetime = new ElapsedTime();
                }
                if(front == 1){
                    armtarget = 1000;
                    gripspinny.setPower(-1);
                    if(armPose - armtarget < 100) {
                        ready = true;
                    }
                }else{
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                }
                if (ready) {
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_4;
                    drive.followTrajectorySequenceAsync(trajectory4);
                    drivetime = new ElapsedTime();
                    ready = false;
                    front = 0;
                    many += 1;
                    gripspinny.setPower(1);
                }
                break;
            case TRAJECTORY_4:
                // Check if the drive class is busy following the trajectory
                // Move on to the next state, TURN_1, once finished
                if(drivetime.time(TimeUnit.MILLISECONDS) > 1000 && drivetime.time(TimeUnit.MILLISECONDS) < 1200){
                    armtarget = (int) armspecimenpickup;
                    wristpose = wristspecimenpickup;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                }
                if(drivetime.time(TimeUnit.MILLISECONDS) > 1200 && (!limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState())) {
                    //raise arm to take off hook and bring arm in
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                    r1press = 2;
                    ready = true;
                }
                if(!drive.isBusy() && (limitwrist1.getState() || limitwrist3.getState())){
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_3;
                    drive.followTrajectorySequenceAsync(trajectory12);
                    lasttraj = 4;
                    drivetime = new ElapsedTime();
                }
                if (ready) {
                    ready = false;
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_5;
                    drive.followTrajectorySequenceAsync(trajectory5);
                    drivetime = new ElapsedTime();
                }
                break;
            case TRAJECTORY_5:
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                if(drivetime.time(TimeUnit.MILLISECONDS) > 700 && (limitfront.isPressed() || limitfront2.isPressed()) && front == 0){
                    front = 1;
                    drivetime = new ElapsedTime();
                }
                if(front == 1){
                    armtarget = 1000;
                    gripspinny.setPower(-1);
                    if(armPose - armtarget < 100) {
                        ready = true;
                    }
                }else{
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                }
                if (ready) {
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_6;
                    drive.followTrajectorySequenceAsync(trajectory6);
                    drivetime = new ElapsedTime();
                    many += 1;
                    ready = false;
                    front = 0;
                    gripspinny.setPower(1);
                }
                break;
            case TRAJECTORY_6:
                // Check if the drive class is busy following the trajectory
                // Move on to the next state, TURN_1, once finished
                if(drivetime.time(TimeUnit.MILLISECONDS) > 1000 && drivetime.time(TimeUnit.MILLISECONDS) < 1200){
                    armtarget = (int) armspecimenpickup;
                    wristpose = wristspecimenpickup;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                }
                if(drivetime.time(TimeUnit.MILLISECONDS) > 1200 && (!limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState())) {
                    //raise arm to take off hook and bring arm in
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                    r1press = 2;
                    ready = true;
                }
                if(!drive.isBusy() && (limitwrist1.getState() || limitwrist3.getState())){
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_3;
                    drive.followTrajectorySequenceAsync(trajectory12);
                    lasttraj = 6;
                    drivetime = new ElapsedTime();
                }
                if(ready && many >= times){
                    many = 0;
                    ready = false;
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_7;
                    drive.followTrajectorySequenceAsync(trajectory7);
                    drivetime = new ElapsedTime();
                }
                else if(ready) {
                    many += 1;
                    ready = false;
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_5;
                    drive.followTrajectorySequenceAsync(trajectory5);
                    drivetime = new ElapsedTime();
                }
                break;
            case TRAJECTORY_7:
                if(drivetime.time(TimeUnit.MILLISECONDS) > 700 && (limitfront.isPressed() || limitfront2.isPressed()) && front == 0){
                    front = 1;
                    drivetime = new ElapsedTime();
                }
                if(front == 1){
                    armtarget = 1000;
                    gripspinny.setPower(-1);
                    if(armPose - armtarget < 100) {
                        ready = true;
                    }
                    drivetime = new ElapsedTime();
                    front = 2;
                }
                else if (front == 2){

                }else{
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                }
                if(ready && drivetime.time(TimeUnit.MILLISECONDS) > 150){
                    currentState = AsyncFollowingFSM.State.IDLE;
                    drive.setPoseEstimate(new Pose2d());
                    drivetime = new ElapsedTime();
                    ready = false;
                    front = 0;
                    times = 3;
                    gripspinny.setPower(1);
                }
                break;
            case TURN_1:
                if(!drive.isBusy()){
                    currentState = AsyncFollowingFSM.State.IDLE;
                }
            case IDLE:
                // Do nothing in IDLE
                // currentState does not change once in IDLE
                // This concludes the autonomous program
                drive();
                buttons1.button();
                buttons2.button();
                break;
        }

        // Anything outside of the switch statement will run independent of the currentState

        // We update drive continuously in the background, regardless of state
        drive.update();
        // We update our lift PID continuously in the background, regardless of state

        // Read pose
        poseEstimate = drive.getPoseEstimate();

        // Continually write pose to `PoseStorage`
        PoseStorage.currentPose = poseEstimate;

        // Continually write pose to `PoseStorage`
        PoseStorage.currentPose = poseEstimate;

    }

    public void drive(){
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double angle = poseEstimate.getHeading();

        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        if(gamepad1.dpad_right){ // strafe left
            axial = 0;
            lateral = .5;
            yaw = 0;
        }
        else if(gamepad1.dpad_left){ // strafe right
            axial = 0;
            lateral = -.5;
            yaw = 0;
        }else if(gamepad1.dpad_down){ // strafe forward
            axial = -.5;
            lateral = 0;
            yaw = 0;
        }else if(gamepad1.dpad_up) { // Strafe backward
            axial = .5;
            lateral = 0;
            yaw = 0;
        }else{ // set control to the sticks
            axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
            yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);
        }
        double power_level = 1;
        if (gamepad1.ps) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetDeviceConfigurationForOpMode();
            imu.resetYaw();
            drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), Math.toRadians(0)));
        }

        //elbow1.setPosition(servo1pose);
        //elbow2.setPosition(servo2pose);

        double leftFrontPower = (axial + lateral + yaw) * power_level;
        double rightFrontPower = (axial - lateral - yaw) * power_level;
        double leftBackPower = (axial - lateral + yaw) * power_level;
        double rightBackPower = (axial + lateral - yaw) * power_level;

        // If the sticks are being used
        if(!gamepad1.dpad_left & !gamepad1.dpad_right & !gamepad1.dpad_up & !gamepad1.dpad_down) {
            double yaw_rad = /*orientation.getYaw(AngleUnit.RADIANS)*/angle + 3.14159 / 2;
            double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            axial = temp;
        }
        // Combie the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = (axial + lateral + yaw) * power_level;
        rightFrontPower = (axial - lateral - yaw) * power_level;
        leftBackPower = (axial - lateral + yaw) * power_level;
        rightBackPower = (axial + lateral - yaw) * power_level;
        // Normalize the values so no wheel power exceeds 00%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
        max = Math.max(max, abs(leftBackPower));
        max = Math.max(max, abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }//Arm code Shoulder
        front_left.setPower(leftFrontPower);
        front_right.setPower(rightFrontPower);
        rear_left.setPower(leftBackPower);
        rear_right.setPower(rightBackPower);
    }

    public void arm(){
        toplimit = 1406 + (2 * slideticks * 2);
        controller.setPID(p, i, d);
        slidesPose = -slides.getCurrentPosition() * 2;
        armd = -slides.getCurrentPosition() / slideticks * .03 / 19.6;
        armf = .001 + -slides.getCurrentPosition() / slideticks * .2 / 19.6;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
        if (-250 < slidesPose - slidestarget && slidesPose - slidestarget < 250 && (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1)) {
            double otherPose = -slides.getCurrentPosition() / slideticks;
            if (otherPose < bottomlimit && gamepad2.right_stick_y > 0) {
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            } else if (slidesPose > toplimit && gamepad2.right_stick_y < 0) {
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            } else {
                slides.setPower(gamepad2.right_stick_y * .75);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }
        } else {
            slides.setPower(-power);
        }
        armcontroller.setPID(armp, armi, armd);
        double armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        if (abs(abs(armPose) - abs(armtarget)) < 150 && (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1)) {
            if (armPose < newpos + 45 && gamepad2.left_stick_y > 0) {
                Arm1.setPower(0);
                Arm2.setPower(0);
                armtarget = (int) (armPose);
            } else {
                Arm1.setPower(gamepad2.left_stick_y / 2);
                Arm2.setPower(gamepad2.left_stick_y / 2);
                armtarget = (int) (armPose);
            }
        } else {
            Arm1.setPower(-armpower);
            Arm2.setPower(-armpower);
        }
        telemetry.addData("pose", armPose);
        telemetry.addData("target", armtarget);
        telemetry.addData("power", -armpower);
        telemetry.addData("now - target", slidesPose - slidestarget);
        telemetry.addData("pose", slidesPose);
        telemetry.addData("target", slidestarget);
        telemetry.addData("power", -power);
        telemetry.update();
        wristy.setPosition(wristpose);
        twisty.setPosition(twistpose);
    }
    public class Button2{
        String button = "";
        String nowbutton = "";
        String lastbutton = "";
        String type = "";
        public void button(){
            if (button == "") {
                if (gamepad2.a) {
                    button = "a";
                }
                else if(gamepad2.b){
                    button = "b";
                }
                else if (gamepad2.x){
                    button = "x";
                }
                else if (gamepad2.y){
                    button = "y";
                }
                else if (gamepad2.right_bumper){
                    button = "r1";
                }
                else if (gamepad2.left_bumper){
                    button = "l1";
                }
                else if (gamepad2.left_trigger > .4){
                    button = "l2";
                }
                else if (gamepad2.right_trigger > .4){
                    button = "r2";
                }else if (gamepad2.dpad_up){
                    button = "up";
                }
                else if (gamepad2.dpad_down){
                    button = "down";
                }
                else if (gamepad2.dpad_left){
                    button = "left";
                }
                else if (gamepad2.dpad_right){
                    button = "right";
                }else if (gamepad2.ps){
                    button = "ps";
                }
                else if (gamepad2.left_stick_button){
                    button = "l3";
                }
                else if (gamepad2.right_stick_button){
                    button = "r3";
                }
            }
            endbutton();
            ButtonControl();
        }
        public void ButtonControl(){
            if(nowbutton == "l2"){
                apress = 2;
                lastbutton = "";
                nowbutton = "";
                dpress = 1;
            }
            if(nowbutton == "r2"){
                armtarget = 0;
                slidestarget = 0;
                twistpose = .5;
                wristpose = 0;
                gripspinny.setPower(0);
                lastbutton = "";
                nowbutton = "";
                dpress = 1;
            }
            if(lastbutton == ""){
                if (nowbutton == "l3") {
                    armtarget = (int) (97 * armticks);
                    wristpose = .5;
                    lastbutton = "l3";
                    nowbutton = "";
                    dpress = 1;
                }
                if(nowbutton == "r3"){
                    armtarget = (int) armbasket;
                    wristpose = wristbasket;
                    lastbutton = "r3";
                    nowbutton = "";
                    dpress = 1;
                }
                if(dpress == 3 && nowbutton == "a"){
                    slidestarget = (int) oldtarget;
                    nowbutton = "";
                    lastbutton = "a";
                    wristpose = 0;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    dpress = 1;
                }
                else if(nowbutton == "a"){
                    //Arm goes out
                    armtarget = (int) (0 * armticks);
                    slidestarget = (int) (3 * slideticks * 2);
                    wristpose = 0;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    lastbutton = "a";
                    nowbutton = "";
                    dpress = 1;
                }
                if(nowbutton == "b"){
                    //Spit out
                    ypress = 1.5;
                    lastbutton = "";
                    nowbutton = "";
                    dpress = 1;
                }
                else if (nowbutton == "l1"){
                    //Arm moves to hang specimen
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "r1"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = (int) armspecimenpickup;
                    wristpose = wristspecimenpickup;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    lastbutton = "r1";
                    nowbutton = "";
                    dpress = 1;
                }
                else if(nowbutton == "y"){
                    //Arm moves to pick up stuff from eddy
                    basketmove = 2;
                    armtarget = (int) armbasket;
                    wristpose = wristbasket;
                    twistpose = .5;
                    lastbutton = "";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "l3"){
                if(nowbutton == "l3"){
                    armtarget = -200;
                    lastbutton = "";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "r1"){
                if(nowbutton == "r1" || !limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                    r1press = 2;
                    lastbutton = "l1";
                    nowbutton = "";

                }
            }
            else if(lastbutton == "r3"){
                if(nowbutton == "r3") {
                    slidestarget = (int) slidebasket;
                    basketmove = 2;
                    lastbutton = "";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "a"){
                if(!limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState()){
                    //brings arm back
                    wristpose = .5;
                    twistpose = .5;
                    armtarget = 0;
                    oldtarget = slidesPose;
                    slidestarget = 0;
                    dpress = 2;
                    aapress = 2;
                    nowbutton = "";
                    lastbutton = "";
                } else if(nowbutton == "a"){
                    //brings arm back
                    wristpose = .5;
                    twistpose = .5;
                    aapress = 2;
                    armtarget = 0;
                    slidestarget = 0;
                    nowbutton = "";
                    lastbutton = "";
                }
                if(nowbutton == "x"){
                    armtarget =  (int) (slidesPose * .1066 - 225 );
                    nowbutton = "";
                }
                if(nowbutton == "up"){
                    //wrist tilts up
                    if(wristpose < .5){
                        wristpose += .5;
                    }
                    nowbutton = "";
                }
                else if(nowbutton == "right"){
                    //twists right
                    if(twistpose < 1){
                        twistpose += .5;
                    }
                    nowbutton = "";
                }
                else if (nowbutton == "left"){
                    //twists left
                    if(twistpose > 0){
                        twistpose -= .5;
                    }
                    nowbutton = "";
                }
                else if(nowbutton == "down"){
                    //wrist tilts down
                    if(wristpose > 0){
                        wristpose -= .5;
                    }
                    if(wristpose == 0){
                        gripspinny.setPower(-1);
                    }
                    nowbutton = "";
                }
                else if(nowbutton == "b"){
                    //reverse intake
                    gripspinny.setPower(gripspinny.getPower() * -1);
                    nowbutton = "";
                }

            }

            else if(lastbutton == "l1"){
                /*if(nowbutton == "l1" || limitfront.isPressed()) {
                    //Arm drops block on the hang and goes back in
                    wristy.setPosition(0);
                    wristpose = 0;
                    gripspinny.setPower(-1);
                    xpress = 1.5;
                    lastbutton = "";
                    nowbutton = "";

                }*/
                if(nowbutton == "l1" || (limitfront.isPressed() || limitfront2.isPressed()) ) {
                    //Arm drops block on the hang and goes back in
                    armtarget = 1000;
                    gripspinny.setPower(-1);
                    xpress = 1.5;
                    lastbutton = "";
                    nowbutton = "";

                }

            }


        }
        public void endbutton(){
            if (!gamepad2.a && button == "a") {
                nowbutton = "a";
                button = "";
            }
            else if(!gamepad2.b && button == "b"){
                nowbutton = "b";
                button = "";
            }
            else if (!gamepad2.x && button == "x"){
                nowbutton = "x";
                button = "";
            }
            else if (!gamepad2.y && button == "y"){
                nowbutton = "y";
                button = "";
            }
            else if (!gamepad2.right_bumper && button == "r1"){
                nowbutton = "r1";
                button = "";
            }
            else if (!gamepad2.left_bumper && button == "l1"){
                nowbutton = "l1";
                button = "";
            }
            else if (gamepad2.left_trigger < .4 && button == "l2"){
                nowbutton = "l2";
                button = "";
            }
            else if (gamepad2.right_trigger < .4 && button == "r2"){
                nowbutton = "r2";
                button = "";
            }else if (!gamepad2.dpad_up && button == "up"){
                nowbutton = "up";
                button = "";
            }
            else if (!gamepad2.dpad_down && button == "down"){
                nowbutton = "down";
                button = "";
            }
            else if (!gamepad2.dpad_left && button == "left"){
                nowbutton = "left";
                button = "";
            }
            else if (!gamepad2.dpad_right && button == "right"){
                nowbutton = "right";
                button = "";
            }else if (!gamepad2.ps && button == "ps"){
                nowbutton = "ps";
                button = "";
            }
            else if (!gamepad2.left_stick_button && button == "l3"){
                nowbutton = "l3";
                button = "";
            }
            else if (!gamepad2.right_stick_button && button == "r3"){
                nowbutton = "r3";
                button = "";
            }
        }
    }
    public class Button1{
        String button = "";
        String nowbutton = "";
        String lastbutton = "";
        String type = "";
        public void button(){
            if (button == "") {
                if (gamepad1.a) {
                    button = "a";
                }
                else if(gamepad1.b){
                    button = "b";
                }
                else if (gamepad1.x){
                    button = "x";
                }
                else if (gamepad1.y){
                    button = "y";
                }
                else if (gamepad1.right_bumper){
                    button = "r1";
                }
                else if (gamepad1.left_bumper){
                    button = "l1";
                }
                else if (gamepad1.left_trigger > .4){
                    button = "l2";
                }
                else if (gamepad1.right_trigger > .4){
                    button = "r2";
                }else if (gamepad1.dpad_up){
                    button = "up";
                }
                else if (gamepad1.dpad_down){
                    button = "down";
                }
                else if (gamepad1.dpad_left){
                    button = "left";
                }
                else if (gamepad1.dpad_right){
                    button = "right";
                }else if (gamepad1.ps){
                    button = "ps";
                }
                else if (gamepad1.left_stick_button){
                    button = "l3";
                }
                else if (gamepad1.right_stick_button){
                    button = "r3";
                }
            }
            endbutton();
            ButtonControl();
        }
        public void ButtonControl(){
            if(lastbutton == "") {
                if (nowbutton == "r2") {
                    press = 2;
                    nowbutton = "";
                    lastbutton = "";
                    dpress = 1;
                }
                else if (nowbutton == "r1") {
                    inta = 2;
                    slidestarget = (int) (1.987235 * slideticks * 2);
                    wristpose = .186668;
                    twistpose = 0;
                    gripspinny.setPower(-1);
                    nowbutton = "";
                    lastbutton = "r1";
                } else if (nowbutton == "r3") {
                    ready = false;
                    currentState = AsyncFollowingFSM.State.TURN_1;
                    turn = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(poseEstimate.getX(), poseEstimate.getY() + .1, Math.toRadians(180)))
                            .build();
                    drive.followTrajectoryAsync(turn);
                    drivetime = new ElapsedTime();
                    lastbutton = "";
                    nowbutton = "";
                    dpress = 1;
                } else if (nowbutton == "l1") {
                    //Arm moves to pick up stuff from eddy
                    armtarget = (int) armspecimenpickup;
                    wristpose = wristspecimenpickup;
                    times = 3;
                    many = 1;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                } else if (nowbutton == "l2") {
                    //Arm moves to pick up stuff from eddy
                    armtarget = (int) armspecimenpickup;
                    wristpose = wristspecimenpickup;
                    times = 4;
                    many = 1;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    lastbutton = "l1";
                    nowbutton = "";
                    dpress = 1;
                }
            }
            else if(lastbutton == "r1"){
                if(nowbutton == "r1" || !limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    armtarget = 0;
                    slidestarget = 0;
                    twistpose = .5;
                    wristpose = 0;
                    gripspinny.setPower(0);
                    dpress = 1;
                    lastbutton = "";
                    nowbutton = "";

                }
            }

            else if(lastbutton == "l1"){
                if(!limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState()){
                    //raise arm to take off hook and bring arm in
                    ready = false;
                    currentState = AsyncFollowingFSM.State.TRAJECTORY_3;
                    drive.followTrajectorySequenceAsync(trajectory3);
                    drivetime = new ElapsedTime();
                    drive.setPoseEstimate(startPose);
                    gripspinny.setPower(0);
                    lastbutton = "";
                    nowbutton = "";

                }
                if(nowbutton == "r2"){
                    armtarget = 0;
                    slidestarget = 0;
                    gripspinny.setPower(0);
                    wristpose = 0;
                    nowbutton = "";
                    lastbutton = "";
                }
            }
        }
        public void endbutton(){
            if (!gamepad1.a && button == "a") {
                nowbutton = "a";
                button = "";
            }
            else if(!gamepad1.b && button == "b"){
                nowbutton = "b";
                button = "";
            }
            else if (!gamepad1.x && button == "x"){
                nowbutton = "x";
                button = "";
            }
            else if (!gamepad1.y && button == "y"){
                nowbutton = "y";
                button = "";
            }
            else if (!gamepad1.right_bumper && button == "r1"){
                nowbutton = "r1";
                button = "";
            }
            else if (!gamepad1.left_bumper && button == "l1"){
                nowbutton = "l1";
                button = "";
            }
            else if (gamepad1.left_trigger < .4 && button == "l2"){
                nowbutton = "l2";
                button = "";
            }
            else if (gamepad1.right_trigger < .4 && button == "r2"){
                nowbutton = "r2";
                button = "";
            }else if (!gamepad1.dpad_up && button == "up"){
                nowbutton = "up";
                button = "";
            }
            else if (!gamepad1.dpad_down && button == "down"){
                nowbutton = "down";
                button = "";
            }
            else if (!gamepad1.dpad_left && button == "left"){
                nowbutton = "left";
                button = "";
            }
            else if (!gamepad1.dpad_right && button == "right"){
                nowbutton = "right";
                button = "";
            }else if (!gamepad1.ps && button == "ps"){
                nowbutton = "ps";
                button = "";
            }
            else if (!gamepad1.left_stick_button && button == "l3"){
                nowbutton = "l3";
                button = "";
            }
            else if (!gamepad1.right_stick_button && button == "r3"){
                nowbutton = "r3";
                button = "";
            }
        }
    }
    public void check(){
        if(dpress == 2 && slidesPose < 100){
            if(!limitwrist1.getState() || !limitwrist2.getState() || !limitwrist3.getState()){
                dpress = 1;
            }else{
                dpress = 3;
            }
        }
    }
    public void extra_in(){
        if(r1press == 2){
            runtime = new ElapsedTime();
            r1press = 3;
        }else if(r1press == 3 && runtime.time(TimeUnit.MILLISECONDS) < 200){
            gripspinny.setPower(-1);
        }else if(r1press == 3){
            gripspinny.setPower(0);
            r1press = 1;
        }
    }
    public void drop(){
        if(xpress == 1.5){
            runtime = new ElapsedTime();
            xpress = 2;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 750){
            gripspinny.setPower(0);
            xpress = 1;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 250){
            gripspinny.setPower(1);
        }
    }
    public void basket(){
        if(basketmove == 2 && abs(slidesPose - slidestarget) < 100){
            rpress = 1.5;
            basketmove = 1;
        }
    }

    public void spit(){
        if(apress == 2){
            runtime = new ElapsedTime();
            apress = 3;
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 150){
            gripspinny.setPower(1);
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 500){
            gripspinny.setPower(-1);
        }else if(apress == 3){
            gripspinny.setPower(0);
            apress = 1;
        }
    }
    public void spit2() {
        if (aapress == 2) {
            runtime = new ElapsedTime();
            aapress = 3;
        } else if (aapress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 100) {
            gripspinny.setPower(1);
        } else if (aapress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 500) {
            gripspinny.setPower(-1);
        } else if (aapress == 3) {
            gripspinny.setPower(0);
            aapress = 1;
        }
    }
    public void dropoff(){
        if(ypress == 1.5){
            runtime = new ElapsedTime();
            gripspinny.setPower(1);
            ypress = 2;
        }else if(ypress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 200){
            gripspinny.setPower(0);
            ypress = 1;
        }

    }
    public void basketdrop(){
        if(rpress == 1.5){
            runtime = new ElapsedTime();
            gripspinny.setPower(.25);
            rpress = 2;
        }else if(rpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 500){
            gripspinny.setPower(0);
            rpress = 3;
        }else if(rpress == 3){
            slidestarget = 0;
            rpress = 1;
        }

    }
    public void eddy(){
        if(press == 2){
            armtarget = 0;
            slidestarget = (int) (1800 - (2*slideticks*2));
            wristpose = 0;
            press = 3;
        }else if(press == 3 && abs(abs(slidesPose) - abs(slidestarget)) < 700){
            ypress = 1.5;
            press = 4;
        }else if(press == 4 && ypress == 1){
            armtarget = 0;
            slidestarget = 0;
            press = 1;
        }
    }
    public void intake(){
        if(abs(slidesPose - slidestarget) < 100 && inta == 2){
            armtarget = -230;
            inta = 1;
        }
    }
}



