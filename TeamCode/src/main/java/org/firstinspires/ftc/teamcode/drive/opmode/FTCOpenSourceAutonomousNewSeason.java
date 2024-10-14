

package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AsyncFollowingFSM;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

/**
 * FFTCOpenSourceAutonomouss Example for only vision detection using tensorflow and park
 */

@Autonomous(name = "NewAuto", group = "00-Autonomous", preselectTeleOp = "BucDays")
public class FTCOpenSourceAutonomousNewSeason extends LinearOpMode {
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
    private ElapsedTime drivetime = new ElapsedTime();

    private DcMotor slides = null;
    private DcMotor Arm1 = null;
    private DcMotor Arm2 = null;
    private DcMotor ArmPos = null;
    private Servo wristy = null;
    private Servo twisty = null;
    private CRServo gripspinny = null;

    double mode = 1;
    double basketmove =1;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;

    double bottomlimit = .25;
    double slidebasket = 1600 ;
    double armbasket = 2000;
    double twistbasket = .5;
    double wristbasket = .6;
    double slidespecimen = 0;
    double armspecimen = 1408;
    double wristspecimen = .18;
    double twistspecimen = .5;
    double armspecimenpickup = 0;
    double wristspecimenpickup = .39;
    double xpress = 1;
    public Codethatworks.Button buttons = null;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    public RevTouchSensor limitfront;
    public DigitalChannel limitwrist1;
    public DigitalChannel limitwrist2;
    public double r1press = 1;
    public double armPose = 0;
    double slidesPose = 0;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    double wristpose = .4;
    double twistpose = .5;
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    AsyncFollowingFSM.State currentState = AsyncFollowingFSM.State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(-3.5, 62.5, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        initializations();
        AsyncFollowingFSM.Lift lift = new AsyncFollowingFSM.Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-3.5, 32.5, Math.toRadians(-90)))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .splineToSplineHeading(new Pose2d(-3.5, 40, Math.toRadians(-90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-20, 50, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-36, 61, Math.toRadians(90)), Math.toRadians(90))
                .build();

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);
        boolean ready = false;
        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = AsyncFollowingFSM.State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.
            arm();
            drop();
            extra_in();
            basket();
            spit();
            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                    if(!limitfront.isPressed()){
                        wristy.setPosition(0);
                        wristpose = 0;
                        gripspinny.setPower(-1);
                        xpress = 1.5;
                        ready = true;
                    }
                    if (!drive.isBusy() && ready) {
                        currentState = AsyncFollowingFSM.State.TRAJECTORY_2;
                        drive.followTrajectorySequenceAsync(trajectory2);
                        drivetime = new ElapsedTime();
                        ready = false;
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if(drivetime.time(TimeUnit.MILLISECONDS) > 500 && drivetime.time(TimeUnit.MILLISECONDS) < 1000){
                        armtarget = (int) armspecimenpickup;
                        wristpose = wristspecimenpickup;
                        twistpose = .5;
                        gripspinny.setPower(-1);
                    }
                    if(!limitwrist1.getState() || !limitwrist2.getState()) {
                        //raise arm to take off hook and bring arm in
                        armtarget = (int) armspecimen;
                        slidestarget = (int) (slidespecimen * slideticks * 2);
                        wristpose = wristspecimen;
                        twistpose = twistspecimen;
                        r1press = 2;
                        ready = true;
                    }
                    if (!drive.isBusy() && ready) {
                        break;
                        //currentState = AsyncFollowingFSM.State.TURN_1;
                        //drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = AsyncFollowingFSM.State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = AsyncFollowingFSM.State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = AsyncFollowingFSM.State.TURN_2;
                        drive.turnAsync(turnAngle2);
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = AsyncFollowingFSM.State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
        public void Reach(String inout){
            if(inout == "in"){

            }else if(inout == "basket"){

            } else if (inout == "hook") {

            } else if (inout == "down") {

            } else if (inout == "out") {

            } else if (inout == "pickup") {

            }
        }
    }   // end runOpMode // end runOpMode()
    public void initializations(){
        controller = new PIDController(p, i, d);
        armcontroller = new PIDController(armp, armi, armd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        slides = hardwareMap.get(DcMotor.class, "slides"); //0 to -3.5 limit
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
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
        limitfront = hardwareMap.get(RevTouchSensor.class, "limitfront");
        wristy.setPosition(.4);
        twisty.setPosition(.5);
        gripspinny.setPower(0);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        armtarget = 0;
        slidestarget = 0;
    }
    public void arm(){
        toplimit = 1406;
        controller.setPID(p, i, d);
        slidesPose = -slides.getCurrentPosition() * 2;
        armd = -slides.getCurrentPosition()/slideticks * .03 / 19.6;
        armf = .001 + -slides.getCurrentPosition()/slideticks * .2 / 19.6;
        double pid = controller.calculate(slidesPose, slidestarget);
        double ff = Math.cos(Math.toRadians(slidestarget)) * f;
        double power = pid + ff;
        if(-250 < slidesPose - slidestarget && slidesPose - slidestarget < 250 && (gamepad2.right_stick_y > .1 || gamepad2.right_stick_y < -.1)){
            double otherPose = -slides.getCurrentPosition() / slideticks;
            if (otherPose < bottomlimit && gamepad2.right_stick_y > 0){
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }else if(otherPose > toplimit && gamepad2.right_stick_y < 0){
                slides.setPower(0);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }else{
                slides.setPower(gamepad2.right_stick_y/2);
                slidestarget = (int) (-slides.getCurrentPosition() * 2);
            }
        }else {
            slides.setPower(-power);
        }
        armcontroller.setPID(armp, armi, armd);
        armPose = ArmPos.getCurrentPosition();
        double armpid = controller.calculate(armPose, armtarget);
        double armff = Math.cos(Math.toRadians(armtarget)) * armf;
        double armpower = armpid + armff;
        if(-200 < armPose - armtarget && armPose - armtarget < 200 && (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1)){
            Arm1.setPower(gamepad2.left_stick_y/2);
            Arm2.setPower(gamepad2.left_stick_y/2);
            armtarget = (int) (ArmPos.getCurrentPosition());
        }else {
            Arm1.setPower(-armpower);
            Arm2.setPower(-armpower);
        }

        telemetry.addData("pose", armPose);
        telemetry.addData("target", armtarget);
        telemetry.addData("power", -armpower);
        telemetry.addData("pose", slidesPose - slidestarget);
        telemetry.addData("pose", slidesPose);
        telemetry.addData("target", slidestarget);
        telemetry.addData("power", -power);
        //telemetry.addData("state", !limitwrist2.getState());
        telemetry.update();

        wristy.setPosition(wristpose);
        twisty.setPosition(twistpose);

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
        if(basketmove == 2 && abs(armPose - armtarget) < 100){
            slidestarget = (int) slidebasket;
            basketmove = 1;
        }
    }
    public void spit(){
        if(apress == 2){
            runtime = new ElapsedTime();
            apress = 3;
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 100){
            gripspinny.setPower(1);
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 500){
            gripspinny.setPower(-1);
        }else if(apress == 3){
            gripspinny.setPower(0);
            apress = 1;
        }
    }
}   // end class
