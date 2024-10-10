package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
  This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
  the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
  of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
  class is instantiated on the Robot Controller and executed.

  This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
  It includes all the skeletal structure that all linear OpModes contain.

  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="DeepOp", group="ABC Opmode")
//@Disabled
public class DeepOp extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int target = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //private DcMotor Arm = null;
    @Override
    public void runOpMode() {
        //AnalogInput servoencoder1 = hardwareMap.get(AnalogInput.class, "servoencoder1");
        //AnalogInput servoencoder2 = hardwareMap.get(AnalogInput.class, "servoencoder2");
        //Servo elbow1 = hardwareMap.get(Servo.class, "elbow1");
        //Servo elbow2 = hardwareMap.get(Servo.class, "elbow2");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        DcMotor Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        DcMotor Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        DcMotor ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        DigitalChannel limit = hardwareMap.get(DigitalChannel.class, "limit");
        //DcMotor Arm = hardwareMap.get(DcMotor.class, "Arm");
        controller = new PIDController(p, i, d);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /*Arm.setDirection(DcMotor.Direction.FORWARD);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        double servo1pose = 0;
        double servo2pose = 0;
        double wristpose = 0;
        double elbowpose = 0;
        double currentpose = 1;
        double lastpose = 1;
        double armpose = 0;
        double placepose = -1350;
        double armtargetpose = 0;
        double toplimit = -1500;
        double bottomlimit = 0;
        double servo1angle = 0;
        double position = 0;
        double topose = 1;
        double ticks = 22.76; // ticks per degree in encoder
        double out = 0;
        double in = 0;
        double hang = 0;
        double bucket = 0;

        //elbow1.setPosition(servo1pose);
        //elbow2.setPosition(servo2pose);
        /*GamepadEx gamepadEx = new GamepadEx(gamepad1);
        ButtonReader aButton = new ButtonReader(gamepadEx, GamepadKeys.Button.A);
        ButtonReader bButton = new ButtonReader(gamepadEx, GamepadKeys.Button.B);
        ButtonReader xButton = new ButtonReader(gamepadEx, GamepadKeys.Button.X);
        ButtonReader yButton = new ButtonReader(gamepadEx, GamepadKeys.Button.Y);
        ButtonReader rightButton = new ButtonReader(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader leftButton = new ButtonReader(gamepadEx, GamepadKeys.Button.LEFT_BUMPER);*/
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        rear_left.setDirection(DcMotor.Direction.REVERSE);
        rear_right.setDirection(DcMotor.Direction.FORWARD);
        Arm2 .setDirection(DcMotor.Direction.REVERSE);
        ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double axial = -gamepad1.left_stick_y / (gamepad1.left_trigger + 1);  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x / (gamepad1.left_trigger + 1);
            double yaw = gamepad1.right_stick_x / (gamepad1.left_trigger + 1);
            double power_level = 1;
            if (gamepad1.ps) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                imu.resetDeviceConfigurationForOpMode();
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Middle Button (Logic) on Gamepad to reset\n");
            }
            if(gamepad1.a){
                wristpose = 0;
                elbowpose = 260;
                currentpose = 2;
            }
            if(gamepad1.left_bumper){
                wristpose = 0;
                elbowpose = 0;
                currentpose = 1;
            }
            if(gamepad1.right_bumper){
                wristpose = 70;
                elbowpose = 260;
                currentpose = 2;
            }
            if (currentpose == 1){
                armtargetpose = bottomlimit;
            }
            else{
                armtargetpose = placepose;
            }
            controller.setPID(p, i, d);
            //double armPose = Arm.getCurrentPosition();
            //double pid = controller.calculate(armPose, armtargetpose);
            double ff = Math.cos(Math.toRadians(armtargetpose)) * f;
            //servo1angle = 338 + servoencoder1.getVoltage() / 3.3 * 360 * -1;
            //double power = pid + ff;

            //Arm.setPower(power);


            //telemetry.addData("pose", armPose);
            telemetry.addData("target", armtargetpose);
            telemetry.addData("power", servo1angle);
            telemetry.update();

            servo1pose = (wristpose * .83333 + elbowpose * .83333) / 360;
            servo2pose = (elbowpose * .83333 - wristpose * .83333) / 360;
            //elbow1.setPosition(servo1pose);
            //elbow2.setPosition(servo2pose);

            double leftFrontPower = (axial + lateral + yaw) * power_level;
            double rightFrontPower = (axial - lateral - yaw) * power_level;
            double leftBackPower = (axial - lateral + yaw) * power_level;
            double rightBackPower = (axial + lateral - yaw) * power_level;

            // If the sticks are being used
            double yaw_rad = orientation.getYaw(AngleUnit.RADIANS) + 3.14159/2;
            double temp = axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            lateral = -axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //double temp = axial * Math.cos(yaw_rad) + lateral * Math.sin(yaw_rad);
            //lateral = -axial * Math.sin(yaw_rad) + lateral * Math.cos(yaw_rad);
            axial = temp;
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
            if (gamepad2.a){
                position = 50;
                topose = 2;
            }
            if (gamepad2.b){
                position = 0;
                topose = 2;
            }
            if(gamepad2.ps){
                topose = 3;
            }
            double armAngle = ArmPos.getCurrentPosition() / ticks - 22;
            if (topose == 1) {
                if ((gamepad2.left_stick_y > .2 || gamepad2.left_stick_y < -.2)) { // Normal arm movement
                    Arm1.setPower(gamepad2.left_stick_y * -1);
                    Arm2.setPower(gamepad2.left_stick_y * -1);
                    position = armAngle;
                }else{
                    Arm1.setPower(0);
                    Arm2.setPower(0);
                }
            } else if(topose == 2){ // Arm move to position code
                controller.setPID(p, i, d);
                double armPose = ArmPos.getCurrentPosition();
                double pid = controller.calculate(armPose, ((position + 22) * ticks));
                ff = Math.cos(Math.toRadians(position + 22)) * f;
                double power = pid + ff;
                Arm1.setPower(power);
                Arm2.setPower(power);
                if (gamepad2.left_stick_y > .2 || gamepad2.left_stick_y < -.2 || gamepad2.left_stick_y > .2 || gamepad2.left_stick_y < -.2 || gamepad2.dpad_down || gamepad2.dpad_up) {
                    topose = 1;
                }
            }else if(topose == 3){
                Arm1.setPower(-.1);
                Arm2.setPower(-.1);
                if(!limit.getState()){
                    ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    position = -8.8;
                    topose = 2;
                }
            }else{
                Arm1.setPower(-.1);
                Arm2.setPower(-.1);
                if(!limit.getState()){
                    ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Arm1.setPower(0);
                    Arm2.setPower(0);
                }
            }
        }

    }
}



