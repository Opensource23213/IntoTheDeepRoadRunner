package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name="FullTeleop", group="ABC Opmode")
//@Disabled
public class FullTeleop extends LinearOpMode {
    double armtargetpose = 0;
    double servo1angle = 0;
    @Override
    public void runOpMode() {
        Systems drive = new Systems();
        drive.init();
        waitForStart();
        while (opModeIsActive()) {
            drive.Complete_drive_code();
        }
    }
    public class Arm {
        DcMotor Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        DcMotor Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        DcMotor ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        DcMotor slides = hardwareMap.get(DcMotor.class, "slides");

        public double p = 0.004, i = 0, d = 0;

        public double f = 0.01;

        public int target = 0;
        private PIDController controller = new PIDController(p, i, d);

        double armtargetpose = 0;
        double toplimit = -1500;
        double bottomlimit = 0;
        double ticks = 22.76; // ticks per degree in encoder
        Sensors sense = null;
        public double power = 0;
        public void config(String arm, Sensors hey){
            sense = hey;
            slides.setDirection(DcMotor.Direction.FORWARD);
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm1 .setDirection(DcMotor.Direction.REVERSE);
            ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmPos.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (arm == "arm") {
                ticks = 8192;
                p = .01;
                i = 0;
                d = 0;
                f = .01;
                target = 0;
                toplimit = 1800;
                bottomlimit = 0;
            }else{
                ticks = 537.7;
                p = .004;
                i = 0;
                d = 0;
                f = .001;
                target = 0;
                toplimit = 2000;
                bottomlimit = 0;
            }


        }
        public void movetoposition(String arm){
            double armPose;
            if(arm == "arm"){
                armPose = sense.armpose;
                double slidesPose = sense.slidespose;
                d = slidesPose/sense.slideticks * .03 / 19.6;
                f = .001 + slidesPose/sense.slideticks * .2 / 19.6;
            }else{
                armPose = sense.slidespose;
            }
            controller.setPID(p, i, d);
            double pid = controller.calculate(armPose, target);
            double ff =  Math.cos(Math.toRadians(target)) * f;
            power = pid + ff;
            if(arm == "arm"){
                Arm1.setPower(-power);
                Arm2.setPower(-power);
            }else{
                slides.setPower(-power);
            }

        }
    }
    public class Sensors {
        public double armpose = 0;
        public double slidespose = 0;
        public double slideticks = 537.7 / 4.75;
        public double armticks = 8192/360;
        DcMotor ArmPos = hardwareMap.get(DcMotor.class, "ArmPos");
        DcMotor slides = hardwareMap.get(DcMotor.class, "slides");
        public void getSensors(){
            slidespose = -slides.getCurrentPosition() * 2;
            armpose = ArmPos.getCurrentPosition();
        }
    }
    public class Systems {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        Servo wristy = hardwareMap.get(Servo .class, "wrist");
        Servo twisty = hardwareMap.get(Servo.class, "twist");
        CRServo gripspinny = hardwareMap.get(CRServo.class, "gripspinny");

        String button = "";
        String lastbutton = "";
        String nowbutton = "";
        String mode = "auto";
        Arm arm = null;
        Arm slides = null;
        Sensors sense = null;
        public double armpose = 0;
        public double slidespose = 0;
       public void init(){
           arm = new Arm();
           slides = new Arm();
           sense = new Sensors();
           arm.config("arm", sense);
           slides.config("slides", sense);
           imu.initialize(new IMU.Parameters(orientationOnRobot));
           front_left.setDirection(DcMotor.Direction.REVERSE);
           front_right.setDirection(DcMotor.Direction.FORWARD);
           rear_left.setDirection(DcMotor.Direction.REVERSE);
           rear_right.setDirection(DcMotor.Direction.FORWARD);
           front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           wristy.setPosition(.5);
           twisty.setPosition(.5);
       }
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
                }
            }
            endbutton();
            ButtonControl();
        }
        public void ButtonControl(){
            if(lastbutton == ""){
                if(nowbutton == "a"){
                    //Arm goes out
                    lastbutton = "a";
                    nowbutton = "";
                }
                else if(nowbutton == "r1"){
                    //Arm moves to basket position
                    lastbutton = "r1";
                    nowbutton = "";
                }
                else if (nowbutton == "l1"){
                    //Arm moves to hang specimen
                    arm.target = (int) (63.3 * sense.armticks);
                    slides.target = (int) (.5 * sense.slideticks * 2);
                    wristy.setPosition(.4);
                    twisty.setPosition(.5);
                    lastbutton = "l1";
                    nowbutton = "";
                }
                else if(nowbutton == "r2"){
                    //Arm moves to pick up stuff from eddy
                    lastbutton = "r2";
                    nowbutton = "";
                }
                else if(nowbutton == "l2"){
                    //hang
                    lastbutton = "hang";
                    nowbutton = "";
                    mode = "hang";
                }
            }
            else if(lastbutton == "r2"){
                if (nowbutton == "r1"){
                    //turn on intake
                    nowbutton = "";
                }
                else if(nowbutton == "l1"){
                    //raise arm to take off hook and bring arm in
                    lastbutton = "";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "a"){
                if(nowbutton == "a"){
                    //Arm goes back in
                    lastbutton = "";
                    nowbutton = "";
                }
                else if(nowbutton == "r1"){
                    //rotates wrist
                    nowbutton = "";
                }
                else if (nowbutton == "l1"){
                    //Reverses the intake(mainly to give stuff to eduardo)
                    nowbutton = "";
                }
                else if(nowbutton == "x"){
                    //Arm goes to pick up a colored block
                    nowbutton = "";
                }
                else if(nowbutton == "y"){
                    //Arm goes to pick up a yellow block
                    nowbutton = "";
                }
                else if (nowbutton == "r2"){
                    //Arm turns to manual mode
                    lastbutton = "ar2";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "r1"){
                if(nowbutton == "r1"){
                    //Arm drops block into basket and arm goes back in

                    lastbutton = "";
                    nowbutton = "";
                }

            }
            else if(lastbutton == "l1"){
                if(nowbutton == "l1") {
                    //Arm drops block on the hang and goes back in
                    arm.target = (int) (63.3 * sense.armticks);
                    slides.target = (int) (.5 * sense.slideticks * 2);
                    wristy.setPosition(.4);
                    twisty.setPosition(.5);
                    lastbutton = "";
                    nowbutton = "";

                }
            }

            else if(lastbutton == "ar2"){
                //Arm is now in manual mode(Right stick x controls side to side, while left stick y controls up/down)
                if (nowbutton == "r1"){
                    //Intake picks up block
                    //Arm goes back in
                    nowbutton = "";
                }
                else if(nowbutton == "l1"){
                    //twists wrist
                    nowbutton = "";
                }
                if(nowbutton == "a"){
                    //reverse intake
                    nowbutton = "";
                }
                else if(nowbutton == "r2"){
                    //turns auto back on
                    lastbutton = "";
                    nowbutton = "";
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
            }
        }
        public void Complete_drive_code(){
            if (mode == "auto") {
                sense.getSensors();
                button();
                arm.movetoposition("arm");
                slides.movetoposition("slides");
                drive();
            }
            else if(mode == "manual"){
                manual_arm();
                alternate_drive();
                button();
            }
            else if (mode == "hang") {
                hang();
            }
            telemetry.addData("button", button);
            telemetry.addData("nowbutton", nowbutton);
            telemetry.addData("lastbutton", lastbutton);
            telemetry.update();
        }
        public void hang(){
            //hang code
        }
        public void alternate_drive(){
            //Left and right movement while the arm is manual
        }
        public void manual_arm(){
            //Manual arm mode for picking up pixels without camera
        }
        public void drive() {
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
            telemetry.addData("target", armtargetpose);
            telemetry.addData("power", servo1angle);
            telemetry.update();

            //elbow1.setPosition(servo1pose);
            //elbow2.setPosition(servo2pose);

            double leftFrontPower = (axial + lateral + yaw) * power_level;
            double rightFrontPower = (axial - lateral - yaw) * power_level;
            double leftBackPower = (axial - lateral + yaw) * power_level;
            double rightBackPower = (axial + lateral - yaw) * power_level;

            // If the sticks are being used
            double yaw_rad = orientation.getYaw(AngleUnit.RADIANS) + 3.14159 / 2;
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
        }
    }
}



