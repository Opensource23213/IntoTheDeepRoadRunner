package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Codethatworks", group="ABC Opmode")
//@Disabled
public class Codethatworks extends OpMode {
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
    private DcMotor ArmPos = null;
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
    double armbasket = 1857;
    double twistbasket = .5;
    double wristbasket = .2;
    double slidespecimen = .5;
    double armspecimen = 1408;
    double wristspecimen = .3;
    double twistspecimen = .5;
    double armspecimenpickup = 20;
    double wristspecimenpickup = .51;
    double xpress = 1;
    public Button buttons = null;
    public double start = 0;
    public IMU imu = null;
    public DcMotor front_left = null;
    public DcMotor rear_left = null;
    public DcMotor front_right = null;
    public DcMotor rear_right = null;
    public double apress = 1;
    double rpress = 1;
    public RevTouchSensor limitfront;
    public DigitalChannel limitwrist1;
    public DigitalChannel limitwrist2;
    public DigitalChannel limitarm;
    public double r1press = 1;
    public double armPose = 0;
    double slidesPose = 0;
    double armreset = 1;
    double offset = 0;
    double newpos = -312;
    double ypress = 1;
    double check = 1;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    @Override
    public void init() {
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
        limitarm = hardwareMap.get(DigitalChannel.class, "limitarm");
        limitfront = hardwareMap.get(RevTouchSensor.class, "limitfront");
        wristy.setPosition(.5);
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

        buttons = new Button();
        armtarget = 0;
        slidestarget = 0;

    }
    double wristpose = .5;
    double twistpose = .5;
    @Override
    public void loop() {
        buttons.button();
        arm();
        drop();
        spit();
        drive();
        extra_in();
        basket();
        dropoff();
        basketdrop();
    }
    public void drive(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
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
        }

        //elbow1.setPosition(servo1pose);
        //elbow2.setPosition(servo2pose);

        double leftFrontPower = (axial + lateral + yaw) * power_level;
        double rightFrontPower = (axial - lateral - yaw) * power_level;
        double leftBackPower = (axial - lateral + yaw) * power_level;
        double rightBackPower = (axial + lateral - yaw) * power_level;

        // If the sticks are being used
        if(!gamepad1.dpad_left & !gamepad1.dpad_right & !gamepad1.dpad_up & !gamepad1.dpad_down) {
            double yaw_rad = orientation.getYaw(AngleUnit.RADIANS) + 3.14159 / 2;
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
        if(armreset == 1) {
            toplimit = 1406;
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
                } else if (otherPose > toplimit && gamepad2.right_stick_y < 0) {
                    slides.setPower(0);
                    slidestarget = (int) (-slides.getCurrentPosition() * 2);
                } else {
                    slides.setPower(gamepad2.right_stick_y *.75);
                    slidestarget = (int) (-slides.getCurrentPosition() * 2);
                }
            } else {
                slides.setPower(-power);
            }
            armcontroller.setPID(armp, armi, armd);
            armPose = ArmPos.getCurrentPosition() + offset;
            double armpid = controller.calculate(armPose, armtarget);
            double armff = Math.cos(Math.toRadians(armtarget)) * armf;
            double armpower = armpid + armff;
            if (abs(abs(armPose) - abs(armtarget)) < 150 && (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1)) {
                if (armPose < newpos + 45 && gamepad2.left_stick_y > 0) {
                    Arm1.setPower(0);
                    Arm2.setPower(0);
                    armtarget = (int) (ArmPos.getCurrentPosition() + offset);
                } else {
                    Arm1.setPower(gamepad2.left_stick_y / 2);
                    Arm2.setPower(gamepad2.left_stick_y / 2);
                    armtarget = (int) (ArmPos.getCurrentPosition() + offset);
                }
            } else {
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
            telemetry.addData("state", !limitarm.getState());
            telemetry.update();

            wristy.setPosition(wristpose);
            twisty.setPosition(twistpose);
        }else{
            reset();
        }

    }
    public class Button{
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
            if(nowbutton == "ps"){
                armreset = 2;
                armtarget = 0;
                lastbutton = "";
                nowbutton = "";
            }
            if(nowbutton == "l2"){
                apress = 2;
                lastbutton = "";
                nowbutton = "";
            }
            if(nowbutton == "r2"){
                armtarget = 0;
                slidestarget = 0;
                twistpose = .5;
                wristpose = 0;
                gripspinny.setPower(0);
                lastbutton = "";
                nowbutton = "";
            }
            if(lastbutton == ""){
                if(nowbutton == "l3"){
                    armtarget = (int) (97 * armticks);
                    wristpose = .5;
                    lastbutton = "l3";
                    nowbutton = "";
                }
                if(nowbutton == "r3"){
                    armtarget = (int) armbasket;
                    wristpose = wristbasket;
                    lastbutton = "r3";
                    nowbutton = "";
                }
                if(nowbutton == "a"){
                    //Arm goes out
                    armtarget = (int) (0 * armticks);
                    slidestarget = (int) (3 * slideticks * 2);
                    wristpose = 0;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    lastbutton = "a";
                    nowbutton = "";
                }
                if(nowbutton == "b"){
                    //Spit out
                    ypress = 1.5;
                    lastbutton = "";
                    nowbutton = "";
                }
                else if (nowbutton == "l1"){
                    //Arm moves to hang specimen
                    armtarget = (int) armspecimen;
                    slidestarget = (int) (slidespecimen * slideticks * 2);
                    wristpose = wristspecimen;
                    twistpose = twistspecimen;
                    lastbutton = "l1";
                    nowbutton = "";
                }
                else if(nowbutton == "r1"){
                    //Arm moves to pick up stuff from eddy
                    armtarget = (int) armspecimenpickup;
                    wristpose = wristspecimenpickup;
                    twistpose = .5;
                    gripspinny.setPower(-1);
                    lastbutton = "r1";
                    nowbutton = "";
                }
                else if(nowbutton == "y"){
                    //Arm moves to pick up stuff from eddy
                    basketmove = 2;
                    armtarget = (int) armbasket;
                    wristpose = wristbasket;
                    twistpose = .5;
                    lastbutton = "";
                    nowbutton = "";
                }
            }
            else if(lastbutton == "r1"){
                if(nowbutton == "r1" || !limitwrist1.getState() || !limitwrist2.getState()){
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
                if(nowbutton == "a" || !limitwrist1.getState() || !limitwrist2.getState()){
                    //brings arm back
                    wristpose = .5;
                    twistpose = .5;
                    apress = 2;
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
            else if(lastbutton == "l3"){
                if(nowbutton == "l3"){
                    armtarget = -250;
                    lastbutton = "";
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
                if(nowbutton == "l1" || limitfront.isPressed() ) {
                    //Arm drops block on the hang and goes back in
                    armtarget = 1076;
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
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 100){
            gripspinny.setPower(1);
        }else if(apress == 3 && runtime.time(TimeUnit.MILLISECONDS) < 500){
            gripspinny.setPower(-1);
        }else if(apress == 3){
            gripspinny.setPower(0);
            apress = 1;
        }
    }
    public void reset(){
        if(limitarm.getState()) {
            Arm1.setPower(.1);
            Arm2.setPower(.1);
            wristy.setPosition(.5);
        }else{
            ArmPos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm1.setPower(0);
            Arm2.setPower(0);
            offset = newpos;
            armreset = 1;
        }
    }
    public void dropoff(){
        if(ypress == 1.5){
            runtime = new ElapsedTime();
            gripspinny.setPower(1);
            ypress = 2;
        }else if(ypress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 500){
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
}



