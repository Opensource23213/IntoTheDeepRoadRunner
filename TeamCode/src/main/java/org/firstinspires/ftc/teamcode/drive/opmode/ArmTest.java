package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="ArmTest", group="ABC Opmode")
//@Disabled
public class ArmTest extends OpMode {
    private PIDController controller;
    private PIDController armcontroller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int target = 0;
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

    double mode = 2;
    double slideratio = 2;
    double slideticks = 103.8 * slideratio / 4.75;
    double armticks = 8192 / 360;
    double toplimit = 18.6;
    double bottomlimit = .25;
    double slidebasket = 18.77;
    double armbasket = 90;
    double twistbasket = .5;
    double wristbasket = .86;
    double slidespecimen = 0;
    double armspecimen = 61 * armticks;
    double wristspecimen = .28;
    double twistspecimen = .5;
    double armspecimenpickup = -1.8;
    double wristspecimenpickup = .63;
    double xpress = 1;

    @Override
    public void init() {
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
        wristy.setPosition(.5);
        twisty.setPosition(.5);
        gripspinny.setPower(0);
        slides.setDirection(DcMotor.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripspinny.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    double wristpose = .5;
    double twistpose = .5;
    @Override
    public void loop() {
        if(gamepad1.a && gamepad1.left_trigger > .4){
            mode = 1;
        }
        if (gamepad1.b && gamepad1.left_trigger > .4){
            mode = 2;
        }
        if(gamepad1.y){
            wristy.setPosition(.4);
            wristpose = wristspecimen;
            armtarget = (int) armspecimen;
            target = (int) (slidespecimen * slideticks * 10);
            mode = 1;
        }
        if(gamepad1.b){
            wristy.setPosition(.094);
            wristpose = .094;
            xpress = 1.5;
        }
        if(gamepad1.x){
            wristy.setPosition(.4);
            wristpose = .4;
            armtarget = (int) (44 * armticks);
            mode = 1;
            xpress = 1.5;
        }
        if(xpress == 1.5){
            runtime = new ElapsedTime();
            xpress = 2;
        }else if(xpress == 2 && runtime.time(TimeUnit.MILLISECONDS) > 250){
            gripspinny.setPower(1);
            xpress = 3;
        }else if(xpress == 3 && runtime.time(TimeUnit.MILLISECONDS) > 750){
            gripspinny.setPower(0);
            xpress = 1;
        }
        if(gamepad2.y){
            gripspinny.setPower(-1);
        } else if (gamepad2.x) {
            gripspinny.setPower(1);
        }else if(xpress != 2){
            gripspinny.setPower(0);
        }
        if(mode == 1) {
            controller.setPID(p, i, d);
            double slidesPose = -slides.getCurrentPosition() * 10;
            armd = slidesPose/slideticks * .03 / 19.6;
            armf = .001 + slidesPose/slideticks * .2 / 19.6;
            double pid = controller.calculate(slidesPose, target);
            double ff = Math.cos(Math.toRadians(target)) * f;

            double power = pid + ff;

            slides.setPower(-power);

            /*telemetry.addData("pose", slidesPose);
            telemetry.addData("target", target);
            telemetry.addData("power", power);
            telemetry.update();*/
            armcontroller.setPID(armp, armi, armd);
            double ticks = .002866;
            double conversion = ticks * armticks;
            double armPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
            double armpid = controller.calculate(armPose, armtarget);
            double armff = Math.cos(Math.toRadians(armtarget)) * armf;

            double armpower = armpid + armff;

            Arm1.setPower(-armpower);
            Arm2.setPower(-armpower);
            if(-armpower < .18 && -armpower > -.18 && -power < .18 && -power > -.18){
                mode = 2;
            }

            telemetry.addData("pose", slidesPose);
            telemetry.addData("target", target);
            telemetry.addData("power", power);
            telemetry.update();
        }else{
            double slidesPose = -slides.getCurrentPosition() / slideticks;
            double ticks = .002866;
            double conversion = ticks * armticks;
            double ArmPose = (1 - ArmPos.getVoltage() - .2) / ticks * armticks;
            if (slidesPose < bottomlimit && gamepad1.left_stick_y > 0){
                slides.setPower(0);
            }else if(slidesPose > toplimit && gamepad1.left_stick_y < 0){
                slides.setPower(0);
            }else{
                slides.setPower(gamepad1.left_stick_y);
            }

            Arm1.setPower(gamepad1.right_stick_y/2);
            Arm2.setPower(gamepad1.right_stick_y/2);
            if (wristpose >= 0 && gamepad2.left_stick_y > 0){
                wristpose -= gamepad2.left_stick_y * .01;
            }
            if (twistpose >= 0 && gamepad2.right_stick_x > 0){
                twistpose -= gamepad2.right_stick_x * .01;
            }
            if (wristpose <= 1 && gamepad2.left_stick_y < 0){
                wristpose -= gamepad2.left_stick_y * .01;
            }
            if (twistpose <= 1 && gamepad2.right_stick_x < 0){
                twistpose -= gamepad2.right_stick_x * .01;
            }
            twisty.setPosition(twistpose);
            wristy.setPosition(wristpose);
            telemetry.addData("slidepose", slidesPose);
            telemetry.addData("armpose", ArmPose);
            telemetry.addData("wristpose", wristy.getPosition());
            telemetry.addData("twistpose", twisty.getPosition());
            telemetry.update();
        }
    }
}



