package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

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
@TeleOp(name="GripperTest", group="ABC Opmode")
//@Disabled
public class GripperTest extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0;

    public static double f = 0.01;

    public static int target = 0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor Arm = null;
    @Override
    public void runOpMode() {
        CRServo gripspinny = hardwareMap.get(CRServo.class, "gripspinny");
        Servo wristy = hardwareMap.get(Servo.class, "wrist");
        Servo twisty = hardwareMap.get(Servo.class, "twist");

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double a = 0;
        wristy.setPosition(.5);
        twisty.setPosition(.5);
        double apress = 1;
        double rpress = 0;
        double lpress = 0;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a && apress == 1){
                gripspinny.setPower(-1);
                wristy.setPosition(0);
                apress = 2;
            }else if(gamepad1.b){
                gripspinny.setPower(1);
            }
            else if (apress == 2 && !gamepad1.a){
                wristy.setPosition(.5);
                gripspinny.setPower(0);
                runtime = new ElapsedTime();
                apress = 1;
            }
            else if (runtime.time(TimeUnit.MILLISECONDS) < 150) {
                gripspinny.setPower(1);
            } else if(runtime.time(TimeUnit.MILLISECONDS) < 400){
                gripspinny.setPower(-1);
            }else{
                gripspinny.setPower(0);
            }
            if (gamepad1.right_bumper){
                rpress = 1;
            }
            if (gamepad1.left_bumper){
                lpress = 1;
            }
            if (!gamepad1.right_bumper && rpress == 1){
                if(twisty.getPosition()  == 1){
                    twisty.setPosition(.5);
                }else{
                    twisty.setPosition(1);
                }
                rpress = 0;
                lpress = 0;
            }
            if (!gamepad1.left_bumper && lpress == 1){
                if(twisty.getPosition()  == 0){
                    twisty.setPosition(.5);
                }else{
                    twisty.setPosition(0);
                }
                rpress = 0;
                lpress = 0;
            }

        }

    }
}



