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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name="NewEncoder", group="ABC Opmode")
//@Disabled
public class NewEncoder extends OpMode {
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        Arm2 = hardwareMap.get(DcMotor.class, "Arm2");
        ArmPos = hardwareMap.get(AnalogInput.class, "ArmPos");
        Arm1 .setDirection(DcMotor.Direction.REVERSE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    double wristpose = .5;
    double twistpose = .5;
    double ticks = .002866;
    double conversion = ticks * armticks;
    @Override
    public void loop() {
        double zero = .971;
        double ninety = .713;
        double ArmAngle = (1 - ArmPos.getVoltage() - .2006) / ticks * armticks;
        Arm1.setPower(gamepad1.left_stick_y/2);
        Arm2.setPower(gamepad1.left_stick_y/2);
        telemetry.addData("Current Voltage", ArmPos.getVoltage());
        telemetry.addData("Max Voltage", ArmPos.getMaxVoltage());
        telemetry.update();
    }
}



