package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GoBuildaUtil
{
    HardwareMap hardware;
    public ColorSensor leftColor, rightColor;
    public DistanceSensor leftDistance, rightDistance;
        //gyro init
        public BNO055IMU imu;
        public Orientation angles;
    public char blockPos = ' ';

    //initializing motors
    public static DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    public static Servo grasp1, grasp2, angle1, angle2, rightCollection, leftCollection, foundation;

    public GoBuildaUtil() { }

    public void initializeAuto(HardwareMap hw)
    {
        hardware = hw;

        MotorFrontX = hardware.dcMotor.get("fx");
        MotorBackX = hardware.dcMotor.get("bx");
        MotorFrontY = hardware.dcMotor.get("fy");
        MotorBackY = hardware.dcMotor.get("by");
        motorExtend = hardware.dcMotor.get("extend");
        motorRotate = hardware.dcMotor.get("rotate");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grasp1 = hardware.servo.get("grasp1");
        grasp2 = hardware.servo.get("grasp2");
        rightCollection = hardware.servo.get("rightCollection");
        leftCollection = hardware.servo.get("leftCollection");
        foundation = hardware.servo.get("foundation");
        angle1 = hardware.servo.get("angle1");
        angle2 = hardware.servo.get("angle2");
    }

    public void initializeSensors(HardwareMap hw)
    {
        hardware = hw;


        leftColor = hardware.get(ColorSensor.class, "left");
        rightColor = hardware.get(ColorSensor.class, "right");

        leftColor.enableLed(true);
        rightColor.enableLed(true);

        leftDistance = hardware.get(DistanceSensor.class, "left");
        rightDistance = hardware.get(DistanceSensor.class, "right");
    }

    public void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (counts)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (counts)));

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (MotorBackX.isBusy() && MotorFrontX.isBusy()) { }
    }

    public void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (MotorBackY.isBusy() && MotorFrontY.isBusy()) { }
    }

    void botRotate(int distance, int power)
    {
        MotorBackY.setPower(power);
        MotorBackX.setPower(power);
        MotorFrontX.setPower(power);
        MotorFrontY.setPower(power);

        int COUNTS = inchesToCounts(distance);

        MotorFrontX.setTargetPosition(MotorFrontX.getCurrentPosition() + (COUNTS));
        MotorBackX.setTargetPosition(MotorBackX.getCurrentPosition() - (COUNTS));
        MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() + COUNTS);
        MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() - COUNTS);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(MotorBackY.isBusy() && MotorFrontY.isBusy() && MotorBackX.isBusy() && MotorFrontX.isBusy()) { }
    }

    public int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }
}
