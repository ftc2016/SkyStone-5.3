package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Red SkyStoneDragger", group = "Red")
public class RedSkyStoneDragger extends LinearOpMode
{

    //initalizing sensors
    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;
    //gyro init
    BNO055IMU imu;
    Orientation angles;

    //initializing motors
    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp1, grasp2, angle1, angle2, block1, foundation2;


    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeMotors();
        initSensors();

        waitForStart();

        sleep(250);
        detectBlock();

        collectBlock("down");
        sleep(250);

        moveY(-11, 0.2);
        moveX(-54, 0.2);

       collectBlock("up");

        moveX(79, 0.2);
        moveY(11,0.2);

        collectBlock("down");
        moveY(-11, 0.2);
        sleep(250);

        moveX(-80, 0.2);

        collectBlock("up");

        moveX(16, 0.2);
    }


    private void initializeMotors() {
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorFrontY = hardwareMap.dcMotor.get("fy");
        MotorBackY = hardwareMap.dcMotor.get("by");
        motorExtend = hardwareMap.dcMotor.get("extend");
        motorRotate = hardwareMap.dcMotor.get("rotate");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);


        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grasp1 = hardwareMap.servo.get("grasp1");
        grasp2 = hardwareMap.servo.get("grasp2");
        block1 = hardwareMap.servo.get("block1");
        foundation2 = hardwareMap.servo.get("foundation2");
        angle1 = hardwareMap.servo.get("angle1");
        angle2 = hardwareMap.servo.get("angle2");

        grasp1.setPosition(1);
        grasp2.setPosition(0);

    }

    private void initSensors() {
        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftColor.enableLed(true);
        rightColor.enableLed(true);

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
    }

    private int inchesToCounts(double inches) {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    private void detectBlock()
    {
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        char blockPos = ' ';                //0.713

        double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 50;

        while (current >= DESIRED_D) {
            current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", null);
            telemetry.update();

            MotorFrontY.setPower(0.3);
            MotorBackY.setPower(0.3);
        }

        MotorFrontY.setPower(-0.05);
        MotorBackY.setPower(-0.05);

        double leftNormalizedColors = (leftColor.green() + leftColor.red() + leftColor.blue()) / Math.pow(leftDistance.getDistance(DistanceUnit.MM), 2);
        double rightNormalizedColors = (rightColor.green() + rightColor.red() + rightColor.blue()) / Math.pow(rightDistance.getDistance(DistanceUnit.MM), 2);

        Log.i("LeftNormalizationValue", ""+leftNormalizedColors);
        Log.i("RightNormalizationValue", ""+rightNormalizedColors);

        sleep(500);

        if (rightNormalizedColors < 0.5 && leftNormalizedColors > 0.5) {
            moveX(16, 0.2);
            telemetry.addData("right", null);
        }

        if (leftNormalizedColors < 0.5 && rightNormalizedColors < 0.5) {
            moveX(-14, 0.2);
            telemetry.addData("center", null);
        }

        if (leftNormalizedColors < 0.5 && rightNormalizedColors > 0.5) {
            moveX(-10, 0.2);
            telemetry.addData("left", null);
        }
        telemetry.update();

        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void agaga(String position) {
        if (position.equals("release"))
        {
            angle1.setPosition(1);
            angle2.setPosition(0);

            grasp1.setPosition(1);
            grasp2.setPosition(0);

            angle1.setPosition(0);
            angle1.setPosition(1);
        }

        if (position.equals("grasp")) {
            grasp1.setPosition(0);
            grasp2.setPosition(1);
        }
    }

    void moveY(double inches, double power) {
        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy()) {
            telemetry.addData("Move Y method error = ", MotorFrontY.getTargetPosition() - (MotorFrontY.getCurrentPosition() + counts));
        }
    }

    private void moveX(double inches, double power) {
        int counts = inchesToCounts(inches);

        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (counts)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (counts)));

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Move X method error = ", MotorFrontX.getTargetPosition() - (MotorFrontX.getCurrentPosition() + counts));
            telemetry.update();
        }
    }

    void botRotate(int distance, int power) {
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
    }


    void armExtend(int counts, double power) {
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setPower(power);

        motorExtend.setTargetPosition(motorExtend.getCurrentPosition() + counts);

        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motorExtend.isBusy()) {
            telemetry.addData("extension error = ", motorExtend.getTargetPosition() - (motorExtend.getCurrentPosition() + counts));
            telemetry.update();
            if((motorExtend.getCurrentPosition()> motorExtend.getTargetPosition()-20) && (motorExtend.getCurrentPosition()<motorExtend.getTargetPosition()+20))
                break;
        }
    }

    void armRotate(int counts, double power) {

        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate.setPower(power);

        motorRotate.setTargetPosition(motorRotate.getCurrentPosition() + counts);

        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motorRotate.isBusy())
        {
            telemetry.addData(" rotation error = ", motorRotate.getTargetPosition() - (motorRotate.getCurrentPosition() + counts));
            telemetry.update();
            if((motorRotate.getCurrentPosition()> motorRotate.getTargetPosition()-20) && (motorRotate.getCurrentPosition()<motorRotate.getTargetPosition()+20))
                break;
        }
    }

    void collectBlock(String pos)
    {
        if(pos.equalsIgnoreCase("down"))
            block1.setPosition(0.6);

        if(pos.equalsIgnoreCase("up"))
            block1.setPosition(0);
    }
}