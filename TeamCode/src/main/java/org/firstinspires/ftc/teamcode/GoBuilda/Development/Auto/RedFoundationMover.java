package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Autonomous(name = "Red FoundationMover", group = "Red")
public class RedFoundationMover extends LinearOpMode
{

    //initalizing sensors
    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;
        //gyro init
        BNO055IMU imu;
        Orientation angles;

    //initializing motors
    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp, angle, foundation1, foundation2;

    @Override
    public void runOpMode() throws InterruptedException
    {

    }

    private void initializeMotors()
    {
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorFrontY = hardwareMap.dcMotor.get("fy");
        MotorBackY = hardwareMap.dcMotor.get("by");

        MotorFrontX.setDirection(DcMotor.Direction.REVERSE);
        MotorBackX.setDirection(DcMotor.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotor.Direction.FORWARD);
        MotorBackY.setDirection(DcMotor.Direction.REVERSE);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFrontY.setZeroPowerBehavior(BRAKE);
        MotorBackY.setZeroPowerBehavior(BRAKE);
        MotorBackX.setZeroPowerBehavior(BRAKE);
        MotorFrontX.setZeroPowerBehavior(BRAKE);


        motorExtend = hardwareMap.dcMotor.get("extend");
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate = hardwareMap.dcMotor.get("rotate");
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRotate.setZeroPowerBehavior(BRAKE);

        grasp = hardwareMap.servo.get("grasp");
        foundation1 = hardwareMap.servo.get("foundation1");
        foundation2 = hardwareMap.servo.get("foundation2");
        angle = hardwareMap.servo.get("angle");

        grasp.setPosition(1);
        angle.setPosition(0.65);
        foundation1.setPosition(0);
        foundation2.setPosition(0);
    }

    private void initSensors()
    {
        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
    }

    private int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev/Servocity_Omnni_Circumference;

        return (int)(COUNTS_PER_REVOLUTION*inches);
    }

    void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running motor Y front and back", "Encoders");
            telemetry.update();
        }
    }

    void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (counts)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (counts)));

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Running motor X front and back", "Encoders");
            telemetry.update();
        }
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
    }
}
