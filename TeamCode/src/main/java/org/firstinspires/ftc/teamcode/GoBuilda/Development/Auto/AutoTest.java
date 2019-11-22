package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

@Autonomous(name="Scrim 3 Blue")
public class AutoTest extends LinearOpMode
{

    //initalizing sensors
    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;
        //gyro init
        BNO055IMU imu;
        Orientation angles;

    //initializing motors
    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp, angle, foundation;

    //initializing variables
    int i=1;
    boolean isLeft = false, isRight = false;
    final int redThreshold = 1000, greenThreshold = 2000;



    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeMotors();
        initSensors();

        waitForStart();

        detectBlock();
        moveX(54, 0.2);
        grasp.setPosition(0);
        sleep(100);
        moveX(-79, 0.2);
        collectSky2();

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

        motorExtend = hardwareMap.dcMotor.get("extend");
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate = hardwareMap.dcMotor.get("rotate");
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grasp = hardwareMap.servo.get("grasp");
        foundation = hardwareMap.servo.get("foundation");
        angle = hardwareMap.servo.get("angle");

        grasp.setPosition(1);
        angle.setPosition(0.65);
        foundation.setPosition(0);
    }

    private void initSensors()
    {
        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");

        leftDistance = hardwareMap.get(DistanceSensor.class, "leftColor");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightColor");
    }

    void initGyro()
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }


    Orientation getAngles()
    {
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
    }

    float getRotationCorrection(float desiredAngle)
    {
        float motorPowerCorrection;
        float angle_error;
        angles = getAngles();

        angle_error = desiredAngle - (float)angles.firstAngle;
        motorPowerCorrection = -0.025f * angle_error;
        motorPowerCorrection = min(max(motorPowerCorrection, -1.0f),1.0f);
        return motorPowerCorrection;
    }

    void rotationCorrection()
    {
        float desired_angle = 0.0f;
        float angle_error = 10000.0f;

        telemetry.addData("Heading : ",angles.firstAngle);
        telemetry.update();

        angles = getAngles();

        float motorPower = 0;

        while (abs(angle_error) > 1.0f)
        {
            motorPower = getRotationCorrection(desired_angle);
            MotorFrontX.setPower(motorPower);
            MotorFrontY.setPower(motorPower);
            MotorBackX.setPower(-motorPower);
            MotorBackY.setPower(-motorPower);
        }

        telemetry.addData("Heading : ",angles.firstAngle);
        telemetry.update();
    }

    private int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev/Servocity_Omnni_Circumference;

        return (int)(COUNTS_PER_REVOLUTION*inches);
    }

    private void detectBlock()
    {
        double distance = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM))/2.0;
        while(distance >= 60)
        {
            moveY(0.1, 0.1);
        }
        sleep(500);
        if(leftColor.red()<redThreshold && leftColor.green()<greenThreshold)
        {
            isLeft = true;
            telemetry.addData("leftColor", null);
            telemetry.update();
        }

        if (rightColor.red()<redThreshold && leftColor.green()<greenThreshold)
        {
            isRight = true;
            telemetry.addData("rightColor", null);
            telemetry.update();
        }

        if(isRight)
        {
            moveX(5,0.5);
            moveY(-2, 0.5);
            collectSky();
        }
        else if(isLeft)
        {

            moveY(-2, 0.5);
            collectSky();

        }
        else
        {
            moveY(-2, 0.5);
            moveX(-10, 0.5);
            collectSky();
        }
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

    void rotate(int distance, int power)
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

    private void collectSky()
    {
        motorRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grasp.setPosition(0);
        motorExtend.setPower(1);
        grasp.setPosition(0);
        angle.setPosition(0.65);
        motorRotate.setPower(0.7);
        if(i==1)
        {
            grasp.setPosition(0);
            motorExtend.setTargetPosition(motorExtend.getCurrentPosition() + 750);
            motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRotate.setTargetPosition(motorRotate.getCurrentPosition() + 550);
        }
        while (motorRotate.isBusy()) { }
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grasp.setPosition(0);
//        MotorFrontY.setPower(0.1);
//        MotorBackY.setPower(0.1);

//        MotorFrontY.setTargetPosition((int)(MotorFrontY.getCurrentPosition() + 1250/2));
//        MotorBackY.setTargetPosition((int)(MotorBackY.getCurrentPosition() + 1250/2));

//        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && MotorFrontY.isBusy() && MotorBackY.isBusy()) { }
        while (motorExtend.isBusy()) { }
        grasp.setPosition(1);

        angle.setPosition(0.65);
        sleep(250);
        grasp.setPosition(1);
        sleep(250);
        grasp.setPosition(1);
        motorRotate.setPower(0.5);
        motorRotate.setTargetPosition(motorRotate.getCurrentPosition() - 800);
        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (motorRotate.isBusy()) { }
        MotorBackY.setPower(0.1);
        MotorFrontY.setPower(0.1);
        MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() - 2240/4);
        MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() - 2240/4);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && MotorFrontY.isBusy() && MotorBackY.isBusy()) { }
        motorRotate.setPower(0.5);
        motorRotate.setTargetPosition(motorRotate.getCurrentPosition() -50);
        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (motorRotate.isBusy()) {
        }
        i++;
    }

    void collectSky2()
    {
        motorRotate.setPower(0.5);
        motorRotate.setTargetPosition(motorRotate.getCurrentPosition() +300);
        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grasp.setPosition(0);
        moveY(16, 0.2);
        grasp.setPosition(1);
        sleep(500);
        moveY(-17, 0.2);
        grasp.setPosition(1);
        moveX(79, 0.2);
        grasp.setPosition(0);
        moveX(-12.5, 1);
        moveY(-10, 0.5);

    }

}
