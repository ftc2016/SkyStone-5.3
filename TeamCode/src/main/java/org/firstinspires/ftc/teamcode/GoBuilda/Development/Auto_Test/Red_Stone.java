package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto_Test;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.UtilTest;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;


@Autonomous(name="Red Test", group = "Red")
public class Red_Stone extends LinearOpMode
{

    public ColorSensor leftColor, rightColor;
    public DistanceSensor leftDistance, rightDistance;
    //gyro init
    public BNO055IMU imu;
    public Orientation angles;

    public static DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorVertical, motorExtend;
    public static Servo graspL, graspR, block_drag_grasp, block_drag;

   void initPropAuto()
   {
       MotorFrontX = hardwareMap.dcMotor.get("fx");
       MotorBackX = hardwareMap.dcMotor.get("bx");
       MotorFrontY = hardwareMap.dcMotor.get("fy");
       MotorBackY = hardwareMap.dcMotor.get("by");
       motorVertical = hardwareMap.dcMotor.get("vertical");
       motorExtend = hardwareMap.dcMotor.get("extend");

       MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
       MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
       MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
       MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
       motorVertical.setDirection(DcMotorSimple.Direction.REVERSE);

       MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       MotorFrontX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       MotorBackX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       motorVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   }

    public void initServos()
    {
        graspL = hardwareMap.servo.get("graspL");
        graspR = hardwareMap.servo.get("graspR");
        block_drag_grasp = hardwareMap.servo.get("block_drag_grasp");
        block_drag = hardwareMap.servo.get("block_drag");
//        foundation1 = hardwareMap.servo.get("foundation1");
//        foundation2 = hardwareMap.servo.get("foundation2");
    }

    public void initializeSensors()
    {
        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftColor.enableLed(true);
        rightColor.enableLed(true);

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
    }

    public void initGyro()
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

    @Override
    public void runOpMode() throws InterruptedException
    {
        initPropAuto();
        initializeSensors();
        initServos();
        initGyro();

        waitForStart();

    }

    public void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        int motorTargetPos = MotorFrontY.getCurrentPosition() + counts;
        int wheelError = (MotorFrontY.getCurrentPosition()-counts);
        double wheelPow = 0.025 * wheelError;

        double pow = min(max(wheelPow,-1), 1);

        double corrnPow = getRotationCorrection(0);

        while(abs(wheelError)>=10)
        {
            wheelError = motorTargetPos - MotorFrontX.getCurrentPosition();
            MotorFrontY.setPower(pow);
            MotorBackY.setPower(pow);
        }


    }

    public int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
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

    public void rotationCorrection(float des_angle)
    {
        float desired_angle = des_angle;
        float angle_error = 10000.0f;

        angles = getAngles();

        float motorPower = 0;

        while (abs(angle_error) > 2.0f)
        {
            motorPower = getRotationCorrection(desired_angle);
//            MotorFrontX.setPower(motorPower);
//            MotorFrontY.setPower(-motorPower);
//            MotorBackX.setPower(-motorPower);
//            MotorBackY.setPower(+motorPower);
        }
    }


}
