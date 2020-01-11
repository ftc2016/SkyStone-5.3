package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

public class GyroCode
{
    //gyro init
    BNO055IMU imu;
    Orientation angles;

    HardwareMap hard;

    GoBuildaUtil robot = new GoBuildaUtil();

    public void initGyro(HardwareMap hw)
    {
        hard = hw;

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

        imu = hard.get(BNO055IMU.class, "imu");
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

    public void rotationCorrection(float des_angle)
    {
        float desired_angle = des_angle;
        float angle_error = 10000.0f;

        angles = getAngles();

        float motorPower = 0;

        while (abs(angle_error) > 2.0f)
        {
            motorPower = getRotationCorrection(desired_angle);
            robot.MotorFrontX.setPower(motorPower);
            robot.MotorFrontY.setPower(-motorPower);
            robot.MotorBackX.setPower(-motorPower);
            robot.MotorBackY.setPower(+motorPower);

            if((abs(angle_error))<3.0f)
                break;
        }
    }
}
