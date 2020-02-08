package org.firstinspires.ftc.teamcode.Utils;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
<<<<<<< Updated upstream
=======
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
>>>>>>> Stashed changes
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

@Disabled
public class GyroCode
{
    //gyro init
    BNO055IMU imu;
    Orientation angles;

    HardwareMap hard;
    GoBuildaUtil robot = new GoBuildaUtil();

    double last_angle_error;
    double motorPower;
    double angle_error;
    double motorPowerCorrection;

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

    public double getRotationCorrection(double desiredAngle, double desPower)
    {
        double sum;
        angles = getAngles();

<<<<<<< Updated upstream
        angle_error = desiredAngle - (float)angles.firstAngle;
        motorPowerCorrection = -0.025f * angle_error;
        motorPowerCorrection = min(max(motorPowerCorrection, -1.0f),1.0f);
        return motorPowerCorrection;
=======
        angle_error = desiredAngle - angles.firstAngle;
        sum = -0.02f * angle_error;
        motorPowerCorrection -= angle_error*0.003;
        motorPowerCorrection = min(max(motorPowerCorrection,-.1),.1);
        sum = min(max(motorPowerCorrection+sum, -desPower), desPower);

        return sum;
>>>>>>> Stashed changes
    }

    public void resetVariables()
    {
        double last_angle_error = 10000;
        double motorPower = 1500;
        double angle_error = 1500;
        double motorPowerCorrection= 1500;
    }

    public void rotationCorrection(double des_angle, double pow, double allowedError)
    {
        motorPowerCorrection = 0;

<<<<<<< Updated upstream
        while (abs(angle_error) > 1.0f)
        {
            motorPower = getRotationCorrection(desired_angle);
            robot.MotorFrontX.setPower(motorPower);
            robot.MotorFrontY.setPower(motorPower);
            robot.MotorBackX.setPower(-motorPower);
            robot.MotorBackY.setPower(-motorPower);
=======
        while (abs(angle_error) > allowedError || abs(last_angle_error) > allowedError)
        {
            Log.i("LOL", "Running the gyro unsuccesfully");
            last_angle_error = angle_error;
            motorPower = getRotationCorrection(des_angle, pow);
            robot.MotorFrontX.setPower(-motorPower);
            robot.MotorFrontY.setPower(motorPower);
            robot.MotorBackX.setPower(motorPower);
            robot.MotorBackY.setPower(-motorPower);
            Log.i("angleError: ", ""+angle_error);
            Log.i("lastAngleError: ", ""+last_angle_error);
>>>>>>> Stashed changes
        }
    }

    //Target Mode Power
}
