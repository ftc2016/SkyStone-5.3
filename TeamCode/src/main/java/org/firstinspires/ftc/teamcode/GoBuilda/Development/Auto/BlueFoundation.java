package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.GyroCode;
import org.firstinspires.ftc.teamcode.Utils.GyroTest;

@Autonomous(name = "Blue Found Rot")
public class BlueFoundation extends LinearOpMode
{

    GoBuildaUtil robot = new GoBuildaUtil();
    GyroCode gyro = new GyroCode();
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    GyroTest gt = new GyroTest();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.initializeTele(hardwareMap);
        gyro.initGyro(hardwareMap);

        waitForStart();

        robot.NOENC();
        telemetry.addData("MotorPow", gyro.getRotationCorrection(0, 0.3));
        telemetry.update();
        gyro.rotationCorrection(0, 0.3, 1);
        sleep(1000);

        robot.initializeBlueAuto(hardwareMap);
        moveY(10, 0.5, 10);
        sleep(2500);

        robot.initializeTele(hardwareMap);
        telemetry.addData("MotorPow 2nd", gyro.getRotationCorrection(20, 0.3));
        telemetry.update();
        gyro.rotationCorrection(0, 0.3, 1);

        robot.initializeTele(hardwareMap);
        robot.moveY(-10, 0.5, 15);
        robot.moveX(10, 0.5);
        robot.moveY(10, 0.8, 6);
        robot.moveX(-10, 0.5);
        telemetry.addData("MotorPow 3rd", gyro.getRotationCorrection(0, 0.3));
        telemetry.update();
        gyro.rotationCorrection(0, 0.3, 1);


    }

    public void foundationServo(String cmd)
    {
        if(cmd.equalsIgnoreCase("engage"))
        {
            robot.foundation2.setPosition(1);
            robot.foundation.setPosition(0);
        }

        if(cmd.equalsIgnoreCase("disengage"))
        {
            robot.foundation2.setPosition(0);
            robot.foundation.setPosition(1);
        }

    }

    private int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    public void moveY(double inches, double power, double timeLimit)
    {
        elapsedTime.reset();
        elapsedTime.startTime();

        int counts = inchesToCounts(inches);

        robot.MotorFrontY.setPower(power);
        robot.MotorBackY.setPower(power);
        robot.MotorFrontX.setPower(0);
        robot.MotorBackX.setPower(0);

        robot.MotorFrontY.setTargetPosition((robot.MotorFrontY.getCurrentPosition() + (counts)));
        robot.MotorBackY.setTargetPosition((robot.MotorBackY.getCurrentPosition() + (counts)));

        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.MotorBackY.isBusy() && robot.MotorFrontY.isBusy()  && elapsedTime.time()<30) { }

        robot.setZero();
    }

    void FoundRot(int DesAngle) throws InterruptedException
    {
    }
}
