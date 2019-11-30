package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Autonomous(name = "Park", group = "Both")
public class ParkingTheRobot extends LinearOpMode
{
    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp, angle, foundation;


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

    private int inchesToCounts(double inches)
    {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev/Servocity_Omnni_Circumference;

        return (int)(COUNTS_PER_REVOLUTION*inches);
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
        foundation = hardwareMap.servo.get("foundation");
        angle = hardwareMap.servo.get("angle");

        grasp.setPosition(1);
        angle.setPosition(0.65);
        foundation.setPosition(0);
    }

    void armExtend(int counts, double power)
    {
        motorExtend.setPower(power);

        motorExtend.setTargetPosition(motorExtend.getCurrentPosition()+counts);

        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && motorExtend.isBusy())
        {
            telemetry.addData("arm is extending to position", counts);
        }
    }

    void armRotate(int counts, double power)
    {
        motorRotate.setPower(power);

        motorRotate.setTargetPosition(motorRotate.getCurrentPosition()+counts);

        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive() && motorRotate.isBusy())
        {
            telemetry.addData("arm is rotating to position", counts);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeMotors();

        waitForStart();

        sleep(25000);

        armRotate(300, 1);
        armExtend(150, 1);
        moveY(20, 1);

    }
}
