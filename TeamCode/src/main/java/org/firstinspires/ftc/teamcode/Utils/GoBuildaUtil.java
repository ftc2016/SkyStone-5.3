package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GoBuildaUtil
{
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    HardwareMap hardware;
    public ColorSensor leftColor, rightColor;
    public DistanceSensor leftDistance, rightDistance;
        //gyro init
        public BNO055IMU imu;
        public Orientation angles;
    public char blockPos = ' ';

    //initializing motors
<<<<<<< Updated upstream
    public static DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    public static Servo grasp1, grasp2, angle1, angle2, rightCollection, leftCollection, foundation;
=======
    public static DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorVertical, motorExtend;
    public static Servo graspL, graspR, block_drag_grasp, block_drag, foundation, foundation2;

    public void ENC()
    {
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void NoBrake()
    {
        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void Brake()
    {
        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

>>>>>>> Stashed changes

    public GoBuildaUtil() { }

    public void initializeTele(HardwareMap hw)
    {
        hardware = hw;

        MotorFrontX = hardware.dcMotor.get("fx");
        MotorBackX = hardware.dcMotor.get("bx");
        MotorFrontY = hardware.dcMotor.get("fy");
        MotorBackY = hardware.dcMotor.get("by");
        motorExtend = hardware.dcMotor.get("extend");
<<<<<<< Updated upstream
        motorRotate = hardware.dcMotor.get("rotate");
=======
        motorVertical = hardware.dcMotor.get("vertical");
>>>>>>> Stashed changes

        MotorFrontX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
<<<<<<< Updated upstream
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);
=======
        motorVertical.setDirection(DcMotorSimple.Direction.FORWARD);
>>>>>>> Stashed changes

        MotorFrontX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
<<<<<<< Updated upstream
        
        motorRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

=======
        motorVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
>>>>>>> Stashed changes

        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

<<<<<<< Updated upstream
        grasp1 = hardware.servo.get("grasp1");
        grasp2 = hardware.servo.get("grasp2");
        rightCollection = hardware.servo.get("rightCollection");
        leftCollection = hardware.servo.get("leftCollection");
        foundation = hardware.servo.get("foundation");
        angle1 = hardware.servo.get("angle1");
        angle2 = hardware.servo.get("angle2");
=======
        initServos(hw);
>>>>>>> Stashed changes
    }

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
<<<<<<< Updated upstream

        grasp1 = hardware.servo.get("grasp1");
        grasp2 = hardware.servo.get("grasp2");
        rightCollection = hardware.servo.get("rightCollection");
        leftCollection = hardware.servo.get("leftCollection");
        foundation = hardware.servo.get("foundation");
        angle1 = hardware.servo.get("angle1");
        angle2 = hardware.servo.get("angle2");
=======

        initServos(hw);
    }

    public void initializeBlueAuto(HardwareMap hw)
    {
        hardware = hw;

        MotorFrontX = hardware.dcMotor.get("fx");
        MotorBackX = hardware.dcMotor.get("bx");
        MotorFrontY = hardware.dcMotor.get("fy");
        MotorBackY = hardware.dcMotor.get("by");
        motorExtend = hardware.dcMotor.get("extend");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);

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

        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initServos(hw);
    }

    void initServos(HardwareMap hw)
    {
        hardware = hw;

        graspL = hardware.servo.get("graspL");
        graspR = hardware.servo.get("graspR");
        block_drag_grasp = hardware.servo.get("block_drag_grasp");
        block_drag = hardware.servo.get("block_drag");
        foundation = hardware.servo.get("foundation");
        foundation2 = hardware.servo.get("foundation2");
>>>>>>> Stashed changes
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

    public void NOENC()
    {
        MotorFrontX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);

//        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setZero()
    {
        MotorFrontX.setPower(0);
        MotorBackX.setPower(0);
        MotorFrontY.setPower(0);
        MotorBackY.setPower(0);
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

    public void moveY(double inches, double power, double timeLimit)
    {
        elapsedTime.reset();
        elapsedTime.startTime();

        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);
        MotorFrontX.setPower(0);
        MotorBackX.setPower(0);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (MotorBackY.isBusy() && MotorFrontY.isBusy()  && elapsedTime.time()<30) { }

        setZero();
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
