//package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
//
//@Autonomous(name = "Park", group = "Both")
//public class ZParkingTheRobot extends LinearOpMode
//{
//    //initalizing sensors
//    private ColorSensor leftColor, rightColor;
//    private DistanceSensor leftDistance, rightDistance;
//    //gyro init
//    BNO055IMU imu;
//    Orientation angles;
//    char blockPos = ' ';
//
//    //initializing motors
//    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
//    Servo grasp1, grasp2, angle1, angle2, rightCollection, leftCollection, foundation;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        initializeMotors();
//
//        waitForStart();
//
//        sleep(25000);
//
//        armRotate(300, 1);
//        armExtend(500, 1);
//        moveY(48, 1);
//
//    }
//
//    private void initializeMotors() {
//        MotorFrontX = hardwareMap.dcMotor.get("fx");
//        MotorBackX = hardwareMap.dcMotor.get("bx");
//        MotorFrontY = hardwareMap.dcMotor.get("fy");
//        MotorBackY = hardwareMap.dcMotor.get("by");
//        motorExtend = hardwareMap.dcMotor.get("extend");
//        motorRotate = hardwareMap.dcMotor.get("rotate");
//
//        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
//        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
//        MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
//        MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        grasp1 = hardwareMap.servo.get("grasp1");
//        grasp2 = hardwareMap.servo.get("grasp2");
//        rightCollection = hardwareMap.servo.get("rightCollection");
//        leftCollection = hardwareMap.servo.get("leftCollection");
//        foundation = hardwareMap.servo.get("foundation");
//        angle1 = hardwareMap.servo.get("angle1");
//        angle2 = hardwareMap.servo.get("angle2");
//
////        grasp1.setPosition(1);
////        grasp2.setPosition(0);
//
//    }
//
//    private void initSensors()
//    {
//
//        leftColor = hardwareMap.get(ColorSensor.class, "left");
//        rightColor = hardwareMap.get(ColorSensor.class, "right");
//
//        leftColor.enableLed(true);
//        rightColor.enableLed(true);
//
//        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
//        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
//    }
//
//    void moveY(double inches, double power)
//    {
//        int counts = inchesToCounts(inches);
//
//        MotorFrontY.setPower(-power);
//        MotorBackY.setPower(-power);
//
//        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
//        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));
//
//        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy())
//        {
//            telemetry.addData("Running motor Y front and back", "Encoders");
//            telemetry.update();
//        }
//    }
//
//    private int inchesToCounts(double inches)
//    {
//        //wheel specification
//        final double Servocity_Omnni_Circumference = Math.PI * 4;
//        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
//        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev/Servocity_Omnni_Circumference;
//
//        return (int)(COUNTS_PER_REVOLUTION*inches);
//    }
//
//
//    void armExtend(int counts, double power)
//    {
//        motorExtend.setPower(power);
//
//        motorExtend.setTargetPosition(motorExtend.getCurrentPosition()+counts);
//
//        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(opModeIsActive() && motorExtend.isBusy())
//        {
//            telemetry.addData("arm is extending to position", counts);
//        }
//    }
//
//    void armRotate(int counts, double power)
//    {
//        motorRotate.setPower(power);
//
//        motorRotate.setTargetPosition(motorRotate.getCurrentPosition()+counts);
//
//        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(opModeIsActive() && motorRotate.isBusy())
//        {
//            telemetry.addData("arm is rotating to position", counts);
//        }
//    }
//}
