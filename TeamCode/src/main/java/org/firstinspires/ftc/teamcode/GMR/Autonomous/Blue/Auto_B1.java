package org.firstinspires.ftc.teamcode.GMR.Autonomous.Blue;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.GMR.Robot.Robot;
import org.firstinspires.ftc.teamcode.GMR.Robot.SubSystems.DriveTrain;

/**
 * Created by FTC 4316 on 11/11/2017
 */
@Autonomous(name = "Auto B1", group = "Blue")
public class Auto_B1 extends OpMode {

    private Robot robot;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    NavxMicroNavigationSensor gyroscope;
    IntegratingGyroscope gyro;

    Servo leftArm;
    Servo rightArm;
    ColorSensor colorSensorLeft;
    DistanceSensor distanceSensorLeft;

    private States state;

    private boolean isFinished;

    private double position;
    private double goalPosition;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightfront");
        leftFront = hardwareMap.dcMotor.get("leftfront");
        rightRear = hardwareMap.dcMotor.get("rightrear");
        leftRear = hardwareMap.dcMotor.get("leftrear");

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorDistanceLeft");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "colorDistanceLeft");

        gyroscope = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        robot = new Robot(hardwareMap, telemetry);

        goalPosition = 0.35;
        position = 0.85;
        leftArm.setPosition(position); //vertical start
        rightArm.setPosition(0);
        // position

        state = States.TIME;
        isFinished = false;

    }
        @Override
        public void loop(){
            switch(state){
                case TIME:
                    //Starts the timer
                    time.reset();
                    state = States.ARMDOWN;
                    break;
                case ARMDOWN:
                    //Lowers left arm
                    leftArm.setPosition(goalPosition);
                    if(time.seconds() >= 1.0){
                        state = States.READ;
                    } break;

                case READ:
                    //Reads the color/distance sensor to determine which ball to knock off
                    if(colorSensorLeft.blue() > colorSensorLeft.red()){
                        telemetry.addData("Blue:", colorSensorLeft.blue());
                        telemetry.addData("Red:", colorSensorLeft.red());
                        telemetry.addData("The ball is:", "blue");
                        telemetry.update();

                        state = States.LEFTKNOCK;
                    } else if(colorSensorLeft.red() > colorSensorLeft.blue()){
                        telemetry.addData("Blue:", colorSensorLeft.blue());
                        telemetry.addData("Red:", colorSensorLeft.red());
                        telemetry.addData("The ball is:", "red");
                        telemetry.update();

                        state = States.RIGHTKNOCK;
                    } break;

                case LEFTKNOCK:
                    //Knocks the left ball off of the pedestal
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.S, 0.25, 1);
                    } else{
                        isFinished = false;
                        state = States.LEFTARMUP;
                        time.reset();
                    } break;

                case RIGHTKNOCK:
                    //Knocks the right ball off of the pedestal
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 1);
                    } else{
                        isFinished = false;
                        state = States.RIGHTARMUP;
                        time.reset();
                    } break;

                case LEFTARMUP:
                    //Lifts arm up after knocking left ball
                    leftArm.setPosition(0.85);
                    if(time.seconds() >= 1){
                        state = States.LEFTZONE;
                    } break;

                case RIGHTARMUP:
                    //Lifts arm up after knocking right ball
                    leftArm.setPosition(0.85);
                    if(time.seconds() >= 1){
                        state = States.RIGHTZONE;
                    } break;

                case LEFTZONE:
                    //Returns to original position from knocking left ball
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 10);
                    } else{
                        isFinished = false;
                        state = States.TURNBOX;
                        time.reset();
                    } break;

                case RIGHTZONE:
                    //Returns to original position from knocking right ball
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 3);
                    } else{
                        isFinished = false;
                        state = States.TURNBOX;
                        time.reset();
                    } break;
                case TURNBOX:
                    //Turns left to face CryptoBox. UNTESTED
                    if(!isFinished){
                        isFinished = robot.driveTrain.gyroTurn(DriveTrain.Direction.TURNLEFT, 0.25, 90);
                    } else{
                        isFinished = false;
                        state = States.DRIVEBOX;
                    } break;

                case DRIVEBOX:
                    if(!isFinished){
                        isFinished = robot.driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 1);
                    } else{
                        isFinished = false;
                        state = States.END;
                    } break;

                case END:
                    robot.driveTrain.stop();
                    break;
            }

        }

}
enum States {
    SCAN,
    TIME,
    ARMDOWN,
    READ,
    LEFTKNOCK,
    RIGHTKNOCK,
    LEFTARMUP,
    RIGHTARMUP,
    LEFTZONE,
    RIGHTZONE,
    DRIVEOFF,
    DRIVEZONE,
    TURNBOX,
    DRIVEBOX,
    END
}