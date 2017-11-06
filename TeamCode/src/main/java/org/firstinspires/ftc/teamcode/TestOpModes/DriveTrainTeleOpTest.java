package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrain;

/**
 * Created by FTC 4316 on 10/29/2017
 */
@TeleOp(name="Drive Train TeleOp test", group="Test")
public class DriveTrainTeleOpTest extends OpMode {

    private DriveTrain driveTrain;

    private double x;
    private double y;
    private double z;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    private double power;
    private double finches;

    private boolean isFinished;

    NavxMicroNavigationSensor gyroScope;
    IntegratingGyroscope gyro;

    @Override
    public void init() {



        leftFront = hardwareMap.dcMotor.get("leftfront");
        rightFront = hardwareMap.dcMotor.get("rightfront");
        leftRear = hardwareMap.dcMotor.get("leftrear");
        rightRear = hardwareMap.dcMotor.get("rightrear");

        power = 0.1;
        finches = 5;

        gyroScope = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)gyroScope;

        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, gyro,  telemetry);

        isFinished = true;
    }

    @Override
    public void loop() {


            if (gamepad1.dpad_down){
                isFinished = driveTrain.encoderDrive(DriveTrain.Direction.S, power, finches);
            }
            else if(gamepad1.dpad_right){
                isFinished = driveTrain.encoderDrive(DriveTrain.Direction.E, power, finches);
            }
            else if (gamepad1.dpad_up){
                isFinished = driveTrain.encoderDrive(DriveTrain.Direction.N, power, finches);
            }
            else if (gamepad1.dpad_left) {
                isFinished = driveTrain.encoderDrive(DriveTrain.Direction.W, power, finches);
            } else if (gamepad1.a) {
                isFinished = driveTrain.gyroTurn(DriveTrain.Direction.TURNRIGHT, power, 90);
            }

            if (isFinished) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
                z = gamepad1.right_stick_x;

                driveTrain.setMotorPower(x, y, z);
            }

            telemetry.addData("Current Yaw", driveTrain.getYaw());
            driveTrain.displayEncoders();

        }
    }
