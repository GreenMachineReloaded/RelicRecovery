package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.teamcode.DriveTrain;

/**
 * Created by FTC 4316 on 11/19/2017.
 */
@Autonomous(name = "EncoderDrive Test", group="Test")
public class Test extends OpMode {

    private DriveTrain driveTrain;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private NavxMicroNavigationSensor gyro;

    private boolean isFinished;

    public void init(){
        leftFront = hardwareMap.dcMotor.get("leftfront");
        rightFront = hardwareMap.dcMotor.get("rightfront");
        leftRear = hardwareMap.dcMotor.get("leftrear");
        rightRear = hardwareMap.dcMotor.get("rightrear");

        gyro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, gyro, telemetry);

        isFinished = false;
    }
    public void loop(){ 
        if(!isFinished){
            isFinished = driveTrain.encoderDrive(DriveTrain.Direction.N, 0.25, 5);
        }
    }
}
