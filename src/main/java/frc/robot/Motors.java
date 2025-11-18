// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.function.DoubleSupplier;

//Phenoix 6 imports for the motors
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
//WPIlib imports for controls n stuff
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Motors extends SubsystemBase {

  //inizialies motors for the TalonFX Controllers

  public TalonFX motor_1 = new TalonFX(1);
  public TalonFX motor_2 = new TalonFX(2);


  //Statistical stuff, will use later
  public StatusSignal<Angle> position_1;
  public StatusSignal<Angle> position_2;
  public StatusSignal<Current> current_1;
  public StatusSignal<Current> current_2;
  public StatusSignal<AngularVelocity> velocity_1;
  public StatusSignal<AngularVelocity> velocity_2;

  //Voltage stuff and controller stuff
  public PositionVoltage positionvoltage = new PositionVoltage(0);
  public DutyCycleOut dutycycleout = new DutyCycleOut(1);
  public PIDController pidController = new PIDController(0,0,0);
  MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0);
  MotionMagicVelocityVoltage motionmagicrequest_velocity = new MotionMagicVelocityVoltage(50);
  

  /** Creates a new Motors. */
  public Motors() {


    //References the statistical stuff
      position_1 = motor_1.getPosition();
      position_2 = motor_2.getPosition();
      pidController = new PIDController(0.5,0.3, 0);
      TalonFXConfiguration config = new TalonFXConfiguration();


      //motion magic configuations
      config.MotionMagic.MotionMagicCruiseVelocity = 50;
      config.MotionMagic.MotionMagicAcceleration = 20;
      config.MotionMagic.MotionMagicJerk = 1000;

      config.Slot0.kP = 0.4;
      config.Slot0.kI = 0.6;

      motor_1.getConfigurator().apply(config);
      motor_2.getConfigurator().apply(config);

      



  }


//From this point, it is all methods

//Standard for motor_1
public\ Command run_standard_1(DoubleSupplier leftySupplier) {
    return this.runEnd(
        () -> {motor_1.setControl(motionmagicrequest.withPosition(leftySupplier.getAsDouble()));},
        () -> {motor_1.stopMotor();}
    );
}
//Standard for motor_2
public Command run_standard_2(DoubleSupplier rightySupplier) {
    return this.runEnd(
        () -> {motor_2.setControl(motionmagicrequest.withPosition(rightySupplier.getAsDouble()));},
        () -> {motor_2.stopMotor();}
    );
}
//Inverted, multiplies the input by -1 to change the direction
public Command inverted_1(DoubleSupplier leftySupplier){
  return this.runEnd(
      ()->{motor_1.setControl(motionmagicrequest.withPosition((leftySupplier.getAsDouble())*-1));},
      ()->{motor_1.stopMotor();}
    
    );
}
//Inverted 2, multiplies the input by -1 to change the direction

public Command inverted_2(DoubleSupplier rightySupplier){
  return this.runEnd(
        ()->{motor_2.setControl(motionmagicrequest.withPosition((rightySupplier.getAsDouble())*-1));},
        ()->{motor_2.stopMotor();}
    
    );
}

public Command blink() {
  return Commands.sequence(
      Commands.parallel(
          runOnce(() -> motor_1.setControl(dutycycleout.withOutput(0.2))),
          runOnce(() -> motor_2.setControl(dutycycleout.withOutput(0.2)))
      ),
      Commands.waitSeconds(0.3),
      Commands.parallel(
          runOnce(() -> motor_1.stopMotor()),
          runOnce(() -> motor_2.stopMotor())
      ),
      Commands.waitSeconds(0.3)
  ).repeatedly();
}

//Uses velocity 
public Command velocity_1(){
  return this.runEnd(
      ()->{motor_1.setControl(motionmagicrequest_velocity.withVelocity(50));},
      ()->{motor_1.stopMotor();}
   );
}

//Uses velocity
public Command velocity_2(){
  return this.runEnd(
      ()->{motor_2.setControl(motionmagicrequest_velocity.withVelocity(50));}, 
      ()->{motor_2.stopMotor();}
    );
}

//Resets the position to 0
public Command reset(){
  return this.runEnd(
      ()->{motor_1.setControl(motionmagicrequest.withPosition(0)); motor_2.setControl(motionmagicrequest.withPosition(0));}, 
      ()->{motor_1.stopMotor();motor_2.stopMotor();}
    
    );
}

//Alternatively, the reset command can also use the Command.Sequence and Command.Parrallel, will use this for blinking

//Same thing as the default without the smooth acceleration
public Command positionvoltageCommand_1(DoubleSupplier leftySupplier){
  return this.runEnd(
    ()->{motor_1.setControl(positionvoltage.withPosition(leftySupplier.getAsDouble()));},
    ()->{motor_1.stopMotor();}
  );

}
//Same thing as the default without the smooth accleration
public Command positionvoltageCommand_2(DoubleSupplier rightySupplier){
  return this.runEnd(
    ()->{motor_2.setControl(positionvoltage.withPosition(rightySupplier.getAsDouble()));},
    ()->{motor_2.stopMotor();}
  );
}




  public double getMotorPosition_1() {
    return position_1.refresh().getValueAsDouble();
  }

  public double getMotorPosition_2() {
    return position_2.refresh().getValueAsDouble();
  }

  @Override
  public void periodic() {
    
  }
}
