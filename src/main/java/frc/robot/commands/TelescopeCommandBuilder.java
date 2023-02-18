package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.TelescopeConstants;
//import frc.robot.commands.interpolation.InterpolatingTable;
//import frc.robot.subsystems.cargoHandling.FeedServo;
//import frc.robot.subsystems.cargoHandling.Feeder;
//import frc.robot.subsystems.cargoHandling.Hood;
//import frc.robot.subsystems.cargoHandling.Indexer;
//import frc.robot.subsystems.cargoHandling.Intake;
//import frc.robot.subsystems.cargoHandling.IntakeArm;
//import frc.robot.subsystems.cargoHandling.Shooter;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.Telescope;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class TelescopeCommandBuilder {


    public static Command lowerNodeCommand (Telescope telescope){
        return new RunCommand (() -> telescope.setVoltage(TelescopeConstants.kTelescopeLowerPosition) ) ;       

    }


    public static Command middleNodeCommand (Telescope telescope){
        return new RunCommand (() -> telescope.setVoltage(TelescopeConstants.kTelescopeMiddlePosition));

    }

    public static Command upperNodeCommand (Telescope telescope){
        return new RunCommand (() -> telescope.setVoltage(TelescopeConstants.kTelescopeUpperPosition));
    }

    
}


