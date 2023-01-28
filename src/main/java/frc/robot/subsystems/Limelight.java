// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public static final String TX = "tx";
  public static final String TY = "ty";
  public static final String TA = "ta";
  public static final String LED_MODE = "ledMode";
  public static final String GET_PIPE = "getpipe";
  public static final String TV = "tv";
  public static final int TARGET_FOUND = 1;
  private double x = 0.0;
  private double y = 0.0;
  private double area = 0.0;
  private boolean hasActivePipeline = false;
  private boolean hasTarget = false;
  String tableName = "limelight";
  String pipelineID = "pipeLineID";
  NetworkTable table = null;
  
  
  public Limelight() {
     //get the table  does this have to a continuous get prior to getting entries or does get Once, use many work?
     table = NetworkTableInstance.getDefault().getTable(tableName);
     if (table != null)
        SmartDashboard.putString("TableName:", tableName);
     else
        SmartDashboard.putString("TableName:", "Unable to find limelight table");
  }

  //getters for values populated by subsystem periodic
  public double getX() { return x;}
  public double getY() { return y;}
  public double getArea() { return area;}
  public boolean isActive() { return hasActivePipeline;}
  public boolean isTargetValid() { return hasTarget;}

  public void turnOffLED() {
    int OFF = 1;
    if (table != null)
       table.getEntry(LED_MODE).setNumber(OFF); //0 is OFF
  }
  public void turnOnLED() {
    int ON = 0;
    if (table != null)
       table.getEntry(LED_MODE).setNumber(ON); //0 is OFF
  }

  @Override
  public void periodic() {
    updateAprilTagData();
  }

  private void updateAprilTagData() {
    if (table != null) {
      if (table.containsKey(GET_PIPE)) {
        hasActivePipeline = true;
        if ((int)table.getEntry(TV).getInteger(0) == TARGET_FOUND) {
           hasTarget = true;
           x = table.getEntry(TX).getDouble(0.0);
           y = table.getEntry(TY).getDouble(0.0);
           area = table.getEntry(TA).getDouble(0.0);
        }  else {
           hasTarget = false;
           x = 0.0;
           y = 0.0;
           area = 0.0; 
        }
      }
      SmartDashboard.putBoolean("Target Exists:", hasTarget);
      SmartDashboard.putNumber(TX, x);
      SmartDashboard.putNumber(TY, y);
      SmartDashboard.putNumber(TA, area);
    } else {
      SmartDashboard.putString("TableName:", "Unable to find limelight table");
    }  
  }
}