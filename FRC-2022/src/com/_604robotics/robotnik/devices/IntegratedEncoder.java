package com._604robotics.robotnik.devices;

public interface IntegratedEncoder {

public boolean getInverted();

public void setInverted(boolean inverted);

public void setdistancePerRotation(double distancePerRotation);

public double getPositionConversionFactor();

public double getVelocityConversionFactor();

public double getPosition();

public double getVelocity();

public void zero();

public void zero(double value);

public void setSimPosition(double value);

}
