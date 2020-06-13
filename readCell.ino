

//Given a cell number, return the cell voltage
//Vcell = GAIN * ADC(cell) + OFFSET
//Conversion example from datasheet: 14-bit ADC = 0x1800, Gain = 0x0F, Offset = 0x1E = 2.365V
float readCellVoltage(byte cellNumber, boolean divide)
{
  if (cellNumber < 1 || cellNumber > 15) return (-0); //Return error

  //Serial1.print("Read cell number: ");
  //Serial1.println(cellNumber);

  //Reduce the caller's cell number by one so that we get register alignment
  cellNumber--;

  byte registerNumber = bq796x0_VC1_HI + (cellNumber * 2);

  int cellValue = registerDoubleRead(registerNumber);

  //int cellValue = 0x1800; //6,144 - Should return 2.365
  //int cellValue = 0x1F10l; //Should return 3.052

  //Cell value should now contain a 14 bit value

  if (cellValue == 0) return (0);

  float cellVoltage = cellValue * gain + offset; //0x1800 * 0.37 + 60 = 3,397mV

  if (divide)
  {
    cellVoltage /= (float)1000;
  }
  return (cellVoltage);
}

// read doible register
int registerDoubleRead(byte regAddress)
{
  Wire.beginTransmission(bqI2CAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(bqI2CAddress, 2);
  byte reg1 = Wire.read();
  byte reg2 = Wire.read();
  int combined = (int)reg1 << 8;
  combined |= reg2;
  return (combined);
}

void readCellBq() {
  if (millis() - lastTime > 200)
  {
//    Serial1.println("R-Cell 1 :" + String(readCellVoltage(1, true)));
//    dump();
    loop2();
//data_canbus();
    lastTime = millis();
  }
}
