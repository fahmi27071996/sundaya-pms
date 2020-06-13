
#include <Wire.h>
#include "sundaya_bq769x0.h"
// #include <MapleFreeRTOS900.h>
// ====================== canbus header ================//

#include <HardwareCAN.h>

// ====================== canbus header ================//

// ====================== bq76940 TYPE ====================== //

boolean bq76940_crc = true;

// ====================== bq76940 TYPE ====================== //

// ====================== canbus variables ==================//

byte msgD0; // variable to be used in the example.

// Instanciation of CAN interface
HardwareCAN canBus(CAN1_BASE);
CanMsg msg;

// ====================== canbus variables ==================//

//The bq769x0 without CRC has the 7-bit address 0x08. The bq769x0 with CRC has the address 0x18.
//Please see the datasheet for more info
int bqI2CAddress = 0x08; //7-bit I2C address

//My pack is a 15 cell lipo that runs at 48V. Your pack may vary. Read the datasheet!
//This code is written for the bq76940. The bq76940 supports 9 to 15 cells.
#define NUMBER_OF_CELLS 16

//Max number of ms before timeout error. 100 is pretty good
#define MAX_I2C_TIME 100

volatile boolean bq769x0_IRQ_Triggered = false; //Keeps track of when the Alert pin has been raised

float gain = 0; //These are two internal factory set values.
int offset = 0; //We read them once at boot up and use them in many functions

long lastTime; //Used to blink the status LED

long totalCoulombCount = 0; //Keeps track of overall pack fuel gauage

int cellVoltage[NUMBER_OF_CELLS + 1]; //Keeps track of the cell voltages

//GPIO declarations
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//byte irqPin = ; //Interrupt enabled, connected to bq pin ALERT
byte statLED = PC14; //Ob board status LED

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// ==================  canbus setup ================= //

// ================== for poweroad ============//

#define base_id 124045412
#define multiply 65536

#define dock_number 1

#define can_enable PB13
#define supply_enable PB14
#define busbar_short PB12

#define on true
#define off false

void send_pack(unsigned long id, byte d0, byte d1, byte d2, byte d3, byte d4, byte d5, byte d6, byte d7)
{
  msg.IDE = CAN_ID_EXT;   // Indicates a standard identifier ; CAN_ID_EXT would mean this frame uses an extended identifier
  msg.RTR = CAN_RTR_DATA; // Indicated this is a data frame, as opposed to a remote frame (would then be CAN_RTR_REMOTE)
  msg.ID = id;            // Identifier of the frame : 0-2047 (0-0x3ff) for standard idenfiers; 0-0x1fffffff for extended identifiers
  msg.DLC = 8;            // Number of data bytes to follow

  // Prepare frame : send something
  msg.Data[0] = d0;
  msg.Data[1] = d1;
  msg.Data[2] = d2;
  msg.Data[3] = d3;
  msg.Data[4] = d4;
  msg.Data[5] = d5;
  msg.Data[6] = d6;
  msg.Data[7] = d7;
  CANsend(&msg); // Send this frame
}

// ================== for poweroad ============//

void CANSetup(void)
{

  CAN_STATUS Stat;

  // Initialize CAN module
  canBus.map(CAN_GPIO_PA11_PA12);
  Stat = canBus.begin(CAN_SPEED_250, CAN_MODE_NORMAL);

  canBus.filter(0, 0, 0);
  canBus.set_irq_mode(); // Use irq mode (recommended), so the handling of incoming messages
  // will be performed at ease in a task or in the loop. The software fifo is 16 cells long,
  // allowing at least 15 ms before processing the fifo is needed at 125 kbps
  Stat = canBus.status();
  if (Stat != CAN_OK)
  {
    Serial1.println("CANBUS GAGAl");
    digitalWrite(PC14, LOW);
  }
  else
  {
    Serial1.println("CANBUS SUCCES");
    digitalWrite(PC14, HIGH);
  }

  /* Your own error processing here */; // Initialization failed
}

// Send one frame. Parameter is a pointer to a frame structure (above), that has previously been updated with data.
// If no mailbox is available, wait until one becomes empty. There are 3 mailboxes.
CAN_TX_MBX CANsend(CanMsg *pmsg) // Should be moved to the library?!
{
  CAN_TX_MBX mbx;

  do
  {
    mbx = canBus.send(pmsg);
#ifdef USE_MULTITASK
    vTaskDelay(1); // Infinite loops are not multitasking-friendly
#endif
  } while (mbx == CAN_TX_NO_MBX); // Waiting outbound frames will eventually be sent, unless there is a CAN bus failure.
  return mbx;
}

// Send message
// Prepare and send a frame containing some value
void SendCANmessage(long id = 0x001, byte d0 = 0x00, byte d1 = 0x00, byte d2 = 0x00, byte d3 = 0x00, byte d4 = 0x00, byte d5 = 0x00, byte d6 = 0x00, byte d7 = 0x00)
{
  // Initialize the message structure
  // A CAN structure includes the following fields:
  msg.IDE = CAN_ID_EXT;   // Indicates a standard identifier ; CAN_ID_EXT would mean this frame uses an extended identifier
  msg.RTR = CAN_RTR_DATA; // Indicated this is a data frame, as opposed to a remote frame (would then be CAN_RTR_REMOTE)
  msg.ID = id;            // Identifier of the frame : 0-2047 (0-0x3ff) for standard idenfiers; 0-0x1fffffff for extended identifiers
  msg.DLC = 8;            // Number of data bytes to follow

  // Prepare frame : send something
  msg.Data[0] = d0;
  msg.Data[1] = d1;
  msg.Data[2] = d2;
  msg.Data[3] = d3;
  msg.Data[4] = d4;
  msg.Data[5] = d5;
  msg.Data[6] = d6;
  msg.Data[7] = d7;
  CANsend(&msg); // Send this frame
}

// ==================  canbus setup ================= //

// ================== for poweroad ============//

// ================= for messages ==================== //

void serial_messages()
{
  delay(2000);
  Serial1.println("=========== Sundaya 2020 ===========");
  Serial1.println("Serial command message for controlling the bq76940");
  Serial1.println("1 : Entering ship mode bq76940");
  Serial1.println("2 : Load ON");
  Serial1.println("3 : Load OFF");
  Serial1.println("=========== Sundaya 2020 ===========");
  //get_uid ();
  delay(2000);
}

// ================= for messages ==================== //

void setup()
{
  
  // ================ for enable output from SY8502 ========== //

  pinMode(supply_enable, OUTPUT);
  digitalWrite(supply_enable, HIGH);

  delay(500);
  // ================ for enable output from SY8502 ========== //

  //BMS
  pinMode(PB4, OUTPUT);
  //  digitalWrite(PB4, LOW);
  digitalWrite(PB4, HIGH);
  delay(2000);
  digitalWrite(PB4, LOW);

  Serial1.begin(115200);

  Wire.begin(); //Start I2C communication

  pinMode(statLED, OUTPUT);
  digitalWrite(statLED, LOW); //Turn off the LED for now

  if (initBQ() == false) //Call init with pin 2 (IRQ0) or 3 (IRQ1)
  {
    Serial1.println("bq76940 failed to respond - check your wiring");
    Serial1.println("Hanging.");
    while (1)
      ;
  }
  else
  {
    Serial1.println("bq76940 initialized!");
  }

  lastTime = millis();

  serial_messages();

  // ================ FOR CANBUS =============//

  pinMode(can_enable, OUTPUT);
  digitalWrite(can_enable, LOW);
  delay(1000);
  //  digitalWrite(can_enable, HIGH);
  //  delay(1000);
  //  digitalWrite(can_enable, LOW);
  CANSetup();
  msgD0 = 0x01;

  // ================ FOR CANBUS =============//

  pinMode(busbar_short, INPUT);
  // getData();
  readDataCell();
}

void dummyCan()
{

  //  digitalWrite(can_enable, LOW);

  //  CANSetup();

  delay(1000);
  long msgID = 0x101;
  SendCANmessage(msgID, msgD0);
  msgD0++;
  Serial1.println("cansend");
}

void busbar_short_detection()
{
  if (digitalRead(busbar_short) == 0)
  {
    // pressed
    // data_canbus();

    display_cell_voltage();
    dump();
    discharge(on);
    delay(1000);
    //digitalWrite(PC14, LOW);
  }

  else
  {
    discharge(off);
    digitalWrite(statLED, LOW);
  }
}

void control_serial()
{
  if (Serial1.available() > 0)
  {
    int incoming = Serial1.parseInt();
    Serial1.println("MAsuk");

    if (incoming == 1)
    {
      Serial1.println("Entering ship mode");
      enterSHIPmode();
    }

    if (incoming == 2)
    {
      Serial1.println("ON Load");
      //      CANSetup();
      registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_ON);
    }

    if (incoming == 3)
    {
      Serial1.println("OFF Load");
      registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_OFF);
    }

    dummyCan();

    if (incoming == 4)
    {
      //      registerWrite(bq796x0_SYS_STAT, bq796x0_OCD);
      //digitalWrite(PB4, LOW);
      //delay(2000);
      //digitalWrite(PB4, HIGH);
      //delay(2000);
      Wire.begin();
      delay(2000);
      digitalWrite(PB4, LOW);
      delay(2000);
      digitalWrite(PB4, HIGH);
      delay(2000);
      digitalWrite(PB4, LOW);

      if (initBQ() == false) //Call init with pin 2 (IRQ0) or 3 (IRQ1)
      {
        Serial1.println("bq76940 failed to respond - check your wiring");
        Serial1.println("Hanging.");
        while (1)
          ;
      }
      else
      {
        Serial1.println("bq76940 initialized!");
      }
    }
  }
}

void data_canbus()
{
  //  digitalWrite(can_enable,LOW);
  //  delay(1000);
  long msgID = 0x101;
  SendCANmessage(msgID, msgD0);
  msgD0++;
}

void read_canbus()
{
  digitalWrite(can_enable, LOW);
  CanMsg *r_msg;
  if ((r_msg = canBus.recv()) != NULL)
  {
    Serial1.print(r_msg->ID);
    Serial1.print("#");
    Serial1.print(r_msg->Data[0]);
    Serial1.print(".");
    Serial1.print(r_msg->Data[1]);
    Serial1.print(".");
    Serial1.print(r_msg->Data[2]);
    Serial1.print(".");
    Serial1.print(r_msg->Data[3]);
    Serial1.print(".");
    Serial1.print(r_msg->Data[4]);
    Serial1.print(".");
    Serial1.print(r_msg->Data[5]);
    Serial1.print(".");
    Serial1.print(r_msg->Data[6]);
    Serial1.print(".");
    Serial1.println(r_msg->Data[7]);
    if (r_msg->ID == 0x201 and r_msg->Data[0] == 0x01 and r_msg->Data[1] == 0xFF)
    {
      discharge(true);
      // SETTINGS is pressed!
      Serial1.println("SETTINGS. Pause for 2 seconds.");
      digitalWrite(PC14, HIGH); // turn the onboard LED on
      delay(2000);
      digitalWrite(PC14, LOW); // turn the LED off
    }
    else if (r_msg->ID == 0x201 and r_msg->Data[0] == 0x00 and r_msg->Data[1] == 0xFF)
    {
      discharge(false);
    }
    canBus.free();
  }
}

void dump()
{
  float cellData[14];
  for (int x = 1; x <= 14; x++)
  {
    cellData[x - 1] = readCellVoltage(x, false);
    if (x == 14)
    {
      cellData[x - 1] = readCellVoltage(15, false);
    }
    Serial1.println("cell : " + String(x) + " " + String(cellData[x - 1]));
  }

  int packV = readPackVoltage() * 100;
  int pack[2];
  pack[0] = (25700 - packV) / 256 + 1;
  pack[1] = (25700 - packV) % 256;
  Serial1.print("\n1 = ");
  Serial1.println(pack[0]);
  Serial1.print("\n2 = ");
  Serial1.println(pack[1]);
  Serial1.print("\nPackV = ");
  Serial1.println(packV);

  //  #define base_id 124045412
  //#define multiply 65536
  //dock_number

  /*

    // ===================== //

    1. 1,2 tidak diketahui
    2. 3,4 Vpack data





    // ===================== //
  */
  send_pack(124045412 - dock_number, 0x56, 0x61, pack[1], pack[0], 0x02, 0x64, 0x01, 0x64);
  send_pack(124045412 - dock_number - 65536, 0x53, 0x64, 0x64, 0x49, 0x49, 0x4b, 0x47, 0x45);

  int array_byte1[14];
  int array_byte2[14];

  for (int cell_converter = 0; cell_converter <= 14; cell_converter++)
  {
    int base_cell = (25700 - cellVoltage[cell_converter]);
    array_byte1[cell_converter] = base_cell % 256;
    // array_byte2[cell_converter] = base_cell/256;
  }

  for (int cell_converter = 0; cell_converter <= 14; cell_converter++)
  {
    int base_cell = (25700 - cellVoltage[cell_converter]);
    // array_byte1[cell_converter] = base_cell%256;
    array_byte2[cell_converter] = base_cell / 256;
  }

  //  Serial1.println(String(cellVoltage[0]) + " aneh : " + String(array_byte1[0]) + " aneh1 : " + String (array_byte2[0]));

  // send_pack(124045412-dock_number-(65536*2),byte1,byte2,0x15,0x56,0x15,0x56,0x15,0x56);
  send_pack(124045412 - dock_number - (65536 * 2), array_byte1[0], array_byte2[0], array_byte1[1], array_byte2[1], array_byte1[2], array_byte2[2], array_byte1[3], array_byte2[3]);
  send_pack(124045412 - dock_number - (65536 * 3), array_byte1[4], array_byte2[4], array_byte1[5], array_byte2[5], array_byte1[6], array_byte2[6], array_byte1[7], array_byte2[7]);
  send_pack(124045412 - dock_number - (65536 * 4), array_byte1[8], array_byte2[8], array_byte1[9], array_byte2[9], array_byte1[10], array_byte2[10], array_byte1[11], array_byte2[11]);
  send_pack(124045412 - dock_number - (65536 * 5), array_byte1[12], array_byte2[12], array_byte1[13], array_byte2[13], 0x15, 0x56, 0x15, 0x56);

  send_pack(124045412 - dock_number - (65536 * 6), 0x15, 0x56, 0x17, 0x56, 0x49, 0x4b, 0x42, 0xfc);
  send_pack(124045412 - dock_number - (65536 * 7), 0x64, 0x63, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64);
  send_pack(124045412 - dock_number - (65536 * 8), 0x27, 0x5a, 0x64, 0x64, 0xe1, 0x6f, 0x61, 0x64);
  send_pack(124045412 - dock_number - (65536 * 9), 0x0e, 0x57, 0x64, 0x64, 0xf4, 0x59, 0x64, 0x64);
  send_pack(124045412 - dock_number - (65536 * 10), 0x52, 0x64, 0x59, 0x5f, 0x5a, 0x29, 0x64, 0x00);
}

void display_cell_voltage()
{
  if (millis() - lastTime > 1000)
  {

    if (digitalRead(statLED) == HIGH)
    {
    }
    //      digitalWrite(statLED, LOW);
    else
    {
      //      digitalWrite(statLED, HIGH);
      displayVoltages(false);
    }
  }
}

void discharge(boolean mosfet_control)
{
  if (mosfet_control)
  {
    //registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_ON);
    registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_ON);
  }

  else
  {
    registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_OFF);
  }
}

void charge(boolean mosfet_control)
{
  if (mosfet_control)
  {
    registerWrite(bq796x0_SYS_CTRL2, bq796x0_CHG_ON);
  }

  else
  {
    registerWrite(bq796x0_SYS_CTRL2, bq796x0_CHG_OFF);
  }
}

void loop1()
{
  control_serial();
  //  dump();
  read_canbus();
  data_canbus();
  //  data_canbus();
  //  data_canbus();
  //SendCANmessage(1,1,1,1,1,1,1,1,1);
  //  busbar_short_detection ();
}

void loop()
{
  //  readCellBq();
  control_serial();
  
  checkBusbar();
  readCurrent();
  read_canbus();
  delay(500);
  
  // dump();
  // delay(200);
  
}

void loop2()
{
  control_serial();
  //  data_canbus();
  dump();
  read_canbus();
  control_serial();
  if (millis() - lastTime > 1000)
  {
    //    displayVoltages(true);

    for (int i = 0; i < NUMBER_OF_CELLS; i++)
    {
      cellVoltage[i] = readCellVoltage(i, true);
    }

    // Serial1.println();

    //Toggle stat LED
    //    if (digitalRead(statLED) == HIGH)
    //      digitalWrite(statLED, LOW);
    //    else
    //    {
    //      digitalWrite(statLED, HIGH);
    //      //      displayVoltages(true);
    //
    //    }

    //    Serial1.println("cell : " + String(readCellVoltage(10,true)));

    int temp = readTemp(1);
    Serial1.print("Die temp = ");
    Serial1.println(temp);

    //Calc the pack voltage by hand?
    float totalVoltage = 0;
    //    for(int i = 1 ; i <= 15 ; i++)
    //    {
    //      Serial1.print("cell : " + String(i) + " " + readCellVoltage(i,true));
    //      totalVoltage +=readCellVoltage(i,true);
    //    }

    //    Serial1.print("\nPackV manual = ");
    //    Serial1.println(totalVoltage);

    lastTime = millis();
  }

  //For every IRQ event read the flags and update the coulomb counter and other major events
  if (bq769x0_IRQ_Triggered == true)
  {
    //Read the status register and update if needed
    byte sysStat = registerRead(bq796x0_SYS_STAT);

    Serial1.print("sysStat: 0x");
    Serial1.println(sysStat, HEX);

    //Double check that ADC is enabled
    //byte sysVal = registerRead(bq796x0_SYS_CTRL1);
    //if(sysVal & bq796x0_ADC_EN)
    //{
    //  Serial1.println("ADC Enabled");
    //}

    //We need to write 1s into all the places we want a zero, but not overwrite the 1s we want left alone
    byte sysNew = 0;

    //Check for couloumb counter read
    if (sysStat & bq796x0_CC_READY)
    {
      Serial1.println("CC Ready");
      totalCoulombCount += readCoulombCounter(); //Add this 250ms reading to the global fuel gauge
      sysNew |= bq796x0_CC_READY;                //Clear this status bit by writing a one into this spot
    }

    if (sysStat & bq796x0_DEVICE_XREADY) //Internal fault
    {
      Serial1.println("Internal fault");
      sysNew |= bq796x0_DEVICE_XREADY; //Clear this status bit by writing a one into this spot
    }

    if (sysStat & bq796x0_OVRD_ALERT) //Alert pin is being pulled high externally?
    {
      Serial1.println("Override alert");
      sysNew |= bq796x0_OVRD_ALERT; //Clear this status bit by writing a one into this spot
    }

    if (sysStat & bq796x0_UV) //Under voltage
    {
      Serial1.println("Under voltage alert!");
      sysNew |= bq796x0_UV; //Clear this status bit by writing a one into this spot
    }

    if (sysStat & bq796x0_OV) //Over voltage
    {
      Serial1.println("Over voltage alert!");
      sysNew |= bq796x0_OV; //Clear this status bit by writing a one into this spot
    }

    if (sysStat & bq796x0_SCD) //Short circuit detect
    {
      Serial1.println("Short Circuit alert!");
      sysNew |= bq796x0_SCD; //Clear this status bit by writing a one into this spot
    }

    if (sysStat & bq796x0_OCD) //Over current detect
    {
      Serial1.println("Over current alert!");
      //sysNew |= bq796x0_OCD; //Clear this status bit by writing a one into this spot
    }

    //Update the SYS_STAT with only the ones we want, only these bits will clear to zero
    registerWrite(bq796x0_SYS_STAT, sysNew); //address, value

    bq769x0_IRQ_Triggered = false; //Reset flag
  }

  byte sysVal = registerRead(bq796x0_SYS_CTRL2);
  //  byte sysVal = registerRead(bq796x0_SYS_STAT);

  Serial1.println("sysVal : " + String(sysVal));
  delay(2000);
}

void bq769x0IRQ()
{
  bq769x0_IRQ_Triggered = true;
}

//Initiates the first few I2C commands
//Returns true if we can verify communication
//Set CC_CFG to default 0x19
//Turn on the ADC
//Assume we are checking internal die temperatures (leave TEMP_SEL at zero)
//Configure the interrupts for Arduino Uno
//Read the Gain and Offset factory settings into global variables
boolean initBQ() //byte irqPin)
{
  //Test to see if we have correct I2C communication
  //byte testByte = registerRead(bq796x0_OV_TRIP); //Should be something other than zero on POR
  byte testByte = registerRead(bq796x0_ADCGAIN2); //Should be something other than zero on POR

  for (byte x = 0; x < 10 && testByte == 0; x++)
  {
    Serial1.print(".");
    testByte = registerRead(bq796x0_ADCGAIN2);
    delay(100);
  }
  //if(testByte == 0x00) return false; //Something is very wrong. Check wiring.

  //"For optimal performance, [CC_CFG] should be programmed to 0x19 upon device startup." page 40
  registerWrite(bq796x0_CC_CFG, 0x19); //address, value

  //Double check that ADC is enabled
  byte sysVal = registerRead(bq796x0_SYS_CTRL1);
  if (sysVal & bq796x0_ADC_EN)
  {
    Serial1.println("ADC Already Enabled");
  }
  sysVal |= bq796x0_ADC_EN;                 //Set the ADC_EN bit
  registerWrite(bq796x0_SYS_CTRL1, sysVal); //address, value

  //Enable countinous reading of the Coulomb Counter
  sysVal = registerRead(bq796x0_SYS_CTRL2);
  sysVal |= bq796x0_CC_EN;                  //Set the CC_EN bit
  registerWrite(bq796x0_SYS_CTRL2, sysVal); //address, value
  //Serial1.println("Coulomb counter enabled");

  //Attach interrupt
  // pinMode(irqPin, INPUT); //No pull up

  // if(irqPin == 2)
  //Interrupt zero on Uno is pin 2
  //   attachInterrupt(0, bq769x0IRQ, RISING);
  // else if (irqPin == 3)
  //Interrupt one on Uno is pin 3
  //  attachInterrupt(1, bq769x0IRQ, RISING);
  //  else
  Serial1.println("irqPin invalid. Alert IRQ not enabled.");

  //Gain and offset are used in multiple functions
  //Read these values into global variables
  gain = readGAIN() / (float)1000; //Gain is in uV so this converts it to mV. Example: 0.370mV/LSB
  offset = readADCoffset();        //Offset is in mV. Example: 65mV

  Serial1.print("gain: ");
  Serial1.print(gain);
  Serial1.println("uV/LSB");

  Serial1.print("offset: ");
  Serial1.print(offset);
  Serial1.println("mV");

  //Read the system status register
  byte sysStat = registerRead(bq796x0_SYS_STAT);
  if (sysStat & bq796x0_DEVICE_XREADY)
  {
    Serial1.println("Device X Ready Error");
    //Try to clear it
    for (int y = 0; y < 10; y++)
    {

      registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY);
      delay(500);
    }

    //Check again
    byte sysStat = registerRead(bq796x0_SYS_STAT);
    if (sysStat & bq796x0_DEVICE_XREADY)
    {
      Serial1.println("Device X Ready Not Cleared");
    }
  }

  //Set any other settings such as OVTrip and UVTrip limits
  float under = readUVtrip();
  float over = readOVtrip();

  Serial1.print("Undervoltage trip: ");
  Serial1.print(under);
  Serial1.println("V");

  Serial1.print("Overvoltage trip: ");
  Serial1.print(over);
  Serial1.println("V");
  if (under != 3.32)
  {
    writeUVtrip(3.32); //Set undervoltage to 3.32V
    Serial1.print("New undervoltage trip: ");
    Serial1.print(readUVtrip());
    Serial1.println("V"); //should print 3.32V
  }

  if (over != 4.27)
  {
    writeOVtrip(4.27); //Set overvoltage to 4.27V
    Serial1.print("New overvoltage trip: ");
    Serial1.print(readOVtrip());
    Serial1.println("V"); //should print 4.27V
  }

  return true;
}

//Pretty print the pack voltages
void displayVoltages(boolean print_out)
{
  //Serial1.println("Voltages===>:" + String(NUMBER_OF_CELLS));
  for (int i = 0; i < 15; i++)
  {
    if (i + 1 == 14)
    {
      cellVoltage[i] = readCellVoltage(15, false);
    }
    else
    {
      cellVoltage[i] = readCellVoltage(i + 1, false);
    }
  }

  if (print_out)
  {
    for (int i = 0; i < NUMBER_OF_CELLS - 1; i++)
    {
      Serial1.println("Cell : " + String(i + 1) + " " + String(cellVoltage[i]));
    }
  }
}

//Enable or disable the balancing of a given cell
//Give me a cell # and whether you want balancing or not
void enableBalancing(byte cellNumber, boolean enabled)
{
  byte startingBit, cellRegister;

  if (cellNumber < 1 || cellNumber > 15)
    return; //Out of range

  if (cellNumber < 6)
  {
    startingBit = 0;
    cellRegister = bq796x0_CELLBAL1;
  }
  else if (cellNumber < 11)
  {
    startingBit = 6;                 //The 2nd Cell balancing register starts at CB6
    cellRegister = bq796x0_CELLBAL2; //If the cell number is 6-10 then we are in the 2nd cell balancing register
  }
  else if (cellNumber < 16)
  {
    startingBit = 11;
    cellRegister = bq796x0_CELLBAL3;
  }

  byte cell = registerRead(cellRegister); //Read what is currently there

  if (enabled)
    cell |= (1 << (cellNumber - startingBit)); //Set bit for balancing
  else
    cell &= ~(1 << (cellNumber - startingBit)); //Clear bit to disable balancing

  registerWrite(cellRegister, cell); //Make it so
}

//Calling this function will put the IC into ultra-low power SHIP mode
//A boot signal is needed to get back to NORMAL mode
void enterSHIPmode(void)
{
  //This function is currently untested but should work
  byte sysValue = registerRead(bq796x0_SYS_CTRL1);

  sysValue &= 0xFC; //Step 1: 00
  registerWrite(bq796x0_SYS_CTRL1, sysValue);

  sysValue |= 0x03; //Step 2: non-01
  registerWrite(bq796x0_SYS_CTRL1, sysValue);

  sysValue &= ~(1 << 1); //Step 3: 01
  registerWrite(bq796x0_SYS_CTRL1, sysValue);

  sysValue = (sysValue & 0xFC) | (1 << 1); //Step 4: 10
  registerWrite(bq796x0_SYS_CTRL1, sysValue);

  //bq should now be in powered down SHIP mode and will not respond to commands
  //Boot on VS1 required to start IC
}

////Given a cell number, return the cell voltage
////Vcell = GAIN * ADC(cell) + OFFSET
////Conversion example from datasheet: 14-bit ADC = 0x1800, Gain = 0x0F, Offset = 0x1E = 2.365V
//float readCellVoltage(byte cellNumber, boolean divide)
//{
//  if (cellNumber < 1 || cellNumber > 15) return (-0); //Return error
//
//  //Serial1.print("Read cell number: ");
//  //Serial1.println(cellNumber);
//
//  //Reduce the caller's cell number by one so that we get register alignment
//  cellNumber--;
//
//  byte registerNumber = bq796x0_VC1_HI + (cellNumber * 2);
//
//  //Serial1.print("register: 0x");
//  //Serial1.println(registerNumber, HEX);
//
//  int cellValue = registerDoubleRead(registerNumber);
//
//  //int cellValue = 0x1800; //6,144 - Should return 2.365
//  //int cellValue = 0x1F10l; //Should return 3.052
//
//  //Cell value should now contain a 14 bit value
//
//  //Serial1.print("Cell value (dec): ");
//  //Serial1.println(cellValue);
//
//  if (cellValue == 0) return (0);
//
//  float cellVoltage = cellValue * gain + offset; //0x1800 * 0.37 + 60 = 3,397mV
//
//  if (divide)
//  {
//    cellVoltage /= (float)1000;
//  }
//
//
//  //Serial1.print("Cell voltage: ");
//  //Serial1.println(cellVoltage, 3);
//
//  return (cellVoltage);
//}

//Given a thermistor number return the temperature in C
//Valid thermistor numbers are 1 to 3 for external and 0 to read the internal die temp
//If you switch between internal die and external TSs this function will delay 2 seconds
int readTemp(byte thermistorNumber)
{
  //There are 3 external thermistors (optional) and an internal temp reading (channel 0)
  if (thermistorNumber < 0 || thermistorNumber > 3)
    return (-0); //Return error

  //Serial1.print("Read thermistor number: ");
  //Serial1.println(thermistorNumber);

  byte sysValue = registerRead(bq796x0_SYS_CTRL1);

  if (thermistorNumber > 0)
  {
    //See if we need to switch between internal die temp and external thermistor
    if ((sysValue & bq796x0_TEMP_SEL) == 0)
    {
      //Bad news, we have to do a switch and wait 2 seconds
      //Set the TEMP_SEL bit
      sysValue |= bq796x0_TEMP_SEL;
      sendx(bq796x0_SYS_CTRL1, sysValue); //address, value

      Serial1.println("Waiting 2 seconds to switch thermistors");
      delay(2000);
    }

    int registerNumber = bq796x0_TS1_HI + ((thermistorNumber - 1) * 2);
    int thermValue = registerDoubleRead(registerNumber);

    //Therm value should now contain a 14 bit value

    Serial1.print("Therm value: 0x");
    Serial1.println(thermValue, HEX); //0xC89 = 3209

    float thermVoltage = thermValue * (float)382; //0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
    thermVoltage /= (float)1000000;               //Convert to V

    Serial1.print("thermVoltage: ");
    Serial1.println(thermVoltage, 3);

    float thermResistance = ((float)10000 * thermVoltage) / (3.3 - thermVoltage);

    Serial1.print("thermResistance: ");
    Serial1.println(thermResistance);

    //We now have thermVoltage and resistance. With a datasheet for the NTC 103AT thermistor we could
    //calculate temperature.
    int temperatureC = thermistorLookup(thermResistance);

    Serial1.print("temperatureC: ");
    Serial1.println(temperatureC);

    return (temperatureC);
  }
  else if (thermistorNumber == 0)
  {
    //See if we need to switch between internal die temp and external thermistor
    if ((sysValue & 1 << 3) != 0)
    {
      //Bad news, we have to do a switch and wait 2 seconds
      //Clear the TEMP_SEL bit
      sysValue &= ~(1 << 3);
      registerWrite(bq796x0_SYS_CTRL1, sysValue); //address, value

      Serial1.println("Waiting 2 seconds to switch to internal die thermistors");
      delay(2000);
    }

    int thermValue = registerDoubleRead(bq796x0_TS1_HI); //There are multiple internal die temperatures. We are only going to grab 1.

    //Therm value should now contain a 14 bit value
    //Serial1.print("Therm value: 0x");
    //Serial1.println(thermValue, HEX);

    float thermVoltage = thermValue * (float)382; //0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
    thermVoltage /= (float)1000000;               //Convert to V

    //Serial1.print("thermVoltage: ");
    //Serial1.println(thermVoltage, 3);

    float temperatureC = 25.0 - ((thermVoltage - 1.2) / 0.0042);

    //Serial1.print("temperatureC: ");
    //Serial1.println(temperatureC);

    //float temperatureF = (temperatureC * ((float)9/5)) + 32;

    //Serial1.print("temperatureF: ");
    //Serial1.println(temperatureF);

    return ((int)temperatureC);
  }
}

//Returns the coulomb counter value in microVolts
//Example: 84,400uV
//Coulomb counter is enabled during bqInit(). We do not use oneshot.
//If the counter is enabled in ALWAYS ON mode it will set the ALERT pin every 250ms. You can respond to this however you want.
//Host may clear the CC_READY bit or let it stay at 1.
float readCoulombCounter(void)
{
  int count = registerDoubleRead(bq796x0_CC_HI);

  //int count = 0xC350; //Test. Should report -131,123.84

  float count_uV = count * 8.44; //count should be naturally in 2's compliment. count_uV is now in uV

  return (count_uV);
}

//Returns the pack voltage in volts
//Vbat = 4 * GAIN * ADC(cell) + (# of cells * offset)
float readPackVoltage(void)
{
  unsigned int packADC = registerDoubleRead(bq796x0_BAT_HI);

  //Serial1.print("packADC = ");
  //Serial1.println(packADC);

  //packADC = 0x6DDA; //28,122 Test. Should report something like 42.520V

  //packADC = 35507
  //gain = 0.38uV/LSB
  //offset = 47mV
  //53970
  float packVoltage = 4 * gain * packADC;    //53970 in uV?
  packVoltage += (NUMBER_OF_CELLS * offset); //Should be in mV

  return (packVoltage / (float)1000); //Convert to volts
}

//Reads the gain registers and calculates the system's factory trimmed gain
//GAIN = 365uV/LSB + (ADCGAIN<4:0>) * 1uV/LSB
//ADC gain comes from two registers that have to be moved around and combined.
int readGAIN(void)
{
  byte val1 = registerRead(bq796x0_ADCGAIN1);
  byte val2 = registerRead(bq796x0_ADCGAIN2);
  val1 &= 0b00001100; //There are some unknown reservred bits around val1 that need to be cleared

  //Recombine the bits into one ADCGAIN
  byte adcGain = (val1 << 1) | (val2 >> 5);

  int gain = 365 + adcGain;

  return (gain);
}

//Returns the factory trimmed ADC offset
//Offset is -127 to 128 in mV
int readADCoffset(void)
{
  //Here we need to convert a 8bit 2's compliment to a 16 bit int
  char offset = registerRead(bq796x0_ADCOFFSET);

  return ((int)offset); //8 bit char is now a full 16-bit int. Easier math later on.
}

//Returns the over voltage trip threshold
//Default is 0b.10.OVTRIP(0xAC).1000 = 0b.10.1010.1100.1000 = 0x2AC8 = 10,952
//OverVoltage = (OV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (10,952 * 0.370) + 56mV = 4.108V
float readOVtrip(void)
{
  int trip = registerRead(bq796x0_OV_TRIP);

  trip <<= 4; //Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
  trip |= 0x2008;

  float overVoltage = ((float)trip * gain) + offset;
  overVoltage /= 1000; //Convert to volts

  //Serial1.print("overVoltage should be around 4.108: ");
  //Serial1.println(overVoltage, 3);

  return (overVoltage);
}

//Given a voltage (4.22 for example), set the over voltage trip register
//Example: voltage = 4.2V = (4200mV - 56mV) / 0.370mv = 11,200
//11,200 = 0x2BC0 =
void writeOVtrip(float tripVoltage)
{
  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
  registerWrite(bq796x0_OV_TRIP, val);    //address, value
}

//Returns the under voltage trip threshold
//Default is 0b.01.UVTRIP(0x97).0000 = 0x1970 = 6,512
//UnderVoltage = (UV_TRIP * GAIN) + ADCOFFSET
//Gain and Offset is different for each IC
//Example: voltage = (6,512 * 0.370) + 56mV = 2.465V
float readUVtrip(void)
{
  int trip = registerRead(bq796x0_UV_TRIP);

  trip <<= 4; //Shuffle the bits to align to 0b.01.XXXX.XXXX.0000
  trip |= 0x1000;

  float underVoltage = ((float)trip * gain) + offset;
  underVoltage /= 1000; //Convert to volts

  //Serial1.print("underVoltage should be around 2.465: ");
  //Serial1.println(underVoltage, 3);

  return (underVoltage);
}

//Given a voltage (2.85V for example), set the under voltage trip register
void writeUVtrip(float tripVoltage)
{
  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
  registerWrite(bq796x0_UV_TRIP, val);    //address, value
}

//Under voltage and over voltage use the same rules for calculating the 8-bit value
//Given a voltage this function uses gain and offset to get a 14 bit value
//Then strips that value down to the middle-ish 8-bits
//No registers are written, that's up to the caller
byte tripCalculator(float tripVoltage)
{
  tripVoltage *= 1000; //Convert volts to mV

  //Serial1.print("tripVoltage to be: ");
  //Serial1.println(tripVoltage, 3);

  tripVoltage -= offset;
  tripVoltage /= gain;

  int tripValue = (int)tripVoltage; //We only want the integer - drop decimal portion.

  //Serial1.print("tripValue should be something like 0x2BC0: ");
  //Serial1.println(tripValue, HEX);

  tripValue >>= 4;     //Cut off lower 4 bits
  tripValue &= 0x00FF; //Cut off higher bits

  //Serial1.print("About to report tripValue: ");
  //Serial1.println(tripValue, HEX);

  return (tripValue);
}

// ================= sundaya ==================//

uint8_t _crc8_ccitt_update(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;
  data = inCrc ^ inData;

  for (i = 0; i < 8; i++)
  {
    if ((data & 0x80) != 0)
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
      data <<= 1;
  }

  return data;
}

void sendx(byte address, byte data)
{
  char buf[3];
  buf[0] = (char)address;
  buf[1] = data;
  uint8_t crc = 0;
  int I2CAddress = bqI2CAddress;

  // note that writes to the bq769x0 IC are: 1) start - 2) address - 3) address - 4) data - 5) CRC8 - 6) stop bit
  Wire.beginTransmission(bqI2CAddress); // writes start bit - the first step
  Wire.write(buf[0]);                   // writes register address
  Wire.write(buf[1]);                   // writes data - the fourth step

  boolean crcEnabled = true;
  if (crcEnabled == true)
  {
    // CRC is calculated over the slave address (including R/W bit), register address, and data.
    crc = _crc8_ccitt_update(crc, (I2CAddress << 1) | 0);
    crc = _crc8_ccitt_update(crc, buf[0]);
    crc = _crc8_ccitt_update(crc, buf[1]);
    buf[2] = crc;

    Wire.write(buf[2]); // writes CRC
  }

  Wire.endTransmission();
}

// ================= sundaya ==================//

//Write a given value to a given register
void registerWrite(byte regAddress, byte regData)
{

  if (bq76940_crc)
  {
    sendx(regAddress, regData);
  }

  else
  {
    Wire.beginTransmission(bqI2CAddress);
    Wire.write(regAddress);
    Wire.endTransmission();

    Wire.beginTransmission(bqI2CAddress);
    Wire.write(regAddress);
    Wire.write(regData);
    Wire.endTransmission();
  }
}

//Returns a given register
byte registerRead(byte regAddress)
{
  Wire.beginTransmission(bqI2CAddress);
  //Here's where I2C can time out
  Wire.write(regAddress);
  Wire.endTransmission();

  Wire.requestFrom(bqI2CAddress, 1);

  /*byte counter = 0;
    while(Wire.available() == 0)
    {
    delay(1);
    if(counter++ > 250) return(0); //Return with error
    }*/

  return (Wire.read());
}

////Returns the atmoic int from two sequentials reads
//int registerDoubleRead(byte regAddress)
//{
//  Wire.beginTransmission(bqI2CAddress);
//  Wire.write(regAddress);
//  Wire.endTransmission();
//
//  Wire.requestFrom(bqI2CAddress, 2);
//
//  /*byte counter = 0;
//    while(Wire.available() < 2)
//    {
//    Serial1.print(".");
//    if(counter++ > MAX_I2C_TIME)
//    {
//    return(-1); //Time out error
//    }
//    delay(1);
//    }*/
//
//  byte reg1 = Wire.read();
//  byte reg2 = Wire.read();
//
//  //Serial1.print("reg1: 0x");
//  //Serial1.print(reg1, HEX);
//  //Serial1.print(" reg2: 0x");
//  //Serial1.println(reg2, HEX);
//
//  int combined = (int)reg1 << 8;
//  combined |= reg2;
//
//  return (combined);
//}

//Given a resistance on a super common 103AT-2 thermistor, return a temperature in C
//This is a poor way of converting the resistance to temp but it works for now
//From: http://www.rapidonline.com/pdf/61-0500e.pdf
int thermistorLookup(float resistance)
{
  //Resistance is coming in as Ohms, this lookup table assume kOhm
  resistance /= 1000; //Convert to kOhm

  int temp = 0;

  if (resistance > 329.5)
    temp = -50;
  if (resistance > 247.7)
    temp = -45;
  if (resistance > 188.5)
    temp = -40;
  if (resistance > 144.1)
    temp = -35;
  if (resistance > 111.3)
    temp = -30;
  if (resistance > 86.43)
    temp = -25;
  if (resistance > 67.77)
    temp = -20;
  if (resistance > 53.41)
    temp = -15;
  if (resistance > 42.47)
    temp = -10;
  if (resistance > 33.90)
    temp = -5;
  if (resistance > 27.28)
    temp = 0;
  if (resistance > 22.05)
    temp = 5;
  if (resistance > 17.96)
    temp = 10;
  if (resistance > 14.69)
    temp = 15;
  if (resistance > 12.09)
    temp = 20;
  if (resistance > 10.00)
    temp = 25;
  if (resistance > 8.313)
    temp = 30;

  return (temp);
}

//
//
//
////#include <Wire_slave.h>
//#include <Wire.h>
////#include<SoftWire.h>
//#include "SparkFun_BQ769x0.h"
//
////The bq769x0 without CRC has the 7-bit address 0x08. The bq769x0 with CRC has the address 0x18.
////Please see the datasheet for more info
//int bqI2CAddress = 0x08; //7-bit I2C address
//
////My pack is a 15 cell lipo that runs at 48V. Your pack may vary. Read the datasheet!
////This code is written for the bq76940. The bq76940 supports 9 to 15 cells.
//#define NUMBER_OF_CELLS 16
//
////Max number of ms before timeout error. 100 is pretty good
//#define MAX_I2C_TIME 100
//
//volatile boolean bq769x0_IRQ_Triggered = false; //Keeps track of when the Alert pin has been raised
//
//float gain = 0; //These are two internal factory set values.
//int offset = 0; //We read them once at boot up and use them in many functions
//
//long lastTime; //Used to blink the status LED
//
//long totalCoulombCount = 0; //Keeps track of overall pack fuel gauage
//
//float cellVoltage[NUMBER_OF_CELLS + 1]; //Keeps track of the cell voltages
//
////GPIO declarations
////-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//
////byte irqPin = ; //Interrupt enabled, connected to bq pin ALERT
//byte statLED = PC14; //Ob board status LED
//
////-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//
//
//void requestEvent()
//{
//  Wire.beginTransmission(8);
//  Wire.write(bq796x0_SYS_CTRL2);
//  Wire.endTransmission();
//  Wire.beginTransmission(8);
//  Wire.write(bq796x0_DSG_ON);
//  Wire.endTransmission();
//}
//
//
//void setup()
//{
//
//  pinMode(PB14,OUTPUT);
//  digitalWrite(PB14, HIGH);
//  pinMode(PB4,OUTPUT);
//  digitalWrite(PB4, LOW);
//
//  delay(500);
//
//  Serial1.begin(115200);
//  Serial1.println("bq76940 example");
//
//  Wire.begin(); //Start I2C communication
//
//    pinMode(statLED, OUTPUT);
//  digitalWrite(statLED, LOW); //Turn off the LED for now
//
//  if(initBQ() == false) //Call init with pin 2 (IRQ0) or 3 (IRQ1)
//  {
//    Serial1.println("bq76940 failed to respond - check your wiring");
//    Serial1.println("Hanging.");
//    while(1);
//  }
//  else
//  {
//    Serial1.println("bq76940 initialized!");
//  }
//
//  lastTime = millis();
//
//  //Testing
//
//  //  Serial1.println();
//
//  /*
//  readCoulombCounter();
//
//   readTemp(0); //Read the die temperature. Should report something like room Temp
//
//   delay(500);
//   while(1);
//
//
//   enableBalancing(1, true); //test
//   */
//
////   byte sysVal = registerRead(bq796x0_SYS_CTRL2);
////   sysVal |= bq796x0_CHG_OFF;
////   registerWrite(bq796x0_SYS_CTRL2, sysVal);
//   Serial1.println("send1");
//   sendx(bq796x0_SYS_CTRL2,bq796x0_DSG_ON);
//  delay(1000);
//   Serial1.println("send2");
//}
//
//void send_on ()
//{
//  Wire.beginTransmission(bqI2CAddress);
//  //Wire.write(bq796x0_SYS_CTRL2);
//  Wire.endTransmission();
//  byte error;
//  error = Wire.endTransmission();
//  Serial1.println("gagal1 : " + String(error));
//
//  Wire.beginTransmission(bqI2CAddress);
//  Wire.write(bq796x0_SYS_CTRL2);
//  Wire.write(bq796x0_DSG_ON);
//  Wire.endTransmission();
//  error;
//  error = Wire.endTransmission();
//  Serial1.println("gagal2 : " + String(error));
//}
//
//void loop1()
//{
//  if(Serial1.available())
//  {
//    int incoming = Serial1.parseInt();
//
//    if(incoming == 1)
//    {
//      Serial1.println("mode on");
//      send_on ()     ;
//    }
//  }
//}
//
//void loop()
//{
//  //Each second make a reading of cell voltages
//  //And blink the status LED
//  if(millis() - lastTime > 1000)
//  {
//    for(int i = 0 ; i < NUMBER_OF_CELLS ; i++)
//    {
//      cellVoltage[i] = readCellVoltage(i);
//    }
//
//   // Serial1.println();
//
//    //Toggle stat LED
//    if(digitalRead(statLED) == HIGH)
//      digitalWrite(statLED, LOW);
//    else
//    {
//      digitalWrite(statLED, HIGH);
//      displayVoltages();
//    }
//
//    int temp = readTemp(1);
//    Serial1.print("Die temp = ");
//    Serial1.println(temp);
//
//    float packV = readPackVoltage();
//    Serial1.print("PackV = ");
//    Serial1.println(packV);
//
//    //Calc the pack voltage by hand?
//    float totalVoltage = 0;
//    for(int i = 0 ; i < NUMBER_OF_CELLS ; i++)
//    {
//
//      totalVoltage += cellVoltage[i];
//    }
//
//    Serial1.print("PackV manual = ");
//    Serial1.println(totalVoltage);
//
//
//    lastTime = millis();
//  }
//
//  //For every IRQ event read the flags and update the coulomb counter and other major events
//  if(bq769x0_IRQ_Triggered == true)
//  {
//    //Read the status register and update if needed
//    byte sysStat = registerRead(bq796x0_SYS_STAT);
//
//    Serial1.print("sysStat: 0x");
//    Serial1.println(sysStat, HEX);
//
//    //Double check that ADC is enabled
//    //byte sysVal = registerRead(bq796x0_SYS_CTRL1);
//    //if(sysVal & bq796x0_ADC_EN)
//    //{
//    //  Serial1.println("ADC Enabled");
//    //}
//
//    //We need to write 1s into all the places we want a zero, but not overwrite the 1s we want left alone
//    byte sysNew = 0;
//
//    //Check for couloumb counter read
//    if(sysStat & bq796x0_CC_READY)
//    {
//      Serial1.println("CC Ready");
//      totalCoulombCount += readCoulombCounter(); //Add this 250ms reading to the global fuel gauge
//      sysNew |= bq796x0_CC_READY; //Clear this status bit by writing a one into this spot
//    }
//
//    if(sysStat & bq796x0_DEVICE_XREADY) //Internal fault
//    {
//      Serial1.println("Internal fault");
//      sysNew |= bq796x0_DEVICE_XREADY; //Clear this status bit by writing a one into this spot
//    }
//
//    if(sysStat & bq796x0_OVRD_ALERT) //Alert pin is being pulled high externally?
//    {
//      Serial1.println("Override alert");
//      sysNew |= bq796x0_OVRD_ALERT; //Clear this status bit by writing a one into this spot
//    }
//
//    if(sysStat & bq796x0_UV) //Under voltage
//    {
//      Serial1.println("Under voltage alert!");
//      sysNew |= bq796x0_UV; //Clear this status bit by writing a one into this spot
//    }
//
//    if(sysStat & bq796x0_OV) //Over voltage
//    {
//      Serial1.println("Over voltage alert!");
//      sysNew |= bq796x0_OV; //Clear this status bit by writing a one into this spot
//    }
//
//    if(sysStat & bq796x0_SCD) //Short circuit detect
//    {
//      Serial1.println("Short Circuit alert!");
//      //sysNew |= bq796x0_SCD; //Clear this status bit by writing a one into this spot
//    }
//
//    if(sysStat & bq796x0_OCD) //Over current detect
//    {
//      Serial1.println("Over current alert!");
//      //sysNew |= bq796x0_OCD; //Clear this status bit by writing a one into this spot
//    }
//
//    //Update the SYS_STAT with only the ones we want, only these bits will clear to zero
//    registerWrite(bq796x0_SYS_STAT, sysNew); //address, value
//
//    bq769x0_IRQ_Triggered = false; //Reset flag
//  }
//
//  if(Serial1.available()>0)
//  {
//    int incoming = Serial1.parseInt();
//    Serial1.println("data " + String(incoming));
//
//    if(incoming == 1)
//    {
//      Serial1.println("Entering ship mode");
//      enterSHIPmode();
//    }
//
//    if(incoming == 2)
//    {
//      Serial1.println("ON Load");
//      registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_ON);
//    }
//
//    if(incoming == 3)
//    {
//      Serial1.println("OFF Load");
//      registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_OFF);
//    }
//
//  }
//
//  //Display cell voltages
//
//
//  //Display CC fuel gauge
//  //totalCoulomb = readCoulombCounter();
//
//  //Display temperatures
//
//  //Display any other register info
////byte sysVal = registerRead(bq796x0_SYS_CTRL1
//
////  registerWrite(bq796x0_SYS_STAT, bq796x0_CC_READY); //address, value
////  registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY);
////  registerWrite(bq796x0_SYS_STAT, bq796x0_OVRD_ALERT);
////  registerWrite(bq796x0_SYS_STAT, bq796x0_UV);
////  registerWrite(bq796x0_SYS_STAT, bq796x0_OV);
////  registerWrite(bq796x0_SYS_STAT, bq796x0_SCD);
// // registerWrite(bq796x0_SYS_STAT, 0x00);
//
//  byte sysVal = registerRead(bq796x0_SYS_CTRL2);
//// // registerWrite(bq796x0_SYS_CTRL2, bq796x0_DSG_ON);
////  sysVal |= bq796x0_DSG_ON; //Set the CC_EN bit
//////  registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY); //address, value
////  registerWrite(bq796x0_SYS_CTRL2, sysVal); //address, value
// // registerWrite(bq796x0_SYS_CTRL2, bq796x0_CHG_ON); //address, value
//
//
//  Serial1.println("sysVal : " + String(sysVal));
// // byte sysVal1 = registerRead(bq796x0_SYS_STAT);
////  Serial1.println("sysVal2 : " + String(sysVal1));
//  delay(2000);
//
//}
//
//// this is irq handler for bq769x0 interrupts, has to return void and take no arguments
//// always make code in interrupt handlers fast and short
//void bq769x0IRQ()
//{
//  bq769x0_IRQ_Triggered = true;
//}
//
////Initiates the first few I2C commands
////Returns true if we can verify communication
////Set CC_CFG to default 0x19
////Turn on the ADC
////Assume we are checking internal die temperatures (leave TEMP_SEL at zero)
////Configure the interrupts for Arduino Uno
////Read the Gain and Offset factory settings into global variables
//boolean initBQ()//byte irqPin)
//{
//  //Test to see if we have correct I2C communication
//  //byte testByte = registerRead(bq796x0_OV_TRIP); //Should be something other than zero on POR
//  byte testByte = registerRead(bq796x0_ADCGAIN2); //Should be something other than zero on POR
//
//  for(byte x = 0 ; x < 10 && testByte == 0 ; x++)
//  {
//    Serial1.print(".");
//    testByte = registerRead(bq796x0_ADCGAIN2);
//    delay(100);
//  }
//  //if(testByte == 0x00) return false; //Something is very wrong. Check wiring.
//
//  //"For optimal performance, [CC_CFG] should be programmed to 0x19 upon device startup." page 40
//  registerWrite(bq796x0_CC_CFG, 0x19); //address, value
//
//
//  //Double check that ADC is enabled
//  byte sysVal = registerRead(bq796x0_SYS_CTRL1);
//  if(sysVal & bq796x0_ADC_EN)
//  {
//    Serial1.println("ADC Already Enabled");
//  }
//  sysVal |= bq796x0_ADC_EN; //Set the ADC_EN bit
//  registerWrite(bq796x0_SYS_CTRL1, sysVal); //address, value
//
//  //Enable countinous reading of the Coulomb Counter
//  sysVal = registerRead(bq796x0_SYS_CTRL2);
//  sysVal |= bq796x0_CC_EN; //Set the CC_EN bit
//  registerWrite(bq796x0_SYS_CTRL2, sysVal); //address, value
//  //Serial1.println("Coulomb counter enabled");
//
//  //Attach interrupt
// // pinMode(irqPin, INPUT); //No pull up
//
// // if(irqPin == 2)
//    //Interrupt zero on Uno is pin 2
// //   attachInterrupt(0, bq769x0IRQ, RISING);
// // else if (irqPin == 3)
//    //Interrupt one on Uno is pin 3
//  //  attachInterrupt(1, bq769x0IRQ, RISING);
////  else
//    Serial1.println("irqPin invalid. Alert IRQ not enabled.");
//
//  //Gain and offset are used in multiple functions
//  //Read these values into global variables
//  gain = readGAIN() / (float)1000; //Gain is in uV so this converts it to mV. Example: 0.370mV/LSB
//  offset = readADCoffset(); //Offset is in mV. Example: 65mV
//
//  Serial1.print("gain: ");
//  Serial1.print(gain);
//  Serial1.println("uV/LSB");
//
//  Serial1.print("offset: ");
//  Serial1.print(offset);
//  Serial1.println("mV");
//
//  //Read the system status register
//  byte sysStat = registerRead(bq796x0_SYS_STAT);
//  if(sysStat & bq796x0_DEVICE_XREADY)
//  {
//    Serial1.println("Device X Ready Error");
//    //Try to clear it
//    for (int y=0;y<10;y++)
//    {
//
//      registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY);
//      delay(500);
//    }
//
//
//
//    //Check again
//    byte sysStat = registerRead(bq796x0_SYS_STAT);
//    if(sysStat & bq796x0_DEVICE_XREADY)
//    {
//      Serial1.println("Device X Ready Not Cleared");
//    }
//  }
//
//  //Set any other settings such as OVTrip and UVTrip limits
//  float under = readUVtrip();
//  float over = readOVtrip();
//
//  Serial1.print("Undervoltage trip: ");
//  Serial1.print(under);
//  Serial1.println("V");
//
//  Serial1.print("Overvoltage trip: ");
//  Serial1.print(over);
//  Serial1.println("V");
//  if(under != 3.32)
//  {
//    writeUVtrip(3.32); //Set undervoltage to 3.32V
//    Serial1.print("New undervoltage trip: ");
//    Serial1.print(readUVtrip());
//    Serial1.println("V"); //should print 3.32V
//  }
//
//  if(over != 4.27)
//  {
//    writeOVtrip(4.27); //Set overvoltage to 4.27V
//    Serial1.print("New overvoltage trip: ");
//    Serial1.print(readOVtrip());
//    Serial1.println("V"); //should print 4.27V
//  }
//
//  return true;
//}
//
////Pretty print the pack voltages
//void displayVoltages(void)
//{
//  Serial1.println("Voltages:");
//
//  for(int i = 1 ; i < NUMBER_OF_CELLS  ; i++)
//  {
//    Serial1.print("[");
//    Serial1.print(i);
//    Serial1.print("]");
//
//    Serial1.print(cellVoltage[i], 2);
//    Serial1.print(" ");
//
//    if(i % 5 == 0) Serial1.println();
//  }
//}
//
////Enable or disable the balancing of a given cell
////Give me a cell # and whether you want balancing or not
//void enableBalancing(byte cellNumber, boolean enabled)
//{
//  byte startingBit, cellRegister;
//
//  if(cellNumber < 1 || cellNumber > 15) return; //Out of range
//
//  if(cellNumber < 6)
//  {
//    startingBit = 0;
//    cellRegister = bq796x0_CELLBAL1;
//  }
//  else if(cellNumber < 11)
//  {
//    startingBit = 6; //The 2nd Cell balancing register starts at CB6
//    cellRegister = bq796x0_CELLBAL2; //If the cell number is 6-10 then we are in the 2nd cell balancing register
//  }
//  else if(cellNumber < 16)
//  {
//    startingBit = 11;
//    cellRegister = bq796x0_CELLBAL3;
//  }
//
//  byte cell = registerRead(cellRegister); //Read what is currently there
//
//  if(enabled)
//    cell |= (1<<(cellNumber - startingBit)); //Set bit for balancing
//  else
//    cell &= ~(1<<(cellNumber - startingBit)); //Clear bit to disable balancing
//
//  registerWrite(cellRegister, cell); //Make it so
//}
//
////Calling this function will put the IC into ultra-low power SHIP mode
////A boot signal is needed to get back to NORMAL mode
//void enterSHIPmode(void)
//{
//  //This function is currently untested but should work
//  byte sysValue = registerRead(bq796x0_SYS_CTRL1);
//
//  sysValue &= 0xFC; //Step 1: 00
//  registerWrite(bq796x0_SYS_CTRL1, sysValue);
//
//  sysValue |= 0x03; //Step 2: non-01
//  registerWrite(bq796x0_SYS_CTRL1, sysValue);
//
//  sysValue &= ~(1<<1); //Step 3: 01
//  registerWrite(bq796x0_SYS_CTRL1, sysValue);
//
//  sysValue = (sysValue & 0xFC) | (1<<1); //Step 4: 10
//  registerWrite(bq796x0_SYS_CTRL1, sysValue);
//
//  //bq should now be in powered down SHIP mode and will not respond to commands
//  //Boot on VS1 required to start IC
//}
//
////Given a cell number, return the cell voltage
////Vcell = GAIN * ADC(cell) + OFFSET
////Conversion example from datasheet: 14-bit ADC = 0x1800, Gain = 0x0F, Offset = 0x1E = 2.365V
//float readCellVoltage(byte cellNumber)
//{
//  if(cellNumber < 1 || cellNumber > 15) return(-0); //Return error
//
//  //Serial1.print("Read cell number: ");
//  //Serial1.println(cellNumber);
//
//  //Reduce the caller's cell number by one so that we get register alignment
//  cellNumber--;
//
//  byte registerNumber = bq796x0_VC1_HI + (cellNumber * 2);
//
//  //Serial1.print("register: 0x");
//  //Serial1.println(registerNumber, HEX);
//
//  int cellValue = registerDoubleRead(registerNumber);
//
//  //int cellValue = 0x1800; //6,144 - Should return 2.365
//  //int cellValue = 0x1F10l; //Should return 3.052
//
//  //Cell value should now contain a 14 bit value
//
//  //Serial1.print("Cell value (dec): ");
//  //Serial1.println(cellValue);
//
//  if(cellValue == 0) return(0);
//
//  float cellVoltage = cellValue * gain + offset; //0x1800 * 0.37 + 60 = 3,397mV
//  cellVoltage /= (float)1000;
//
//  //Serial1.print("Cell voltage: ");
//  //Serial1.println(cellVoltage, 3);
//
//  return(cellVoltage);
//}
//
////Given a thermistor number return the temperature in C
////Valid thermistor numbers are 1 to 3 for external and 0 to read the internal die temp
////If you switch between internal die and external TSs this function will delay 2 seconds
//int readTemp(byte thermistorNumber)
//{
//  //There are 3 external thermistors (optional) and an internal temp reading (channel 0)
//  if(thermistorNumber < 0 || thermistorNumber > 3) return(-0); //Return error
//
//  //Serial1.print("Read thermistor number: ");
//  //Serial1.println(thermistorNumber);
//
//  byte sysValue = registerRead(bq796x0_SYS_CTRL1);
//
//  if(thermistorNumber > 0)
//  {
//    //See if we need to switch between internal die temp and external thermistor
//    if((sysValue & bq796x0_TEMP_SEL) == 0)
//    {
//      //Bad news, we have to do a switch and wait 2 seconds
//      //Set the TEMP_SEL bit
//      sysValue |= bq796x0_TEMP_SEL;
//      sendx(bq796x0_SYS_CTRL1, sysValue); //address, value
//
//        Serial1.println("Waiting 2 seconds to switch thermistors");
//      delay(2000);
//    }
//
//    int registerNumber = bq796x0_TS1_HI + ((thermistorNumber - 1) * 2);
//    int thermValue = registerDoubleRead(registerNumber);
//
//    //Therm value should now contain a 14 bit value
//
//    Serial1.print("Therm value: 0x");
//    Serial1.println(thermValue, HEX); //0xC89 = 3209
//
//    float thermVoltage = thermValue * (float)382; //0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
//    thermVoltage /= (float)1000000; //Convert to V
//
//    Serial1.print("thermVoltage: ");
//    Serial1.println(thermVoltage, 3);
//
//    float thermResistance = ((float)10000 * thermVoltage) / (3.3 - thermVoltage);
//
//    Serial1.print("thermResistance: ");
//    Serial1.println(thermResistance);
//
//    //We now have thermVoltage and resistance. With a datasheet for the NTC 103AT thermistor we could
//    //calculate temperature.
//    int temperatureC = thermistorLookup(thermResistance);
//
//    Serial1.print("temperatureC: ");
//    Serial1.println(temperatureC);
//
//    return(temperatureC);
//  }
//  else if(thermistorNumber == 0)
//  {
//    //See if we need to switch between internal die temp and external thermistor
//    if((sysValue & 1<<3) != 0)
//    {
//      //Bad news, we have to do a switch and wait 2 seconds
//      //Clear the TEMP_SEL bit
//      sysValue &= ~(1<<3);
//      registerWrite(bq796x0_SYS_CTRL1, sysValue); //address, value
//
//        Serial1.println("Waiting 2 seconds to switch to internal die thermistors");
//      delay(2000);
//    }
//
//    int thermValue = registerDoubleRead(bq796x0_TS1_HI); //There are multiple internal die temperatures. We are only going to grab 1.
//
//    //Therm value should now contain a 14 bit value
//    //Serial1.print("Therm value: 0x");
//    //Serial1.println(thermValue, HEX);
//
//    float thermVoltage = thermValue * (float)382; //0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
//    thermVoltage /= (float)1000000; //Convert to V
//
//    //Serial1.print("thermVoltage: ");
//    //Serial1.println(thermVoltage, 3);
//
//    float temperatureC = 25.0 - ((thermVoltage - 1.2) / 0.0042);
//
//    //Serial1.print("temperatureC: ");
//    //Serial1.println(temperatureC);
//
//    //float temperatureF = (temperatureC * ((float)9/5)) + 32;
//
//    //Serial1.print("temperatureF: ");
//    //Serial1.println(temperatureF);
//
//    return((int)temperatureC);
//  }
//
//}
//
////Returns the coulomb counter value in microVolts
////Example: 84,400uV
////Coulomb counter is enabled during bqInit(). We do not use oneshot.
////If the counter is enabled in ALWAYS ON mode it will set the ALERT pin every 250ms. You can respond to this however you want.
////Host may clear the CC_READY bit or let it stay at 1.
//float readCoulombCounter(void)
//{
//  int count = registerDoubleRead(bq796x0_CC_HI);
//
//  //int count = 0xC350; //Test. Should report -131,123.84
//
//  float count_uV = count * 8.44; //count should be naturally in 2's compliment. count_uV is now in uV
//
//  return(count_uV);
//}
//
////Returns the pack voltage in volts
////Vbat = 4 * GAIN * ADC(cell) + (# of cells * offset)
//float readPackVoltage(void)
//{
//  unsigned int packADC = registerDoubleRead(bq796x0_BAT_HI);
//
//  //Serial1.print("packADC = ");
//  //Serial1.println(packADC);
//
//  //packADC = 0x6DDA; //28,122 Test. Should report something like 42.520V
//
//  //packADC = 35507
//  //gain = 0.38uV/LSB
//  //offset = 47mV
//  //53970
//  float packVoltage = 4 * gain * packADC; //53970 in uV?
//  packVoltage += (NUMBER_OF_CELLS * offset); //Should be in mV
//
//  return(packVoltage / (float)1000); //Convert to volts
//}
//
////Reads the gain registers and calculates the system's factory trimmed gain
////GAIN = 365uV/LSB + (ADCGAIN<4:0>) * 1uV/LSB
////ADC gain comes from two registers that have to be moved around and combined.
//int readGAIN(void)
//{
//  byte val1 = registerRead(bq796x0_ADCGAIN1);
//  byte val2 = registerRead(bq796x0_ADCGAIN2);
//  val1 &= 0b00001100; //There are some unknown reservred bits around val1 that need to be cleared
//
//  //Recombine the bits into one ADCGAIN
//  byte adcGain = (val1 << 1) | (val2 >> 5);
//
//  int gain = 365 + adcGain;
//
//  return(gain);
//}
//
////Returns the factory trimmed ADC offset
////Offset is -127 to 128 in mV
//int readADCoffset(void)
//{
//  //Here we need to convert a 8bit 2's compliment to a 16 bit int
//  char offset = registerRead(bq796x0_ADCOFFSET);
//
//  return((int)offset); //8 bit char is now a full 16-bit int. Easier math later on.
//}
//
////Returns the over voltage trip threshold
////Default is 0b.10.OVTRIP(0xAC).1000 = 0b.10.1010.1100.1000 = 0x2AC8 = 10,952
////OverVoltage = (OV_TRIP * GAIN) + ADCOFFSET
////Gain and Offset is different for each IC
////Example: voltage = (10,952 * 0.370) + 56mV = 4.108V
//float readOVtrip(void)
//{
//  int trip = registerRead(bq796x0_OV_TRIP);
//
//  trip <<= 4; //Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
//  trip |= 0x2008;
//
//  float overVoltage = ((float)trip * gain) + offset;
//  overVoltage /= 1000; //Convert to volts
//
//  //Serial1.print("overVoltage should be around 4.108: ");
//  //Serial1.println(overVoltage, 3);
//
//  return(overVoltage);
//}
//
////Given a voltage (4.22 for example), set the over voltage trip register
////Example: voltage = 4.2V = (4200mV - 56mV) / 0.370mv = 11,200
////11,200 = 0x2BC0 =
//void writeOVtrip(float tripVoltage)
//{
//  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
//  registerWrite(bq796x0_OV_TRIP, val); //address, value
//}
//
////Returns the under voltage trip threshold
////Default is 0b.01.UVTRIP(0x97).0000 = 0x1970 = 6,512
////UnderVoltage = (UV_TRIP * GAIN) + ADCOFFSET
////Gain and Offset is different for each IC
////Example: voltage = (6,512 * 0.370) + 56mV = 2.465V
//float readUVtrip(void)
//{
//  int trip = registerRead(bq796x0_UV_TRIP);
//
//  trip <<= 4; //Shuffle the bits to align to 0b.01.XXXX.XXXX.0000
//  trip |= 0x1000;
//
//  float underVoltage = ((float)trip * gain) + offset;
//  underVoltage /= 1000; //Convert to volts
//
//  //Serial1.print("underVoltage should be around 2.465: ");
//  //Serial1.println(underVoltage, 3);
//
//  return(underVoltage);
//}
//
////Given a voltage (2.85V for example), set the under voltage trip register
//void writeUVtrip(float tripVoltage)
//{
//  byte val = tripCalculator(tripVoltage); //Convert voltage to an 8-bit middle value
//  registerWrite(bq796x0_UV_TRIP, val); //address, value
//}
//
////Under voltage and over voltage use the same rules for calculating the 8-bit value
////Given a voltage this function uses gain and offset to get a 14 bit value
////Then strips that value down to the middle-ish 8-bits
////No registers are written, that's up to the caller
//byte tripCalculator(float tripVoltage)
//{
//  tripVoltage *= 1000; //Convert volts to mV
//
//  //Serial1.print("tripVoltage to be: ");
//  //Serial1.println(tripVoltage, 3);
//
//  tripVoltage -= offset;
//  tripVoltage /= gain;
//
//  int tripValue = (int)tripVoltage; //We only want the integer - drop decimal portion.
//
//  //Serial1.print("tripValue should be something like 0x2BC0: ");
//  //Serial1.println(tripValue, HEX);
//
//  tripValue >>= 4; //Cut off lower 4 bits
//  tripValue &= 0x00FF; //Cut off higher bits
//
//  //Serial1.print("About to report tripValue: ");
//  //Serial1.println(tripValue, HEX);
//
//  return(tripValue);
//}
//
//// ================= sundaya ==================//
//
//uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
//{
//  uint8_t i;
//  uint8_t data;
//  data = inCrc ^ inData;
//
//  for ( i = 0; i < 8; i++ )
//  {
//    if (( data & 0x80 ) != 0 )
//    {
//      data <<= 1;
//      data ^= 0x07;
//    }
//    else data <<= 1;
//  }
//
//  return data;
//}
//void sendx(byte address, int data)
//{
//  char buf[3];
//  buf[0] = (char) address;
//  buf[1] = data;
//  uint8_t crc = 0;
//  int I2CAddress =8;
//
//  // note that writes to the bq769x0 IC are: 1) start - 2) address - 3) address - 4) data - 5) CRC8 - 6) stop bit
//  Wire.beginTransmission(8); // writes start bit - the first step
//  Wire.write(buf[0]);                 // writes register address
//  Wire.write(buf[1]);                 // writes data - the fourth step
//
// boolean crcEnabled = true;
//  if (crcEnabled == true) {
//    // CRC is calculated over the slave address (including R/W bit), register address, and data.
//    crc = _crc8_ccitt_update(crc, (I2CAddress << 1) | 0);
//    crc = _crc8_ccitt_update(crc, buf[0]);
//    crc = _crc8_ccitt_update(crc, buf[1]);
//    buf[2] = crc;
//
//    Wire.write(buf[2]); // writes CRC
//  }
//
//  Wire.endTransmission();
//}
//
//
//
//// ================= sundaya ==================//
//
////Write a given value to a given register
//void registerWrite(byte regAddress, byte regData)
//{
//  Wire.beginTransmission(bqI2CAddress);
//  Wire.write(regAddress);
//  Wire.endTransmission();
//  byte error;
//   error = Wire.endTransmission();
//    Serial.println("gagal : " + String(error));
//
//  Wire.beginTransmission(bqI2CAddress);
//  Wire.write(regAddress);
//  Wire.write(regData);
//  Wire.endTransmission();
//  error;
//   error = Wire.endTransmission();
//   Serial.println("gagal : " + String(error));
//}
//
//
//
//
////Returns a given register
//byte registerRead(byte regAddress)
//{
//  Wire.beginTransmission(bqI2CAddress);
//  //Here's where I2C can time out
//  Wire.write(regAddress);
//  Wire.endTransmission();
//
//  Wire.requestFrom(bqI2CAddress, 1);
//
//  /*byte counter = 0;
//  while(Wire.available() == 0)
//  {
//    delay(1);
//    if(counter++ > 250) return(0); //Return with error
//  }*/
//
//  return(Wire.read());
//}
//
////Returns the atmoic int from two sequentials reads
//int registerDoubleRead(byte regAddress)
//{
//  Wire.beginTransmission(bqI2CAddress);
//  Wire.write(regAddress);
//  Wire.endTransmission();
//
//  Wire.requestFrom(bqI2CAddress, 2);
//
//  /*byte counter = 0;
//   while(Wire.available() < 2)
//   {
//   Serial1.print(".");
//   if(counter++ > MAX_I2C_TIME)
//   {
//   return(-1); //Time out error
//   }
//   delay(1);
//   }*/
//
//  byte reg1 = Wire.read();
//  byte reg2 = Wire.read();
//
//  //Serial1.print("reg1: 0x");
//  //Serial1.print(reg1, HEX);
//  //Serial1.print(" reg2: 0x");
//  //Serial1.println(reg2, HEX);
//
//  int combined = (int)reg1 << 8;
//  combined |= reg2;
//
//  return(combined);
//}
//
////Given a resistance on a super common 103AT-2 thermistor, return a temperature in C
////This is a poor way of converting the resistance to temp but it works for now
////From: http://www.rapidonline.com/pdf/61-0500e.pdf
//int thermistorLookup(float resistance)
//{
//  //Resistance is coming in as Ohms, this lookup table assume kOhm
//  resistance /= 1000; //Convert to kOhm
//
//  int temp = 0;
//
//  if(resistance > 329.5) temp = -50;
//  if(resistance > 247.7) temp = -45;
//  if(resistance > 188.5) temp = -40;
//  if(resistance > 144.1) temp = -35;
//  if(resistance > 111.3) temp = -30;
//  if(resistance > 86.43) temp = -25;
//  if(resistance > 67.77) temp = -20;
//  if(resistance > 53.41) temp = -15;
//  if(resistance > 42.47) temp = -10;
//  if(resistance > 33.90) temp = -5;
//  if(resistance > 27.28) temp = 0;
//  if(resistance > 22.05) temp = 5;
//  if(resistance > 17.96) temp = 10;
//  if(resistance > 14.69) temp = 15;
//  if(resistance > 12.09) temp = 20;
//  if(resistance > 10.00) temp = 25;
//  if(resistance > 8.313) temp = 30;
//
//  return(temp);
//}
