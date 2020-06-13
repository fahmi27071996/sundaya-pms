void sendCanbus()
{
    int packV = readPackVoltage() * 100;
    int pack[2];
    pack[0] = (25700 - packV) / 256 + 1;
    pack[1] = (25700 - packV) % 256;
    send_pack(124045412 - dock_number, 0x56, 0x61, pack[1], pack[0], 0x02, 0x64, 0x01, 0x64);
    send_pack(124045412 - dock_number - 65536, 0x53, 0x64, 0x64, 0x49, 0x49, 0x4b, 0x47, 0x45);
}

void readDataCell()
{
    int cellData[14];
    for (int x = 1; x <= 14; x++)
    {
        cellData[x - 1] = readCellVoltage(x, false);
        if (x == 14)
        {
            cellData[x - 1] = readCellVoltage(15, false);
        }
        Serial1.println("cell : " + String(x) + " " + String(cellData[x - 1]));

        getData(cellData[x - 1]);
    }
}

void getData(int cell)
{
    // readDataCell();
    int results[2];

    conversionData(results, cell);
    Serial1.println("============");

    Serial1.println(results[0]);
    Serial1.println(results[1]);

    Serial1.println("============");
}

void conversionData(int *cellData, int cellVoltage)
{
    int maxValue = 25700;

    cellData[1] = (maxValue - cellVoltage) / 256;
    cellData[0] = (maxValue - cellVoltage) % 256;
}

void dumpPack()
{
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

    int cellData[14];
    for (int x = 1; x <= 14; x++)
    {
        cellData[x - 1] = readCellVoltage(x, false);
        if (x == 14)
        {
            cellData[x - 1] = readCellVoltage(15, false);
        }
        Serial1.println("cell : " + String(x) + " " + String(cellData[x - 1]));

        // getData(cellData[x - 1]);
    }

    int cell1[2], cell2[2], cell3[2], cell4[2], cell5[2], cell6[2], cell7[2];
    int cell8[2], cell9[2], cell10[2], cell11[2], cell12[2], cell13[2], cell14[2];

    conversionData(cell1, cellData[0]);
    conversionData(cell2, cellData[1]);
    conversionData(cell3, cellData[2]);
    conversionData(cell4, cellData[3]);
    conversionData(cell5, cellData[4]);
    conversionData(cell6, cellData[5]);
    conversionData(cell7, cellData[6]);
    conversionData(cell8, cellData[7]);
    conversionData(cell9, cellData[8]);
    conversionData(cell10, cellData[9]);
    conversionData(cell11, cellData[10]);
    conversionData(cell12, cellData[11]);
    conversionData(cell13, cellData[12]);
    conversionData(cell14, cellData[13]);

    // for (int cell_converter = 0; cell_converter <= 14; cell_converter++)
    // {
    //     int base_cell = (25700 - cellVoltage[cell_converter]);
    //     array_byte1[cell_converter] = base_cell % 256;
    //     // array_byte2[cell_converter] = base_cell/256;
    // }

    // for (int cell_converter = 0; cell_converter <= 14; cell_converter++)
    // {
    //     int base_cell = (25700 - cellVoltage[cell_converter]);
    //     // array_byte1[cell_converter] = base_cell%256;
    //     array_byte2[cell_converter] = base_cell / 256;
    // }

    //  Serial1.println(String(cellVoltage[0]) + " aneh : " + String(array_byte1[0]) + " aneh1 : " + String (array_byte2[0]));

    // send_pack(124045412-dock_number-(65536*2),byte1,byte2,0x15,0x56,0x15,0x56,0x15,0x56);
    send_pack(124045412 - dock_number - (65536 * 2), cell1[0], cell1[1], cell2[0], cell2[1], cell3[0], cell3[1], cell4[0], cell4[1]);
    send_pack(124045412 - dock_number - (65536 * 3), cell5[0], cell5[1], cell6[0], cell6[1], cell7[0], cell7[1], cell8[0], cell8[1]);
    send_pack(124045412 - dock_number - (65536 * 4), cell9[0], cell9[1], cell10[0], cell10[1], cell11[0], cell11[1], cell12[0], cell12[1]);
    send_pack(124045412 - dock_number - (65536 * 5), cell13[0], cell13[1], cell14[0], cell14[1], 0x15, 0x56, 0x15, 0x56);

    send_pack(124045412 - dock_number - (65536 * 6), 0x15, 0x56, 0x17, 0x56, 0x49, 0x4b, 0x42, 0xfc);
    send_pack(124045412 - dock_number - (65536 * 7), 0x64, 0x63, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64);
    send_pack(124045412 - dock_number - (65536 * 8), 0x27, 0x5a, 0x64, 0x64, 0xe1, 0x6f, 0x61, 0x64);
    send_pack(124045412 - dock_number - (65536 * 9), 0x0e, 0x57, 0x64, 0x64, 0xf4, 0x59, 0x64, 0x64);
    send_pack(124045412 - dock_number - (65536 * 10), 0x52, 0x64, 0x59, 0x5f, 0x5a, 0x29, 0x64, 0x00);
}

void checkBusbar()
{
    if (digitalRead(busbar_short) == 0)
    {

        // digitalWrite(statLED, HIGH);
        dumpPack();
        ledSet();
    }
    else
    {
        digitalWrite(statLED, LOW);
    }
}

void ledSet()
{
    const long interval = 1000; 
    int ledState = LOW; 
     unsigned long currentMillis = millis();
      unsigned long previousMillis = 0;
      //millis() - lastTime
    if (millis() - previousMillis >= interval)
    {
        // save the last time you blinked the LED
        previousMillis = millis();

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW)
        {
            ledState = HIGH;
        }
        else
        {
            ledState = LOW;
        }

        // set the LED with the ledState of the variable:
        digitalWrite(statLED, ledState);
    }
}
