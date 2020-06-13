// void readCurrent(){
//     int adcRaw = 0;
//     byte sysStat = registerRead(bq796x0_SYS_STAT);
//     adcRaw = (registerRead(CC_HI_BYTE) << 8) | registerRead(CC_LO_BYTE);
//     int pack_current_mA = (int16_t) adcRaw * 8.44 ;// conf->shunt_res_mOhm; 

//     Serial1.println("ADC : " + String(adcRaw));
//     Serial1.println("current : " + String(pack_current_mA));
//     // registerWrite(SYS_STAT, 0b10000000);
//     //registerWrite(bq796x0_SYS_STAT, sysNew);
//     registerWrite(SYS_STAT, 0b10000000);
// }