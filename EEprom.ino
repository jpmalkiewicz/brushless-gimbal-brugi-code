void eepromRead() {
  
  kp1r = EEPROM.read(2); 
  ki1r = EEPROM.read(3);
  kd1r = EEPROM.read(4);
    
  kp2r = EEPROM.read(5); 
  ki2r = EEPROM.read(6);
  kd2r = EEPROM.read(7);

  kp1rf = kp1r / 100; 
  ki1rf = ki1r / 100; 
  kd1rf = kd1r / 100; 
       
  kp2rf = kp2r / 100; 
  ki2rf = ki2r / 100; 
  kd2rf = kd2r / 100; 
 
  SetParameter_pid1(kp1r,ki1r,kd1r);
  SetParameter_pid2(kp2r,ki2r,kd2r); 
       
}

void eepromWrite() {
   
  EEPROM.write(0,VersionStatus); 
  EEPROM.write(1,VersionNumber); 
  
  EEPROM.write(2, kp1r);
  EEPROM.write(3, ki1r);
  EEPROM.write(4, kd1r);
 
  EEPROM.write(5, kp2r);
  EEPROM.write(6, ki2r);
  EEPROM.write(7, kd2r);

}

void eepromClear() {
  for (int i = 0; i < 512; i++)
  EEPROM.write(i, 0); 
}
