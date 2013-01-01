
void serial() {
   
  if (Serial.available() > 0) {
   
      byte SerialIn = Serial.read(); 
      
      switch (SerialIn) { 
          
        case 'S':
        
        Serial.print('*'); 
        Serial.print('S');
        Serial.write(constrain(rangle[0] + 90,0,255)); 
        Serial.write(constrain(rangle[1] + 90,0,255));
        Serial.write(constrain(rangle[2] + 90,0,255));  
        Serial.print('#');
        
        break; 
        
        case 'P': 
         
        kp1 = kp_pid1 * 100; 
        ki1 = ki_pid1 * 100; 
        kd1 = kd_pid1 * 100;
        
        kp2 = kp_pid2 * 100; 
        ki2 = ki_pid2 * 100; 
        kd2 = kd_pid2 * 100;
        
        Serial.print('*');
        Serial.print('P');
        Serial.write(kp1); 
        Serial.write(ki1);
        Serial.write(kd1);
        Serial.write(kp2); 
        Serial.write(ki2);
        Serial.write(kd2); 
        Serial.print('#'); 
       
        break; 
        
        case 'R':
         
        kp1r = Serial.read(); 
        ki1r = Serial.read(); 
        kd1r = Serial.read(); 
         
        kp2r = Serial.read(); 
        ki2r = Serial.read(); 
        kd2r = Serial.read(); 
        
        kp1rf = kp1r / 100; 
        ki1rf = ki1r / 100; 
        kd1rf = kd1r / 100; 
       
        kp2rf = kp2r / 100; 
        ki2rf = ki2r / 100; 
        kd2rf = kd2r / 100; 
        
        SetParameter_pid1(kp1r,ki1r,kd1r);
        SetParameter_pid2(kp2r,ki2r,kd2r); 
        eepromWrite(); 
        
        break; 
        
      } 
    
   } 
   
}
