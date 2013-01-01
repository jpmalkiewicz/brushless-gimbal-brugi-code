void initBlController() {

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); 
  TCCR0B = _BV(CS00);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5 

}

void makestep_motorone(int sign_one, int step_one) {
  
  for(int i=0;i<step_one;i++) {
  
  if(direction_one * sign_one < 0 ) {
    
  direction_a_one = direction_a_one * -1;  
  direction_b_one = direction_b_one * -1;  
  direction_c_one = direction_c_one * -1;  
  direction_one = direction_one * -1;
  
  }

  if(position_a_one + direction_a_one > 255 | position_a_one + direction_a_one < 0) direction_a_one = direction_a_one * -1;
  if(position_b_one + direction_b_one > 255 | position_b_one + direction_b_one < 0) direction_b_one = direction_b_one * -1;
  if(position_c_one + direction_c_one > 255 | position_c_one + direction_b_one < 0) direction_c_one = direction_c_one * -1;

  position_a_one = position_a_one + direction_a_one;
  position_b_one = position_b_one + direction_b_one;
  position_c_one = position_c_one + direction_c_one;
  
  OCR0A = SinusValues[position_a_one];
  OCR0B = SinusValues[position_b_one];
  OCR2B = SinusValues[position_c_one];
  
  }  
}

void makestep_motortwo(int sign_two, int step_two) {
  
  for(int i=0;i<step_two;i++) {
  
  if(direction_two * sign_two < 0 ) {
    
  direction_a_two = direction_a_two * -1;  
  direction_b_two = direction_b_two * -1;  
  direction_c_two = direction_c_two * -1;  
  direction_two = direction_two * -1;
  
  }

  if(position_a_two + direction_a_two > 255 || position_a_two + direction_a_two < 0) direction_a_two = direction_a_two * -1;
  if(position_b_two + direction_b_two > 255 || position_b_two + direction_b_two < 0) direction_b_two = direction_b_two * -1;
  if(position_c_two + direction_c_two > 255 || position_c_two + direction_b_two < 0) direction_c_two = direction_c_two * -1;

  position_a_two = position_a_two + direction_a_two;
  position_b_two = position_b_two + direction_b_two;
  position_c_two = position_c_two + direction_c_two;
  
  OCR2A = SinusValues[position_a_two];
  OCR1B = SinusValues[position_b_two];
  OCR1A = SinusValues[position_c_two];
  
  }  
}

