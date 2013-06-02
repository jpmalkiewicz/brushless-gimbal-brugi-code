// Org ATAN2 ~200us, fastAtan2 ~128us, ultraFastAtan2 ~92us
// Fast arctan2
float ultraFastAtan2(float y, float x)
{
  float angle; 
  float coeff_1 = PI/4;
   float coeff_2 = 3*coeff_1;
   float abs_y = fabs(y)+1e-10 ;     // kludge to prevent 0/0 condition
   if (x>=0)
   {
      float r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
   }
   else
   {
      float r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
   return(-angle* (180.0f / PI));     // negate if in quad III or IV
   else
   return(angle* (180.0f / PI));
}

float fastAtan2(float y, float x) // in deg
{
  #define fp_is_neg(val) ((((byte*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100));
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10)
      z = z / (1.0f + 0.28f * z * z);
    if (fp_is_neg(x)) {
      if (y_neg) z -= PI;
      else z += PI;
    }
  } else {
    z = (PI / 2.0f) - z / (z * z + 0.28f);
    if (y_neg) z -= PI;}
  z *= (180.0f / PI);
  return z;
}

inline int16_t _atan2(float y, float x){
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 100); 
  return z;
}

int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}


inline int32_t constrain_int32(int32_t x , int32_t l, int32_t h) {
  if (x <= l) {
    return l;
  } else if (x >= h) {
    return h;
  } else {
    return x;
  }
}

//***************************************************************
// “Efficient approximations for the arctangent function”,
// Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
//***************************************************************
inline float Rajan_FastArcTan(float x) {
  return PI/4.0*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
}

// atan2 for all quadrants by A. Hahn
inline float Rajan_FastArcTan2(float y, float x) {

  uint8_t qCode;
  float pi_2 = PI/2.0;
  float q;
  float z;

  // 6 us
  bool swap45 = (fabs(y) > fabs(x));
    
  // 22us
  if ((y >= 0) && (x >= 0)) { qCode = 0; }
  if ((y >= 0) && (x <= 0)) { qCode = 1; }
  if ((y <= 0) && (x <= 0)) { qCode = 2; }
  if ((y <= 0) && (x >= 0)) { qCode = 3; }

  // 54 us
  if (swap45) {
    q = x / y;
  } else {
    q = y / x;
  }

  // 92 us
  z = Rajan_FastArcTan(q);

  if (swap45) {
    switch (qCode) {
      case 0: z = pi_2 - z;  break;
      case 1: z = pi_2 - z;  break;
      case 2: z = -pi_2 - z; break;
      case 3: z = -pi_2 - z; break;
    }
  } else {
    switch (qCode) {    
      case 0: z = z;         break;
      case 1: z = PI + z;    break;
      case 2: z = -PI + z;   break;
      case 3: z = z;         break;
    }
  }
  
  return z;
}

// atan2 returnig degrees * 100
int16_t Rajan_FastArcTan2_deg100(float y, float x) {
  return 180/PI * 100 * Rajan_FastArcTan2(y, x);
}


/************************/
/* LP Filter            */
/************************/
inline void utilLP_float(float * q, float i, float coeff) {
  *q += (i-*q)*coeff;
}



/************************/
/* Debugging            */
/************************/
// TODO: move functions to other file (utilities ?)

inline void stackCheck() {
  int localVar;  
  // stack limits
  if ((uint32_t)&localVar < stackTop) stackTop = (uint32_t)&localVar;
  if ((uint32_t)&localVar > stackBottom) stackBottom = (uint32_t)&localVar;
}

inline void heapCheck() {
  uint32_t * memPtr;
  // heap limits
  memPtr = (uint32_t *)malloc(8);
  if ((uint32_t)memPtr > heapTop) heapTop = (uint32_t)memPtr;
  if ((uint32_t)memPtr < heapBottom) heapBottom = (uint32_t)memPtr;
  free(memPtr);
}

inline void stackHeapEval(bool doPrint) {

  int diff = stackTop - heapTop;
  if (diff < 64) {
    Serial.println(F("WARNING: low memory < 64"));
  }
  if ((diff <64) || doPrint) {
    Serial.print(F("free memory = ")); Serial.println(diff);
    Serial.print(F("stackSize   = ")); Serial.println(stackBottom-stackTop);
    Serial.print(F("stackBottom = 0x")); Serial.println(stackBottom, HEX);
    Serial.print(F("stackTop    = 0x")); Serial.println(stackTop, HEX);
    Serial.print(F("heapSize    = ")); Serial.println(heapTop-heapBottom);
    Serial.print(F("heapTop     = 0x")); Serial.println(heapTop, HEX);
    Serial.print(F("heapBottom  = 0x")); Serial.println(heapBottom, HEX);
  }
}
