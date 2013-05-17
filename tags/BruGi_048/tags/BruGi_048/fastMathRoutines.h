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


/*******************************************/
/* Filter Utility Functions                */
/* Copyright © 2011, 2012  Bill Nesbitt    */
/*******************************************/

void utilFilterReset(struct utilFilter *f, float setpoint) {
    f->z1 = setpoint;
}

void utilFilterInit(struct utilFilter *f, float dt, float tau, float setpoint) {
    f->tc = dt / tau;
    utilFilterReset(f, setpoint);
}

inline float utilFilter(struct utilFilter *f, float signal) {
    register float z1;

    z1 = f->z1 + (signal - f->z1) * f->tc;
    f->z1 = z1;

    return z1;
}

int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}


