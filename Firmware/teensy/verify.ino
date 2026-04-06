


// checks the atan2 calculations of the FPGA CORDIC engine
int verify_cordic_atan2(Stream &FPGA)
{
  // quick test of cordic
  Serial.print("\n");
  for(float theta=-0.5; theta<+0.49999; theta += 0.01)
  {
    Serial.print(theta,3); Serial.print(": "); 
     for(float amp=100; amp<32767; amp+= 2000)
     {
        int16_t test_y = roundf(sinf(2.0*3.141592653*theta)*amp);
        int16_t test_x = roundf(cosf(2.0*3.141592653*theta)*amp);
        reg_write(FPGA, 0x05, test_y);
        reg_write(FPGA, 0x06, test_x);
        delay(1);

        int16_t result = reg_read(FPGA, 0x05);

        //Serial.print(result, DEC); Serial.print(" ");
        Serial.print((float)result/65536.0,3); Serial.print(" ");
     }

     Serial.print("\n");
  }

 waitForKeyPress();
Serial.print("\n\n");

}