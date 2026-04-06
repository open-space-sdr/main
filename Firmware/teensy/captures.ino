void sdm_capture(Stream &FPGA){
  Serial.print("\nSDM RAM capture \n");

    //reg_write(FPGA_RX, 0x3F, 2); //save choice
    reg_write(FPGA, 0x70, 32767);  //Set index

    Serial.print(reg_read(FPGA, 0x70), DEC);
    Serial.print("\n");  // quick read back

    delay(200);  //read should take ~15ms

    for (int k = 32767; k; k--) {
      reg_write(FPGA, 0x70, 0x8000 | k);
      Serial.print(reg_read(FPGA, 0x71), HEX);
      Serial.print(" ");
    }
    Serial.print("\n"); 
  
  return;
}

void rx_debug_capture(Stream &FPGA){
  Serial.println("RX DEBUG CAPTURE:");
    // Arm capture

    reg_write(FPGA, 0x4D, 0);  // reset the capture
    delay(10);
    Serial.print(reg_read(FPGA, 0x4D), HEX);
    Serial.print(" ");
    reg_write(FPGA, 0x4F, 7);  // MUX SELECTION for debug capture 2
    delay(1);
    reg_write(FPGA, 0x4D, 1);  // activate the capture

    
    delay(10);
reg_write(FPGA, 0x41, 1); reg_write(FPGA, 0x41, 0); // force trigger


    delay(100);
    Serial.print(reg_read(FPGA, 0x4D), HEX);
    Serial.print("\n");

    for (int k = 0; k < 8191; k++) {
      reg_write(FPGA, 0x40, k);
      Serial.print(reg_read(FPGA, 0x4A), HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
    for (int k = 0; k < 8191; k++) {
      reg_write(FPGA, 0x40, k);
      Serial.print(reg_read(FPGA, 0x4B), HEX);
      Serial.print(" ");
    }
  return;
}
