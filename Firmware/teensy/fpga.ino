
void setup_reg(Stream &FPGA) {
  FPGA.flush(); FPGA.setTimeout(1000);
  FPGA.read();FPGA.read();FPGA.read();
  delay(10);
  Serial.print(reg_read(FPGA, 0x02), HEX);
  while (reg_read(FPGA, 0x03) != 0x5678) {
    Serial.print("[Bad read]");
    delay(500);
    FPGA.read();
    FPGA.read();
    FPGA.read();
  }
  Serial.print(reg_read(FPGA, 0x03), HEX);
}


void reg_write(Stream &FPGA, uint8_t addr, uint16_t val){
  FPGA.flush();
  FPGA.write(0x80 | addr);        //send address to write (write bit set)
  FPGA.write((val >> 8) & 0xff);  //MSByte
  FPGA.write(val & 0xff);         //LSByte
  FPGA.flush();delay(1); // delay critical in Rev 2.0 for subsequent reads, unclear why
}


uint16_t reg_read(Stream &FPGA, uint8_t addr) {
  FPGA.flush();
  FPGA.write(0x7F & addr);  //send address to read (write bit clear)
  char bytes[2] = { 0 };
  FPGA.flush();
  FPGA.readBytes(bytes, 2);
  return (bytes[0] << 8) | bytes[1];
}

