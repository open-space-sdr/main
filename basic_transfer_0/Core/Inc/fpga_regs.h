/*
 * fpga_regs.h
 *
 * fpga register addresses and bit definitions
 */

#ifndef INC_FPGA_REGS_H_
#define INC_FPGA_REGS_H_

// core and control
#define REG_ROLE_MODE        0x01   // Sets master/slave role and TDD mode
#define REG_CHAN_MASK        0x02   // Bitmask for active TX channels
#define REG_MAGIC            0x03   // Read-only handshake value (0x5678)
#define REG_RX_MUX           0x22   // Selects the active RX antenna path
#define REG_CTRL             0x23   // Main control: soft reset, force CW, etc.
#define REG_QEC_EN           0x25   // Enables and configures QEC correlator
#define REG_RSSI_SETPOINT    0x51   // Target for the AGC
#define REG_RX_SD_FEEDBACK   0x62   // Control for RX sigma-delta feedback
#define REG_FRAME_SRC        0x66   // Selects source for frame count
#define REG_TLM_MUX          0x4F   // Mux for reading different telemetry values

// modem metrics
#define REG_MM_INTERVAL      0x67   // Sets the measurement window duration
#define REG_MM_TRIGGER       0x68   // Rising edge starts measurement; reads MSB of result
#define REG_MM_MUX_LS        0x69   // Selects metric to read; reads LSB of result

// timing capture
#define REG_TCAP_ARM         0x50   // Write 1 to arm, poll until 0
#define REG_TCAP_TXTIME_LS   0x53   // tx_time_out_capture ls
#define REG_TCAP_TXTIME_MS   0x54   // tx_time_out_capture ms
#define REG_TCAP_RXSYM_IDX   0x55
#define REG_TCAP_RXFRAC      0x56
#define REG_TCAP_RXPER_LS    0x57
#define REG_TCAP_RXPER_MS    0x58
#define REG_TCAP_PHASE       0x59
#define REG_TCAP_TXERR_LS    0x5A
#define REG_TCAP_TXERR_MS    0x5B
#define REG_TCAP_TXPER_LS    0x5C
#define REG_TCAP_TXPER_MS    0x5D

// IQ capture ram regs
#define REG_CAP_ARM          0x4D   // Arms the IQ capture block
#define REG_IQ_IDX           0x40   // Sets the memory index for an IQ read
#define REG_I_RAM            0x4A   // Reads the I-channel sample from the selected index
#define REG_Q_RAM            0x4B   // Reads the Q-channel sample from the selected index

// SPI tunnel regs
#define REG_SPI_MAX2850      0x42   // Tunnel for writing to the MAX2850 (TX) chip
#define REG_SPI_MAX2851      0x43   // Tunnel for writing to the MAX2851 (RX) chip

// control reg bit definitions (for REG_CTRL 0x23)
#define CTRL_TX_DISABLE_LVDS 0x0001 // Disables sigma-delta driving LVDS
#define CTRL_MODEM_RESET     0x0002 // Puts the modem block in reset
#define CTRL_FORCE_CW_TONE   0x0008 // Forces a continuous-wave tone for testing

// modem metric mux Selectors (for REG_MM_MUX_LS 0x69)
#define MM_SEL_EVM_START     0
#define MM_SEL_EVM_STOP      1
#define MM_SEL_CHECK_START   2
#define MM_SEL_CHECK_STOP    3
#define MM_SEL_DETECT_START  4
#define MM_SEL_DETECT_STOP   5
#define MM_SEL_MEM_START     6
#define MM_SEL_MEM_STOP      7

// telemetry mux Selectors (for REG_TLM_MUX 0x4F)
#define TELEM_SEL_DC_IQ          1 // {dc_i[7:0], dc_q[7:0]}
#define TELEM_SEL_RSSI_D_IN      2 // rssi_d_in[8:0]
#define TELEM_SEL_RSSI_RESAMPLED 3 // rssi_resampled[8:0]
#define TELEM_SEL_EVM_LEAKY      4 // leaky_avg_evm[8:0]
#define TELEM_SEL_RX_PERIOD_HI   5 // rx_period[31:16]
#define TELEM_SEL_ADJUST_NEEDED  6 // adjust_needed_count[4:0]

#endif /* INC_FPGA_REGS_H_ */
