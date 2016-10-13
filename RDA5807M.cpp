/*
 * Library for the RDA5807M radio chip.
 *
 * @author Thomas Reusch, http://www.thomas-reusch.net
 * 
 */

#include <Wire.h>

/*
 * Registers (=> Datasheet)
 * Adresses:
 * 0x00 = Chip ID
 * 0x01 = N/A
 * 0x02 = general control (mono, bass, seek, rds, reset, ...)
 * 0x03 = channel control (channel, spacing, band, tune, ...)
 * 0x04 = deemphasis, auto frequency, ...
 * 0x05 = seek snr thresh, volume, ...
 * 0x06 = internal controls (open write mode)
 * 0x07 = softblend, 50MHz, direct freq, ...
 * 0x08 = direct frequency
 * 0x09 = N/A
 * 0x0A = RDS status, seek status, stereo status, 
 */

// register storage
uint16_t registers[16];

// i2c address for sequential read/write access = 0b00010000
#define RDA5807M_I2C_ADDR_SEQ  0x10

// i2c address for direct write access
#define RDA5807M_I2C_ADDR_DIR  0x11

/**
 * reset flag with (e.g.):
 *     registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_SPACING;    // clear bits
 *     registers[RDA5807M_REG_CHAN] |=  RDA5807M_REG_CHAN_SPACING_25; // set bits
 *
 * values marked with * are hardware defaults, we may set other defaults
 */

#define RDA5807M_REG_CHIPID           0x00
#define RDA5807M_REG_NOTUSED1         0x01

#define RDA5807M_REG_CTRL             0x02
#define RDA5807M_REG_CTRL_ENABLE      0x0001 // *0=disabled, 1=enabled
#define RDA5807M_REG_CTRL_RESET       0x0002 // *0=not_reset, 1=soft_reset
#define RDA5807M_REG_CTRL_NEWDEMOD    0x0004 // *0=old_demod, 1=new_demod
#define RDA5807M_REG_CTRL_RDS         0x0008 // *0=disabled, 1=enabled
#define RDA5807M_REG_CTRL_CLKMODE     0x0070 //mask
#define RDA5807M_REG_CTRL_CLKMODE_12  0x0010 // 12MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_13  0x0020 // 13MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_192 0x0030 // 19.2MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_24  0x0050 // 24MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_26  0x0060 // 26MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_327 0x0000 // 32.786MHz clock (default)
#define RDA5807M_REG_CTRL_CLKMODE_384 0x0070 // 38.4MHz clock
#define RDA5807M_REG_CTRL_SEEKMODE    0x0080 // *0=wrap, 1=stop
#define RDA5807M_REG_CTRL_SEEK        0x0100 // *0=disable, 1=enable
#define RDA5807M_REG_CTRL_SEEKUP      0x0200 // *0=seek_down, 1=seek_up
#define RDA5807M_REG_CTRL_RCLKDIRIN   0x0400 // *0=normal, 1=rclk_use_direct_input
#define RDA5807M_REG_CTRL_RCLKNC      0x0800 // *0=rclk_always, 1=rclk_not_always
#define RDA5807M_REG_CTRL_BASS        0x1000 // *0=disabled, 1=bass_boost
#define RDA5807M_REG_CTRL_MONO        0x2000 // *0=stereo, 1=force_mono
#define RDA5807M_REG_CTRL_DMUTE       0x4000 // *0=mute, 1=no_mute
#define RDA5807M_REG_CTRL_HIGHZD      0x8000 // *0=high_impedance, 1=normal

#define RDA5807M_REG_CHAN             0x03
#define RDA5807M_REG_CHAN_SPACING     0x0003 //mask
#define RDA5807M_REG_CHAN_SPACING_25  0x0003 // 25kHz channel spacing
#define RDA5807M_REG_CHAN_SPACING_50  0x0002 // 50kHz channel spacing
#define RDA5807M_REG_CHAN_SPACING_100 0x0000 // 100kHz channel spacing (default)
#define RDA5807M_REG_CHAN_SPACING_200 0x0001 // 200kHz channel spacing
#define RDA5807M_REG_CHAN_BAND        0x000C //mask
#define RDA5807M_REG_CHAN_BAND_65_76  0x000C // East Europe FM band
#define RDA5807M_REG_CHAN_BAND_76_91  0x0004 // Japan FM band
#define RDA5807M_REG_CHAN_BAND_76_108 0x0008 // worldwide FM band
#define RDA5807M_REG_CHAN_BAND_87_108 0x0000 // US/Europe band (default)
#define RDA5807M_REG_CHAN_TUNE        0x0010 // *0=disable, 1=enable
#define RDA5807M_REG_CHAN_DIRECT      0x0020 // directly set frequency
#define RDA5807M_REG_CHAN_SELECT      0xFFC0 //mask, default=0x0000

#define RDA5807M_REG_INTL1            0x04
#define RDA5807M_REG_INTL1_AFCD       0x0100 // *0=disabled, 1=enabled
#define RDA5807M_REG_INTL1_SOFTMUTE   0x0200 // 0=disabled, *1=enabled
#define RDA5807M_REG_INTL1_RSVD1      0x0400 //don't care
#define RDA5807M_REG_INTL1_DEEMPH     0x0800 // *0=75µs, 1=50µs
#define RDA5807M_REG_INTL1_RSVD2      0x3000 //don't care
#define RDA5807M_REG_INTL1_RSVD3      0x8000 //don't care

#define RDA5807M_REG_SNDINT           0x05
#define RDA5807M_REG_SNDINT_VOL       0x000F //mask, default=0x000F(max), scale=log
#define RDA5807M_REG_SNDINT_RSVD1     0x0030 //don't care
#define RDA5807M_REG_SNDINT_SEEKTH    0x0F00 //mask, default=0x0800 ~ 32dB SNR
#define RDA5807M_REG_SNDINT_RSVD2     0x7000 //don't care
#define RDA5807M_REG_SNDINT_INTMODE   0x8000 // 0=5ms, *1=int_until_read_reg0Ch

#define RDA5807M_REG_INTL2            0x06
#define RDA5807M_REG_INTL2_OPENMODE   0x6000 // *00=reg_ro_ro, 11=reg_ro_rw
#define RDA5807M_REG_INTL2_RSVD       0x8000 // don't care

#define RDA5807M_REG_FRSB             0x07
#define RDA5807M_REG_FRSB_FREQMODE    0x0001 // *0=channeled, 1=direct
#define RDA5807M_REG_FRSB_SOFTBLEN    0x0002 // 0=disabled, *1=enabled
#define RDA5807M_REG_FRSB_SEEKTHOLD   0x00FC //mask, default=0x0000, see datasheet
#define RDA5807M_REG_FRSB_RSVD1       0x0100 //don't care
#define RDA5807M_REG_FRSB_50M_65M     0x0200 // lowest freq. 0=50MHz, *1=65MHz
#define RDA5807M_REG_FRSB_SBLENDTHR   0x7C00 //mask, default=0x4000, unit=2dB

#define RDA5807M_REG_FREQDIRECT       0x08
#define RDA5807M_REG_FREQDIRECT_FREQ  0x00FF // frequency direct offset



/**
 * Constructor: Do nothing
 */
RDA5807M::RDA5807M() {}

/**
 * initialize
 */
bool RDA5807M::begin() {
  Wire.begin();
  Wire.beginTransmission(RDA5807M_I2C_ADDR_SEQ);
  int ret = Wire.endTransmission();
  if (ret == 0) {

    // initialize all registers
    for(int i=0; i<16; i++) {
      registers[i] = 0x0000;
    }
    
    // enable, new demodulation method, disable mute, disable high impedance
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_ENABLE;
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_RESET;
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_NEWDEMOD;
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_DMUTE;
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_HIGHZD;
    
    // seek threshold 32dB SNR (hardware default)
    // for good measure bitwise and with mask
    // + interrupt-mode 1
    registers[RDA5807M_REG_SNDINT] |= RDA5807M_REG_SNDINT_SEEKTH & 0x0800;
    registers[RDA5807M_REG_SNDINT] |= RDA5807M_REG_SNDINT_INTMODE;
    
    // 50µs deemphasis
    registers[RDA5807M_REG_INTL1] |= RDA5807M_REG_INTL1_DEEMPH;

    // softblend, min. freq 65MHz, default softblend threshold (32dB)
    registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_SOFTBLEN;
    registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_50M_65M;
    registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_SBLENDTHR & 0x0400;
    
    writeRegisters();
    return(true);
  }
  return(false); // no RDA i2c device found
}

/**
 * (over)write all registers
 */
bool RDA5807M::writeRegisters() {
  Wire.beginTransmission(RDA5807M_I2C_ADDR_SEQ);
  for(int i=2; i<7; i++) { // writes start at register 0x02
    Wire.write(registers[i] >> 8);
    Wire.write(registers[i] & 0xFF);
  }
  Wire.endTransmission();
}


bool RDA5807M::writeRegister(int reg) {
  Wire.beginTransmission(RDA5807M_I2C_ADDR_DIR);
  Wire.write(reg);
  Wire.write(registers[reg] >> 8);
  Wire.write(registers[reg] & 0xFF);
  Wire.endTransmission();
}

bool RDA5807M::setEnable(bool e) {
  if(e) {
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_ENABLE;
  }
  else {
    registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_ENABLE;
  }
}

bool RDA5807M::setNewModulatorEnabled(bool e) {
  if(e) {
    registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_NEWDEMOD;
  }
  else {
    registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_NEWDEMOD;
  }
}

void RDA5807M::setChannelSpacing(uint16_t channelSpacing) {
  registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_SPACING; // reset
  registers[RDA5807M_REG_CHAN] |= channelSpacing;          // set new
  writeRegister(RDA5807M_REG_CHAN);
}


