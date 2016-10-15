/*
 * Library for the RDA5807M radio chip.
 *
 * @author Thomas Reusch, http://www.thomas-reusch.net
 * 
 */

#include <Wire.h>
#include <RDA5807M.h>

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
 
/**
 * v~ defines for internal use ~v
 */

#define RDA5807M_REG_CHIPID           0x00
#define RDA5807M_REG_NOTUSED1         0x01

#define RDA5807M_REG_CTRL             0x02
#define RDA5807M_REG_CTRL_ENABLE      0x0001 // *0=disabled, 1=enabled
#define RDA5807M_REG_CTRL_RESET       0x0002 // *0=not_reset, 1=soft_reset
#define RDA5807M_REG_CTRL_NEWDEMOD    0x0004 // *0=old_demod, 1=new_demod
#define RDA5807M_REG_CTRL_RDS         0x0008 // *0=disabled, 1=enabled
#define RDA5807M_REG_CTRL_CLKMODE     0x0070 //mask, default=0x0000 (32.786MHz)
                                             // => values defined in header file
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
#define RDA5807M_REG_CHAN_SPACING     0x0003 //mask, default=0x0000 (100kHz)
#define RDA5807M_REG_CHAN_BAND        0x000C //mask, default=0x0000 (US/EUR)
#define RDA5807M_REG_CHAN_TUNE        0x0010 // *0=disable, 1=enable
#define RDA5807M_REG_CHAN_DIRECT      0x0020 // directly set frequency
#define RDA5807M_REG_CHAN_SELECT      0xFFC0 //mask, default=0x0000

#define RDA5807M_REG_INTL1            0x04
#define RDA5807M_REG_INTL1_AFCD       0x0100 // *0=AFC enabled, 1=AFC disabled
#define RDA5807M_REG_INTL1_SOFTMUTE   0x0200 // 0=disabled, *1=enabled
#define RDA5807M_REG_INTL1_RSVD1      0x0400 //don't care
#define RDA5807M_REG_INTL1_DEEMPH     0x0800 // *0=75µs, 1=50µs
#define RDA5807M_REG_INTL1_RSVD2      0x3000 //don't care
#define RDA5807M_REG_INTL1_RSVD3      0x8000 //don't care

#define RDA5807M_REG_SNDINT           0x05
#define RDA5807M_REG_SNDINT_VOL       0x000F //mask, default=0x000F (max), log
#define RDA5807M_REG_SNDINT_RSVD1     0x0030 //don't care
#define RDA5807M_REG_SNDINT_SEEKTH    0x0F00 //mask, default=0x0800 ~ 32dB SNR
#define RDA5807M_REG_SNDINT_RSVD2     0x7000 //don't care
#define RDA5807M_REG_SNDINT_INTMODE   0x8000 // 0=5ms, *1=int_until_read_reg0Ch

#define RDA5807M_REG_INTL2            0x06
#define RDA5807M_REG_INTL2_OPENMODE   0x6000 // *00=reg_ro_ro, 11=reg_ro_rw
#define RDA5807M_REG_INTL2_RSVD       0x8000 // don't care

#define RDA5807M_REG_FRSB             0x07
#define RDA5807M_REG_FRSB_FREQMODE    0x0001 // *0=channeled, 1=direct
#define RDA5807M_REG_FRSB_SOFTBLEND   0x0002 // 0=disabled, *1=enabled
#define RDA5807M_REG_FRSB_SEEKTHOLD   0x00FC //mask, default=0x0000, see datas.
#define RDA5807M_REG_FRSB_RSVD1       0x0100 //don't care
#define RDA5807M_REG_FRSB_50M_65M     0x0200 // lowest freq. 0=50MHz, *1=65MHz
#define RDA5807M_REG_FRSB_SBLENDTHR   0x7C00 //mask, default=0x4000, unit=2dB

#define RDA5807M_REG_FREQDIRECT       0x08
#define RDA5807M_REG_FREQDIRECT_FREQ  0xFFFF // frequency direct offset
                                             // => (freq_min + freq_direct) kHz
                                             // (freq_min = 76000/87000)

#define RDA5807M_REG_STATUS1          0x0A
#define RDA5807M_REG_STATUS1_CHAN     0x03FF //mask, channel
#define RDA5807M_REG_STATUS1_STEREO   0x0400 // 0=mono, 1=stereo
#define RDA5807M_REG_STATUS1_BLKE     0x0800 // block E: 0=not found, 1=found
#define RDA5807M_REG_STATUS1_RDSSYNC  0x1000 // RDS decoder sync 0=no, 1=yes
#define RDA5807M_REG_STATUS1_SEEKFAIL 0x2000 // 0=seek successful, 1=seek failed
#define RDA5807M_REG_STATUS1_STCOMPL  0x4000 // seek/tune 0=not complete, 1=complete
#define RDA5807M_REG_STATUS1_RDSREADY 0x8000 // R(B)DS group 0=not ready, 1=ready
#define RDA5807M_REG_STATUS2          0x0B
#define RDA5807M_REG_STATUS2_BLERB    0x0003
#define RDA5807M_REG_STATUS2_BLERA    0x000C
#define RDA5807M_REG_STATUS2_ABCDE    0x0010 // 0=0C-0F=ABCD (RDS), 1=0C-0F=E (RBDS)
#define RDA5807M_REG_STATUS2_RSVD     0x0060
#define RDA5807M_REG_STATUS2_FMREADY  0x0080
#define RDA5807M_REG_STATUS2_FMTRUE   0x0100
#define RDA5807M_REG_STATUS2_RSSI     0xFE00
#define RDA5807M_REG_RDSA             0x0C   // 0xFFFF = Data
#define RDA5807M_REG_RDSB             0x0D   // 0xFFFF = Data
#define RDA5807M_REG_RDSC             0x0E   // 0xFFFF = Data
#define RDA5807M_REG_RDSD             0x0F   // 0xFFFF = Data

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
    
    registers[RDA5807M_REG_CHAN] |= RDA5807M_REG_CHAN_TUNE;
    
    // seek threshold 32dB SNR (hardware default)
    // for good measure bitwise and with mask
    // + interrupt-mode 1
    registers[RDA5807M_REG_SNDINT] |= RDA5807M_REG_SNDINT_SEEKTH & 0x0800;
    registers[RDA5807M_REG_SNDINT] |= RDA5807M_REG_SNDINT_INTMODE;
    // ???? 8th bit is undocumented but must be set?!
    registers[RDA5807M_REG_SNDINT] |= 0x0080;
        
    // 50µs deemphasis
//    registers[RDA5807M_REG_INTL1] |= RDA5807M_REG_INTL1_DEEMPH;

    // softblend, min. freq 65MHz, default softblend threshold (32dB)
    registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_SOFTBLEND;
    registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_50M_65M;
    registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_SBLENDTHR & 0x0200;

    writeRegisters();
    registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_RESET;
    return(true);
  }
  return(false); // no RDA i2c device found
}

/**
 * read all registers
 */
void RDA5807M::readRegisters() {
  Wire.requestFrom (RDA5807M_I2C_ADDR_SEQ, 12); // registers 0Ah-0Fh (12 bytes)
  uint8_t reg;
  for(int i=0; i<6; i++) {
    registers[0xA+i] = 0x0000;
    reg = Wire.read();
    registers[0xA+i] = reg << 8;
    reg = Wire.read();
    registers[0xA+i] |= reg;
  }
  Wire.endTransmission();
}

/**
 * (over)write all registers
 */
void RDA5807M::writeRegisters() {
  Wire.beginTransmission(RDA5807M_I2C_ADDR_SEQ);
  for(int i=2; i<9; i++) { // writes start at register 0x02
    Wire.write(registers[i] >> 8);
    Wire.write(registers[i] & 0xFF);
  }
  Wire.endTransmission();
}

/**
 * write one register directly
 */
void RDA5807M::writeRegister(int reg) {
  Wire.beginTransmission(RDA5807M_I2C_ADDR_DIR);
  Wire.write(reg);
  Wire.write(registers[reg] >> 8);
  Wire.write(registers[reg] & 0xFF);
  Wire.endTransmission();
}

// set/get chip enabled bit
void RDA5807M::setChipEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_ENABLE;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_ENABLE;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getChipEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_ENABLE) > 0;
}

// set/get new modulator enabled bit
void RDA5807M::setNewModulatorEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_NEWDEMOD;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_NEWDEMOD;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getNewModulatorEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_NEWDEMOD) > 0;
}

// set/get RDS enabled bit
void RDA5807M::setRDSEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_RDS;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_RDS;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getRDSEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_RDS) > 0;
}

// set/get clock frequency
void RDA5807M::setClockFrequency(uint16_t val) {
  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_CLKMODE;
  registers[RDA5807M_REG_CTRL] |= val & RDA5807M_REG_CTRL_CLKMODE;
  writeRegister(RDA5807M_REG_CTRL);
}
uint16_t RDA5807M::getClockFrequency() {
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_CLKMODE);
}

// set/get seek wrap
void RDA5807M::setSeekWrapEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_SEEKMODE;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_SEEKMODE;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getSeekWrapEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_SEEKMODE) > 0;
}

// start seek
void RDA5807M::startSeek() {
  registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_SEEK;  // set seek bit
  writeRegister(RDA5807M_REG_CTRL);                        // seek
  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_SEEK; // unset seek bit
}

// set/get seek direction up
void RDA5807M::setSeekDirectionUpEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_SEEKUP;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_SEEKUP;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getSeekDirectionUpEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_SEEKUP) > 0;
}


// set/get reference clock direct input enabled
void RDA5807M::setRCKLDirectInputEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_RCLKDIRIN;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_RCLKDIRIN;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getRCKLDirectInputEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_RCLKDIRIN) > 0;
}

// set/get remote clock not calibrated
void RDA5807M::setRCLKNotCalibrated(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_RCLKNC;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_RCLKNC;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getRCLKNotCalibrated(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_RCLKNC) > 0;
}

// set/get bass boost enabled
void RDA5807M::setBassBoostEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_BASS;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_BASS;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getBassBoostEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_BASS) > 0;
}

// set/get mono enabled
void RDA5807M::setMonoEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_MONO;
  else  registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_MONO;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getMonoEnabled(){
  return (registers[RDA5807M_REG_CTRL] & RDA5807M_REG_CTRL_MONO) > 0;
}

// set/get mute enabled
void RDA5807M::setMuteEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_DMUTE;
  else  registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_DMUTE;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getMuteEnabled(){
  return (~registers[RDA5807M_REG_CTRL] & ~RDA5807M_REG_CTRL_DMUTE) > 0;
}

// set/get output high impedance enabled
void RDA5807M::setOutputHighZEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CTRL] &= ~RDA5807M_REG_CTRL_HIGHZD;
  else  registers[RDA5807M_REG_CTRL] |= RDA5807M_REG_CTRL_HIGHZD;
  writeRegister(RDA5807M_REG_CTRL);
}
bool RDA5807M::getOutputHighZEnabled(){
  return (~registers[RDA5807M_REG_CTRL] & ~RDA5807M_REG_CTRL_HIGHZD) > 0;
}

// set/get channel spacing
void RDA5807M::setChannelSpacing(uint16_t val) {
  registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_SPACING;
  registers[RDA5807M_REG_CHAN] |= val & RDA5807M_REG_CHAN_SPACING;
  writeRegister(RDA5807M_REG_CHAN);
}
uint16_t RDA5807M::getChannelSpacing() {
  return (registers[RDA5807M_REG_CHAN] & RDA5807M_REG_CHAN_SPACING);
}

// set/get radio band
void RDA5807M::setRadioBand(uint16_t val) {
  registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_BAND;
  registers[RDA5807M_REG_CHAN] |= val & RDA5807M_REG_CHAN_BAND;
  writeRegister(RDA5807M_REG_CHAN);
}
uint16_t RDA5807M::getRadioBand() {
  return (registers[RDA5807M_REG_CHAN] & RDA5807M_REG_CHAN_BAND);
}

// set/get tune enabled
void RDA5807M::setTuneEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CHAN] |= RDA5807M_REG_CHAN_TUNE;
  else  registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_TUNE;
  writeRegister(RDA5807M_REG_CHAN);
}
bool RDA5807M::getTuneEnabled(){
  return (registers[RDA5807M_REG_CHAN] & RDA5807M_REG_CHAN_TUNE) > 0;
}

// set/get direct control enabled
void RDA5807M::setDirectControlEnabled(bool e) {
  if(e) registers[RDA5807M_REG_CHAN] |= RDA5807M_REG_CHAN_DIRECT;
  else  registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_DIRECT;
  writeRegister(RDA5807M_REG_CHAN);
}
bool RDA5807M::getDirectControlEnabled(){
  return (registers[RDA5807M_REG_CHAN] & RDA5807M_REG_CHAN_DIRECT) > 0;
}

// set/get channel
void RDA5807M::setChannel(uint16_t val) {
  registers[RDA5807M_REG_CHAN] &= ~RDA5807M_REG_CHAN_SELECT;
  registers[RDA5807M_REG_CHAN] |= (val << 6) & RDA5807M_REG_CHAN_SELECT;
  writeRegister(RDA5807M_REG_CHAN);
}
uint16_t RDA5807M::getChannel() {
  return ((registers[RDA5807M_REG_CHAN] & RDA5807M_REG_CHAN_SELECT) >> 6);
}

// set/get auto frequency correction enabled
void RDA5807M::setAFCEnabled(bool e) {
  if(e) registers[RDA5807M_REG_INTL1] &= ~RDA5807M_REG_INTL1_AFCD;
  else  registers[RDA5807M_REG_INTL1] |= RDA5807M_REG_INTL1_AFCD;
  writeRegister(RDA5807M_REG_INTL1);
}
bool RDA5807M::getAFCEnabled(){
  return (registers[RDA5807M_REG_INTL1] & RDA5807M_REG_INTL1_AFCD) > 0;
}

// set/get softmute enabled
void RDA5807M::setSoftmuteEnabled(bool e) {
  if(e) registers[RDA5807M_REG_INTL1] |= RDA5807M_REG_INTL1_SOFTMUTE;
  else  registers[RDA5807M_REG_INTL1] &= ~RDA5807M_REG_INTL1_SOFTMUTE;
  writeRegister(RDA5807M_REG_INTL1);
}
bool RDA5807M::getSoftmuteEnabled(){
  return (registers[RDA5807M_REG_INTL1] & RDA5807M_REG_INTL1_SOFTMUTE) > 0;
}

// set/get deemphasis to 50µs instead of 75µs
void RDA5807M::setDeemphasisFiftyEnabled(bool e) {
  if(e) registers[RDA5807M_REG_INTL1] |= RDA5807M_REG_INTL1_DEEMPH;
  else  registers[RDA5807M_REG_INTL1] &= ~RDA5807M_REG_INTL1_DEEMPH;
  writeRegister(RDA5807M_REG_INTL1);
}
bool RDA5807M::getDeemphasisFiftyEnabled(){
  return (registers[RDA5807M_REG_INTL1] & RDA5807M_REG_INTL1_DEEMPH) > 0;
}

// set/get volume level
void RDA5807M::setVolume(uint8_t val) {
  registers[RDA5807M_REG_SNDINT] &= ~RDA5807M_REG_SNDINT_VOL;
  registers[RDA5807M_REG_SNDINT] |= val & RDA5807M_REG_SNDINT_VOL;
  writeRegister(RDA5807M_REG_SNDINT);
}
uint8_t RDA5807M::getVolume() {
  return (registers[RDA5807M_REG_SNDINT] & RDA5807M_REG_SNDINT_VOL) & 0xFF;
}

// set/get seek threshold
void RDA5807M::setSeekThreshold(uint8_t val) {
  registers[RDA5807M_REG_SNDINT] &= ~RDA5807M_REG_SNDINT_SEEKTH;
  registers[RDA5807M_REG_SNDINT] |= (val << 8) & RDA5807M_REG_SNDINT_SEEKTH;
  writeRegister(RDA5807M_REG_SNDINT);
}
uint8_t RDA5807M::getSeekThreshold() {
  return ((registers[RDA5807M_REG_SNDINT] & RDA5807M_REG_SNDINT_SEEKTH) >> 8) & 0xFF;
}


// set/get interrupt after read register 0Ch enabled
void RDA5807M::setIntAfterReg0CReadEnabled(bool e) {
  if(e) registers[RDA5807M_REG_SNDINT] |= RDA5807M_REG_SNDINT_INTMODE;
  else  registers[RDA5807M_REG_SNDINT] &= ~RDA5807M_REG_SNDINT_INTMODE;
  writeRegister(RDA5807M_REG_SNDINT);
}
bool RDA5807M::getIntAfterReg0CReadEnabled(){
  return (registers[RDA5807M_REG_SNDINT] & RDA5807M_REG_SNDINT_INTMODE) > 0;
}

// set/get read-only register write enabled
void RDA5807M::setOpenModeEnabled(bool e) {
  if(e) registers[RDA5807M_REG_INTL2] |= RDA5807M_REG_INTL2_OPENMODE;
  else  registers[RDA5807M_REG_INTL2] &= ~RDA5807M_REG_INTL2_OPENMODE;
  writeRegister(RDA5807M_REG_INTL2);
}
bool RDA5807M::getOpenModeEnabled(){
  return (registers[RDA5807M_REG_SNDINT] & RDA5807M_REG_SNDINT_INTMODE) > 0;
}

// set/get direct frequency set mode (instead of channeled)
void RDA5807M::setFrequencyOffsetDirectEnabled(bool e) {
  if(e) registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_FREQMODE;
  else  registers[RDA5807M_REG_FRSB] &= ~RDA5807M_REG_FRSB_FREQMODE;
  writeRegister(RDA5807M_REG_FRSB);
}
bool RDA5807M::getFrequencyOffsetDirectEnabled(){
  return (registers[RDA5807M_REG_FRSB] & RDA5807M_REG_FRSB_FREQMODE) > 0;
}

// set/get softblend enabled
void RDA5807M::setSoftblendEnabled(bool e) {
  if(e) registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_SOFTBLEND;
  else  registers[RDA5807M_REG_FRSB] &= ~RDA5807M_REG_FRSB_SOFTBLEND;
  writeRegister(RDA5807M_REG_FRSB);
}
bool RDA5807M::getSoftblendEnabled(){
  return (registers[RDA5807M_REG_FRSB] & RDA5807M_REG_FRSB_SOFTBLEND) > 0;
}

// set/get seek threshold for old seek method
void RDA5807M::setOldSeekThresholdEnabled(uint8_t val) {
  registers[RDA5807M_REG_FRSB] &= ~RDA5807M_REG_FRSB_SEEKTHOLD;
  registers[RDA5807M_REG_FRSB] |= (val << 2) & RDA5807M_REG_FRSB_SEEKTHOLD;
  writeRegister(RDA5807M_REG_FRSB);
}
uint8_t RDA5807M::getOldSeekThresholdEnabled() {
  return ((registers[RDA5807M_REG_FRSB] & RDA5807M_REG_FRSB_SEEKTHOLD) >> 2) & 0xFF;
}

// set/get frequency range for eastern europe band to 50MHz~76MHz
void RDA5807M::setLowFreqencyFiftyEnabled(bool e) {
  if(e) registers[RDA5807M_REG_FRSB] &= ~RDA5807M_REG_FRSB_50M_65M;
  else  registers[RDA5807M_REG_FRSB] |= RDA5807M_REG_FRSB_50M_65M;
  writeRegister(RDA5807M_REG_FRSB);
}
bool RDA5807M::getLowFreqencyFiftyEnabled(){
  return (registers[RDA5807M_REG_FRSB] & RDA5807M_REG_FRSB_50M_65M) > 0;
}

// set/get softblend threshold (unit=2dB)
void RDA5807M::setSoftblendThreshold(uint8_t val) {
  registers[RDA5807M_REG_FRSB] &= ~RDA5807M_REG_FRSB_SBLENDTHR;
  registers[RDA5807M_REG_FRSB] |= (val << 10) & RDA5807M_REG_FRSB_SBLENDTHR;
  writeRegister(RDA5807M_REG_FRSB);
}
uint8_t RDA5807M::getSoftblendThreshold() {
  return ((registers[RDA5807M_REG_FRSB] & RDA5807M_REG_FRSB_SBLENDTHR) >> 10) & 0xFF;
}

// set/get frequency offset value directly
void RDA5807M::setFrequencyOffsetDirect(uint16_t val) {
  registers[RDA5807M_REG_FREQDIRECT] &= ~RDA5807M_REG_FREQDIRECT_FREQ;
  registers[RDA5807M_REG_FREQDIRECT] |= val & RDA5807M_REG_FREQDIRECT_FREQ;
  writeRegister(RDA5807M_REG_FREQDIRECT);
}
uint8_t RDA5807M::getFrequencyOffsetDirect() {
  return ((registers[RDA5807M_REG_FREQDIRECT] & RDA5807M_REG_FREQDIRECT_FREQ)) & 0xFF;
}

// get channel by frequency in kHz
uint16_t RDA5807M::getChannelByFrequency(unsigned long val) {
  switch(getRadioBand()) {
    case RDA5807M_REG_CHAN_BAND_65_76:
      val -= 65000;
      if(getLowFreqencyFiftyEnabled()) {
        val -= 15000;
      }
      break;
    case RDA5807M_REG_CHAN_BAND_76_91:
    case RDA5807M_REG_CHAN_BAND_76_108:
      val -= 76000;
      break;
    case RDA5807M_REG_CHAN_BAND_87_108:
      val -= 87000;
      break;
  }
  switch(getChannelSpacing()) {
    case RDA5807M_REG_CHAN_SPACING_25:
      val = val/25;
      break;
    case RDA5807M_REG_CHAN_SPACING_50:
      val = val/50;
      break;
    case RDA5807M_REG_CHAN_SPACING_100:
      val = val/100;
      break;
    case RDA5807M_REG_CHAN_SPACING_200:
      val = val/200;
      break;
  }
  return val & 0xFFFF;
}

// get frequency by channel in kHz
unsigned long RDA5807M::getFrequencyByChannel(uint16_t val) {
  unsigned long result = 0;
    switch(getRadioBand()) {
    case RDA5807M_REG_CHAN_BAND_65_76:
      result = 65000;
      if(getLowFreqencyFiftyEnabled()) {
        result = 15000;
      }
      break;
    case RDA5807M_REG_CHAN_BAND_76_91:
    case RDA5807M_REG_CHAN_BAND_76_108:
      result = 76000;
      break;
    case RDA5807M_REG_CHAN_BAND_87_108:
      result = 87000;
      break;
  }
  switch(getChannelSpacing()) {
    case RDA5807M_REG_CHAN_SPACING_25:
      result += val*25;
      break;
    case RDA5807M_REG_CHAN_SPACING_50:
      result += val*50;
      break;
    case RDA5807M_REG_CHAN_SPACING_100:
      result += val*100;
      break;
    case RDA5807M_REG_CHAN_SPACING_200:
      result += val*200;
      break;
  }
  return result;
}

// get actual* frequency tuned to (*when in channel mode)
unsigned long RDA5807M::getTunedFrequency() {
  if(getFrequencyOffsetDirectEnabled()) { //TODO: try to get the actual value via status registers
    Serial.println("set registers");
    return getFrequencyByChannel(getFrequencyOffsetDirect());
  }
  else {
    readRegisters();
    Serial.println("status registers");
    return getFrequencyByChannel(
        registers[RDA5807M_REG_STATUS1] & RDA5807M_REG_STATUS1_CHAN
      );
  }
}
