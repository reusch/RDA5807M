#ifndef RDA5807M_h
#define RDA5807M_h


#if ARDUINO >= 100
 #include "Arduino.h"
 #define WIRE_WRITE Wire.write
#else
 #include "WProgram.h"
 #define WIRE_WRITE Wire.send
#endif

#define RDA5807M_REG_CTRL_CLKMODE_12  0x0010 // 12MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_13  0x0020 // 13MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_192 0x0030 // 19.2MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_24  0x0050 // 24MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_26  0x0060 // 26MHz clock
#define RDA5807M_REG_CTRL_CLKMODE_327 0x0000 // 32.786MHz clock (default)
#define RDA5807M_REG_CTRL_CLKMODE_384 0x0070 // 38.4MHz clock

#define RDA5807M_REG_CHAN_SPACING_25  0x0003 // 25kHz channel spacing
#define RDA5807M_REG_CHAN_SPACING_50  0x0002 // 50kHz channel spacing
#define RDA5807M_REG_CHAN_SPACING_100 0x0000 // 100kHz channel spacing (default)
#define RDA5807M_REG_CHAN_SPACING_200 0x0001 // 200kHz channel spacing

#define RDA5807M_REG_CHAN_BAND_65_76  0x000C // East Europe FM band
#define RDA5807M_REG_CHAN_BAND_76_91  0x0004 // Japan FM band
#define RDA5807M_REG_CHAN_BAND_76_108 0x0008 // worldwide FM band
#define RDA5807M_REG_CHAN_BAND_87_108 0x0000 // US/Europe band (default)

class RDA5807M {
  public:
    RDA5807M();
    bool begin();
    void setChipEnabled(bool e);
    bool getChipEnabled();
    void setNewModulatorEnabled(bool e);
    bool getNewModulatorEnabled();
    void setRDSEnabled(bool e);
    bool getRDSEnabled();
    void setClockFrequency(uint16_t val);
    uint16_t getClockFrequency();
    void setSeekWrapEnabled(bool e);
    bool getSeekWrapEnabled();
    void startSeek();
    void setSeekDirectionUpEnabled(bool e);
    bool getSeekDirectionUpEnabled();
    void setRCKLDirectInputEnabled(bool e);
    bool getRCKLDirectInputEnabled();
    void setRCLKNotCalibrated(bool e);
    bool getRCLKNotCalibrated();
    void setBassBoostEnabled(bool e);
    bool getBassBoostEnabled();
    void setMonoEnabled(bool e);
    bool getMonoEnabled();
    void setMuteEnabled(bool e);
    bool getMuteEnabled();
    void setOutputHighZEnabled(bool e);
    bool getOutputHighZEnabled();
    void setChannelSpacing(uint16_t val);
    uint16_t getChannelSpacing();
    void setRadioBand(uint16_t val);
    uint16_t getRadioBand();
    void setTuneEnabled(bool e);
    bool getTuneEnabled();
    void setDirectControlEnabled(bool e);
    bool getDirectControlEnabled();
    void setChannel(uint16_t val);
    uint16_t getChannel();
    void setAFCEnabled(bool e);
    bool getAFCEnabled();
    void setSoftmuteEnabled(bool e);
    bool getSoftmuteEnabled();
    void setDeemphasisFiftyEnabled(bool e);
    bool getDeemphasisFiftyEnabled();
    void setVolume(uint8_t val);
    uint8_t getVolume();
    void setSeekThreshold(uint8_t val);
    uint8_t getSeekThreshold();
    void setIntAfterReg0CReadEnabled(bool e);
    bool getIntAfterReg0CReadEnabled();
    void setOpenModeEnabled(bool e);
    bool getOpenModeEnabled();
    void setFrequencyOffsetDirectEnabled(bool e);
    bool getFrequencyOffsetDirectEnabled();
    void setSoftblendEnabled(bool e);
    bool getSoftblendEnabled();
    void setOldSeekThresholdEnabled(uint8_t val);
    uint8_t getOldSeekThresholdEnabled();
    void setLowFreqencyFiftyEnabled(bool e);
    bool getLowFreqencyFiftyEnabled();
    void setSoftblendThreshold(uint8_t val);
    uint8_t getSoftblendThreshold();
    void setFrequencyOffsetDirect(uint16_t val);
    uint8_t getFrequencyOffsetDirect();
    uint16_t getChannelByFrequency(unsigned long val);
    unsigned long getFrequencyByChannel(uint16_t val);
    unsigned long getTunedFrequency();
  private:
    // register storage
    uint16_t registers[16];

    void readRegisters();
    void writeRegisters();
    void writeRegister(int reg);
};
#endif
