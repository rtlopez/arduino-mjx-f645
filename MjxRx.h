#ifndef MjxRx_h
#define MjxRx_h

#include "Arduino.h"
#include "nRF24L01.h"

class MjxControls
{
  public:
    MjxControls():
      throttle(0), yaw(0), pitch(0), roll(0), yaw_trim(0x40), pitch_trim(0x40), roll_trim(0x40), flags(0) {}
    MjxControls(uint8_t throttle_, uint8_t yaw_, uint8_t pitch_, uint8_t roll_, uint8_t yaw_trim_, uint8_t pitch_trim_, uint8_t roll_trim_, uint8_t flags_):
      throttle(throttle_), yaw(yaw_), pitch(pitch_), roll(roll_), yaw_trim(yaw_trim_), pitch_trim(pitch_trim_), roll_trim(roll_trim_), flags(flags_) {}
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch; 
    uint8_t roll;
    uint8_t yaw_trim;
    uint8_t pitch_trim;
    uint8_t roll_trim;
    uint8_t flags;
};

class MjxRx
{
  public:
    typedef enum { 
      THROTTLE = 0, ROLL, PITCH, YAW, 
      ROLL_TRIM, PITCH_TRIM, YAW_TRIM, 
      TXID_0, TXID_1, TXID_2, 
      FLAGS = 14, SUM 
    } data_index_t;
    
    MjxRx(nRF24& radio_);
    void begin();
    void update(MjxControls& ctrl);
    
    uint8_t getThrottle()   const { return data[THROTTLE]; }
    uint8_t getYaw()        const { return data[YAW]; }
    uint8_t getPitch()      const { return data[PITCH]; }
    uint8_t getRoll()       const { return data[ROLL]; }
    uint8_t getYawTrim()    const { return data[YAW_TRIM]; }
    uint8_t getPitchTrim()  const { return data[PITCH_TRIM]; }
    uint8_t getRollTrim()   const { return data[ROLL_TRIM]; }
    uint8_t getFlags()      const { return data[FLAGS]; }
    bool getBound()         const { return bound; }
    MjxControls getControls() const
    {
      return MjxControls(getThrottle(), getYaw(), getPitch(), getRoll(), getYawTrim(), getPitchTrim(), getRollTrim(), getFlags());
    }
    
  private:
    void bindTx();
    void hoopChannel();
    bool isValid(const uint8_t data[16]);

    nRF24&  radio;
    uint8_t data[16];
    uint8_t rf_ch_num;
    uint8_t txid[3];
    uint8_t rf_channels_work[16];
    bool    bound;
    
    static const uint8_t rf_channels_bind[16];
    static const uint8_t freq_hopping[4][16];
    static const char *  rx_tx_addr;
    static const char *  rx_p1_addr;
};


#endif
