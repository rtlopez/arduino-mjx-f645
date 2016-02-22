#ifndef MjxRx_h
#define MjxRx_h

#include "Arduino.h"
#include "nRF24L01.h"
#include "MjxModel.h"

class MjxRx
{
  public:
    typedef enum { 
      THROTTLE = 0, ROLL, PITCH, YAW, 
      ROLL_TRIM, PITCH_TRIM, YAW_TRIM, 
      TXID_0, TXID_1, TXID_2, 
      FLAGS = 14, SUM 
    } data_index_t;
    
    MjxRx(uint8_t ce_pin, uint8_t cs_pin);
    void begin();
    void update(MjxModel& model);
    
    uint8_t getThrottle()   const { return data[THROTTLE]; }
    uint8_t getYaw()        const { return data[YAW]; }
    uint8_t getPitch()      const { return data[PITCH]; }
    uint8_t getRoll()       const { return data[ROLL]; }
    uint8_t getYawTrim()    const { return data[YAW_TRIM]; }
    uint8_t getPitchTrim()  const { return data[PITCH_TRIM]; }
    uint8_t getRollTrim()   const { return data[ROLL_TRIM]; }
    uint8_t getFlags()      const { return data[FLAGS]; }
    bool getBound()         const { return bound; }
    
    MjxInput getInput() const
    {
      return MjxInput(getThrottle(), getYaw(), getPitch(), getRoll(), getYawTrim(), getPitchTrim(), getRollTrim(), getFlags());
    }
    
  private:
    void bindTx();
    void hoopChannel();
    bool isValid(const uint8_t data[16]);

    nRF24  radio;
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
