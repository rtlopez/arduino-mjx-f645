#ifndef MjxRx_h
#define MjxRx_h

#include "Arduino.h"
#include "MjxModel.h"
#include "libs/nRF24L01/nRF24L01.h"

class MjxRx
{
  public:
    typedef enum { 
      THROTTLE = 0, ROLL, PITCH, YAW, 
      ROLL_TRIM, PITCH_TRIM, YAW_TRIM, 
      TXID_0, TXID_1, TXID_2, 
      FLAGS = 14, SUM 
    } data_index_t;
    
    MjxRx(MjxModel& m);
    void begin();
    void update();
    
  private:
    void bindTx(const uint8_t data[16]);
    void hoopChannel();
    bool isValid(const uint8_t data[16]);

    MjxModel& model;
    nRF24  radio;
    uint8_t rf_ch_num;
    uint8_t txid[3];
    uint8_t rf_channels_work[16];
    bool    bound;
    uint64_t prev_tm;
    
    static const uint8_t rf_channels_bind[16];
    static const uint8_t freq_hopping[4][16];
    static const char *  rx_tx_addr;
    static const char *  rx_p1_addr;
};


#endif
