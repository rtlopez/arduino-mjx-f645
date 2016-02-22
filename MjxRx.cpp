#include "MjxRx.h"

void serial_dump(const uint8_t * buf, uint8_t len);

const char * MjxRx::rx_tx_addr = "mjsss"; //{ 0x6D, 0x6A, 0x73, 0x73, 0x73 };
const char * MjxRx::rx_p1_addr = "jm777"; //{ 0x6A, 0x6D, 0x37, 0x37, 0x37 };

const uint8_t MjxRx::rf_channels_bind[16] = {
  0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36, 0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18
};

// This is frequency hopping table for MJX V2 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
const uint8_t MjxRx::freq_hopping[4][16] = {
 { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36, 0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 },  //  00 - TX sends BIND sequence using THIS ONE ! (ID is CE D5 00)
 { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16, 0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 },  //  01
 { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A, 0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 },  //  02
 { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D, 0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }   //  03 - TX sends actual packets using THIS ONE !
};

MjxRx::MjxRx(uint8_t ce_pin, uint8_t cs_pin): radio(ce_pin, cs_pin)
{
  for(size_t i = 0; i < sizeof(data); i++) data[i] = 0;
}

void MjxRx::begin()
{
  Serial.println(F(" * Initialization MJX radio start"));
  radio.begin();
  radio.write_register(CONFIG, 0x0C);
  radio.write_register(EN_AA, 0x00);
  radio.write_register(EN_RXADDR, 0x3F);
  radio.write_register(SETUP_AW, 0x03);
  radio.write_register(SETUP_RETR, 0xFF);
  radio.write_register(RF_CH, 0x08);
  radio.write_register(RF_SETUP, 0x07);
  radio.write_register(STATUS, 0x70);
  radio.write_register(OBSERVE_TX, 0x00);
  radio.write_register(CD, 0x00);

  radio.write_register(RX_ADDR_P2, 0xC3);
  radio.write_register(RX_ADDR_P3, 0xC4);
  radio.write_register(RX_ADDR_P4, 0xC5);
  radio.write_register(RX_ADDR_P5, 0xC6);
  
  radio.write_register(RX_PW_P0, 0x10);     // Payload size (16)
  radio.write_register(RX_PW_P1, 0x10);
  radio.write_register(RX_PW_P2, 0x10);
  radio.write_register(RX_PW_P3, 0x10);
  radio.write_register(RX_PW_P4, 0x10);
  radio.write_register(RX_PW_P5, 0x10);
  radio.write_register(FIFO_STATUS, 0x00);

  radio.write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t *>(rx_tx_addr), 5);
  radio.write_register(RX_ADDR_P1, reinterpret_cast<const uint8_t *>(rx_p1_addr), 5);
  radio.write_register(TX_ADDR, reinterpret_cast<const uint8_t *>(rx_tx_addr), 5);
  
  if(1)
  {
    radio.activate(0x53); // magic for BK2421 bank switch
    //Serial.write("Try to switch banks "); Serial.print(radio.read_register(STATUS)); Serial.write("\n");
    if (radio.read_register(STATUS) & 0x80) {
      //Serial.write("BK2421!\n");
      long nul = 0;
      // MJX: Same values
      radio.write_register(0x00, (const uint8_t *) (("\x40\x4B\x01\xE2")), 4);
      radio.write_register(0x01, (const uint8_t *) (("\xC0\x4B\x00\x00")), 4);
      radio.write_register(0x02, (const uint8_t *) (("\xD0\xFC\x8C\x02")), 4);
      radio.write_register(0x03, (const uint8_t *) (("\xF9\x00\x39\x21")), 4);
      radio.write_register(0x04, (const uint8_t *) (("\xC1\x96\x9A\x1B")), 4);
      radio.write_register(0x05, (const uint8_t *) (("\x24\x06\x7F\xA6")), 4);
      radio.write_register(0x06, (const uint8_t *) &nul, 4);
      radio.write_register(0x07, (const uint8_t *) &nul, 4);
      radio.write_register(0x08, (const uint8_t *) &nul, 4);
      radio.write_register(0x09, (const uint8_t *) &nul, 4);
      radio.write_register(0x0A, (const uint8_t *) &nul, 4);
      radio.write_register(0x0B, (const uint8_t *) &nul, 4);
      radio.write_register(0x0C, (const uint8_t *) (("\x00\x12\x73\x00")), 4);
      radio.write_register(0x0D, (const uint8_t *) (("\x46\xB4\x80\x00")), 4);
      radio.write_register(0x0E, (const uint8_t *) (("\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF")), 11);
      radio.write_register(0x04, (const uint8_t *) (("\xC7\x96\x9A\x1B")), 4);
      radio.write_register(0x04, (const uint8_t *) (("\xC1\x96\x9A\x1B")), 4);
    }
    radio.activate(0x53); // switch bank back
  }
  radio.flush_rx();

  radio.write_register(STATUS, 0x0E);
  radio.write_register(CONFIG, 0x0C);
  radio.write_register(CONFIG, 0x0C);
  radio.write_register(CONFIG, 0x0D);
  radio.write_register(CONFIG, 0x0F);
  
  radio.write_register(RF_CH, 0x1B); // ??
  radio.write_register(STATUS, 0x40);
  
  Serial.println(F(" * Initialization MJX radio done"));
}

static uint64_t prev_tm;
;

void MjxRx::update(MjxModel& model)
{
  uint64_t now = millis();
  uint8_t buf[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  boolean update = false;
  static uint64_t dump_tm = 0;
  
  if(radio.available())
  {
    radio.read(buf, sizeof(buf));
    radio.flush_rx();

    //uint8_t channel = radio.read_register(RF_CH);

    // verify package and handle binding
    bool valid = isValid(buf);
    if(valid)
    {
      for(size_t i = 0; i < sizeof(data); i++) data[i] = buf[i];
      if(!bound && getFlags() == 0xC0) bindTx();
    }

    // follow channel hooping
    hoopChannel();
    uint8_t next_ch = bound ? rf_channels_work[rf_ch_num >> 1] : rf_channels_bind[rf_ch_num >> 1];
    radio.write_register(RF_CH, next_ch);

    // dump data
    if(false && now - dump_tm > 1000)
    {
      Serial.print(F("D: ")); serial_dump(buf, sizeof(buf));
      //Serial.print(F(", B: ")); Serial.print(bound);
      //Serial.print(F(", C: ")); Serial.print(channel, HEX);
      //Serial.print(F(", V: ")); Serial.print(valid);
      Serial.print(F(", T: ")); Serial.print((unsigned long)(now - prev_tm));
      Serial.println();
      dump_tm = now;
    }

    prev_tm = now;
    update = true;
  }
  if(now - prev_tm > 200)
  {
    hoopChannel();
    data[THROTTLE] = 0;
    data[ROLL] = 0;
    data[PITCH] = 0;
    data[YAW] = 0;
    data[ROLL_TRIM] = 0x40;
    data[PITCH_TRIM] = 0x40;
    data[YAW_TRIM] = 0x40;
    prev_tm = now;
    update = true;
  }
  if(update) model.updateInput(getInput());
}

void MjxRx::hoopChannel()
{
  rf_ch_num++;
  if(rf_ch_num >= 32) rf_ch_num = 0;
}

bool MjxRx::isValid(const uint8_t data[16])
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 15;  ++i) sum += data[i];
  return data[SUM] == sum;
}

void MjxRx::bindTx()
{
  if(bound) return;
  
  txid[0] = data[TXID_0];
  txid[1] = data[TXID_1];
  txid[2] = data[TXID_2];
  
  const uint8_t sum = txid[0] + txid[1] + txid[2];
  const uint8_t table = sum & 0x03;                    // Base row is defined by lowest 2 bits
  const uint8_t increment = (sum & 0x1C) >> 2;         // Higher 3 bits define increment to corresponding row

  Serial.print(F(" * Binding start: "));
  Serial.print(F("TX ID: "));
  Serial.print(txid[0], HEX);
  Serial.print(F(" "));
  Serial.print(txid[1], HEX);
  Serial.print(F(" "));
  Serial.print(txid[2], HEX);
  Serial.print(F(", tab: "));
  Serial.print(table, HEX);
  Serial.print(F(", inc: "));
  Serial.print(increment, HEX);
  Serial.println();
  
  const uint8_t (&work_row)[16] = freq_hopping[table];
  //const uint8_t (&bind_row)[16] = freq_hopping[0];
  
  //uint8_t work[16];
  //uint8_t bind[16];

  uint8_t val = 0;
  for (int i = 0; i < 16; ++i)
  {
    // Strange avoidance of channels divisible by 16
    //val = bind_row[i];
    //bind[i] = (val & 0x0f) ? val : val - 3;
    val = work_row[i] + increment;
    rf_channels_work[i] = (val & 0x0f) ? val : val - 3;
  }

  bound = true;
  
  /*
  Serial.print(F(" * Bind Channels: "));
  for (int i = 0; i < 16; ++i)
  {
    Serial.print(rf_channels_bind[i], HEX);
    Serial.print(F(", "));
  }
  Serial.println();
  
  Serial.print(F(" * Work Channels: "));
  for (int i = 0; i < 16; ++i)
  {
    Serial.print(rf_channels_work[i], HEX);
    Serial.print(F(", "));
  }
  Serial.println();
  */
  Serial.println(F(" * Binding done"));
}

void serial_dump(const uint8_t * buf, uint8_t len)
{
  for(uint8_t i = 0; i < len; i++)
  {
    if(i) Serial.print(F(" "));
    Serial.print(buf[i], HEX);
  }
}
