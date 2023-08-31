/**
 * Date: 04/08/2023
 * Notes: Implements the synchronization algorithm employing the software millis() function
 * instead of the hardware timer.
 * 
 * On: 17/08/2023
 * Proceed to implement a minimalistic LoRaWAN end node by employing the LoRaWAN frame structure.
 * Adding mechanism like payload encryption, Message Authentication code
*/

// General remark: remember to pad the stream obtained from contatening B0 and msg, before computing the cmac 

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <AES.h>
#include <AES_CMAC.h>


void transmit();                // implement the transmission sequences algorithm
void process_rx();              // process the data received in the last downlink
bool successful_tx = false;     // is set according to whether the last uplink transmission was successful
bool is_sync = false;           // true if the device has already issued a synchronization
unsigned long slot_start_ref;   // Reference of slot start
unsigned long beg;              // set to the moment an uplink is achieved
unsigned long elapsed;          // amount of time elapsed since the uplink was finished to the moment when a downlink answer was got.
unsigned long remaining_time;    // remaining time to the beginning of the next slot
long t;
uint8_t n;

#define T_SLOT 1757   // Slot duration in ms
#define T_B1 180  // left guard slot in ms
#define T_TX 306  // ms
unsigned long duty_cyc_delay = T_TX*100 - 1000;  // -1000 because of the delay between Tx and Rx  

unsigned long tmp1_millis;
unsigned long tmp2_millis;
long tmp_t;

#define OUT_PIN 12
bool state = false;

// >>>>>>> Encryption and LoRaWAN Frame >>>>>>>
const uint8_t key[16] = {
  0x2b, 0x7e, 0x15, 0x16,
  0x28, 0xae, 0xd2, 0xa6,
  0xab, 0xf7, 0x15, 0x88,
  0x09, 0xcf, 0x4f, 0x3c,
};

#define UP_FRM_PAYLOAD_SIZE  176   // Uplink Frame Payload size
#define UP_PHY_PAYLOAD_SIZE  189   // Uplink Physical Payload size; UP_FRM_PAYLOAD_SIZE + 13 (no FOpts); +4 should be added on the server side to account for the Radio Head headers

// #define DW_FRM_PAYLOAD_SIZE  
#define DW_PHY_SIZE_SYNC    15    // Downlink physical payload size if sync bytes included in FOpts 
#define DW_PHY_SIZE_NOSYNC  13    // ... with no sync bytes

// Fields positioning inside the physical payload in the Uplink :
#define UP_MHDR_POS  0
#define UP_DEVADDR_POS 1
#define UP_FCTRL_POS 5
#define UP_FCNT_POS 6
#define UP_FPORT_POS 8
#define UP_FRMPAYLOAD_POS 9
#define UP_MIC_POS 185

// Fields positions inside the Downlink physical payload:
// Positions with 'SYNC' at the end relate to frame containing sync data (2 bytes) from the NS
// Those with ending with 'NO_SYNC' are for frame structure used for Acknowledgement only 
#define DW_MHDR_POS 0
#define DW_DEVADDR_POS 1
#define DW_FCTRL_POS 5
#define DW_FCNT_POS 6
#define DW_FOPTS_POS_SYNC 8
#define DW_FPORT_POS_SYNC 10
#define DW_MIC_POS_SYNC 11
#define DW_FPORT_POS_NOSYNC 8
#define DW_MIC_POS_NOSYNC 9

#define MIC_SIZE 4    // size of the MIC field (in bytes)

uint8_t up_mhdr;    // uplink message header
// uint8_t dw_mhdr;    // downlink MAC header
uint32_t dev_addr;  // device address
uint8_t f_ctrl;     // Frame control
uint16_t f_cnt;     // frame counter
uint8_t f_port;     // frame port
uint8_t frm_payload[UP_FRM_PAYLOAD_SIZE];  // frame payload
uint32_t mic;       // message integrity code
uint8_t up_b0[16];     // uplink B0, used in computing cmac
uint8_t dw_b0[16];     // downlink B0
uint8_t mic_calc_buf[16 + UP_PHY_PAYLOAD_SIZE - 4]; // store the array used to calcuate cmac; size = size of 'B0' + size of 'phy_payload' - size of 'mic'. Used also with DW messages
uint8_t mac[16];     // store the computed cmac




AES128 m_aes128;
AESTiny128 tiny_aes128;
AES_CMAC cmac(tiny_aes128); // used for cmac computation

uint8_t phy_payload[UP_PHY_PAYLOAD_SIZE];  // Physical payload


void fcnt_into_b0(uint16_t fcnt, uint8_t* b0);  // adds fcnt(as uint32_t) at index 10 of the given b0

// Encrypt plaintext 'p_test' of length 'len' into cyphertext 'c_text' of the same length in ECB fashion
// using the AES128 object 'cipher'. The key of cipher should be already set before calling this function,
// p_text should be padded and enough space should be allocated in c_text.
// p_text will be processed by blocks of 16 bytes.
void ecb128_encrypt(const uint8_t* p_text, uint8_t len , uint8_t* c_text, AES128 *cipher);

// Same thing, but now for decryption
void ecb128_decrypt(const uint8_t* c_text, uint8_t len , uint8_t* output, AES128 *cipher);

// <<<<<<< Encryption and LoRaWAN Frame <<<<<<


// Pins def
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

// Freq
#define RF95_FREQ 868.1

// #define TX_PL_LEN  180    // payload length
#define RX_PL_LEN 15  // Max value for the current config
#define TX_SF 7     // Spreading factor used on uplink
#define RX_SF 8     // downlink SF

// Radio Instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char msg[UP_FRM_PAYLOAD_SIZE] = "";   // Tx message
                    // 28 chars for the constant part above + 10 chars used to encode
                   // the 32 bits returned by the millis function + 2 chars for 'ms' + 1 char for the null character    

uint8_t len = RX_PL_LEN;
uint8_t rx_buf[RX_PL_LEN + 1]; // Reception buffer

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(9600);
  while(!Serial)
    delay(100);

  Serial.println("Feather LoRa Tx Test!");

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  // Radio Head init
  while(!rf95.init()) {
    Serial.println("LoRa init failed!");
    Serial.println("Enable SERIAL_DEBUG for detailed info!");
    while(1);
  }

  Serial.println("LoRa Radio Init OK!");

  // Freq init
  if(!rf95.setFrequency(RF95_FREQ)){
    Serial.println("Set Freq failed!");
    while(1);
  }


  Serial.print("Freq set to: ");
  Serial.println(RF95_FREQ);

  /* The unset default parameters are:
   * 13dBm, Bw = 125kHz, Cr = 4/5, SF = 128chips/symbols (7), CRC is on
   */

  memset(rx_buf, '\0', len+1);
  memset(msg, '\0', UP_FRM_PAYLOAD_SIZE);

  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, state);

  // Setting up frame fields
  up_mhdr = 0b100<<5 | 0x00 | 0x00;   // Mtype(Confirmed Data Up), RFU(0), Major(0)
  dev_addr = 0x26011A1E;              // arbitrary, does not reflect an actual application
  f_ctrl = 0b00000000;                // on uplink: ADR(0), ADRAckReq(0), ACK(0), FPending(0), FOptslen(0000)
  f_cnt = 0;                          // initial value
  f_port = 198;                       // any value between 1 and 223 is fine

  up_b0[0] = 0x49;
  up_b0[1] = 0x00;
  up_b0[2] = 0x00;
  up_b0[3] = 0x00;
  up_b0[4] = 0x00;
  up_b0[5] = 0x00;  // Dir=0 for uplink
  *(uint32_t*) (up_b0+6) = dev_addr;
  // f_cnt should added at index 10 as uint32_t, before calculating cmac for each message
  up_b0[14] = 0x00;
  up_b0[15] = UP_FRM_PAYLOAD_SIZE + 8 + 1;   // MHDR(1), FHDR(7), FPort(1), FRMPayload(UP_FRM_PAYLOAD_SIZE) 

  dw_b0[0] = 0x49;
  dw_b0[1] = 0x00;
  dw_b0[2] = 0x00;
  dw_b0[3] = 0x00;
  dw_b0[4] = 0x00;
  dw_b0[5] = 0x01;  // Dir=1 for downlink
  *(uint32_t*) (dw_b0+6) = dev_addr;
  // f_cnt should added at index 10 as uint32_t
  dw_b0[14] = 0x00;
  // the len should be added to index 15 once it is determined upon message reception

  m_aes128.setKey(key, 16);

  // Write the static fields of 'phy_payload' from here. These fields will not change in our current example
  *(phy_payload + UP_MHDR_POS) = up_mhdr;
  * (uint32_t*) (phy_payload + UP_DEVADDR_POS) = dev_addr;
  * (phy_payload + UP_FCTRL_POS) = f_ctrl;
  * (phy_payload + UP_FPORT_POS) = f_port;
}

void loop() {
  sprintf(msg, "Hello from Adafruit Feather LoRa node to Mohammed VI Polytechnic University Network Server. This is a test of the synchronization algorithm. I have running for %010lu ms!!", millis());
  
  ecb128_encrypt((uint8_t*)msg, UP_FRM_PAYLOAD_SIZE, frm_payload, &m_aes128); // Encrypt the message into frame payload

  // Uptade the dynamic fields of 'phy_payload' (f_cnt and frm_payload)
  * (uint16_t*) (phy_payload + UP_FCNT_POS) = f_cnt;                        // Copy 'f_cnt' to its position
  memcpy(phy_payload+UP_FRMPAYLOAD_POS, frm_payload, UP_FRM_PAYLOAD_SIZE); // as well as 'frm_payload' array

  // Not we should compute the MIC
  // First update up_b0
  fcnt_into_b0(f_cnt, up_b0);
  memcpy(mic_calc_buf, up_b0, 16); // copy 'up_b0'
  memcpy(mic_calc_buf+16, phy_payload, UP_PHY_PAYLOAD_SIZE - 4); // now append 'phy_payload' without the MIC.
  cmac.generateMAC(mac, key, mic_calc_buf, 16 + UP_PHY_PAYLOAD_SIZE - 4);  // perform the cmac generation
  mic = *(uint32_t*) mac;     // take the first four byte as mic
  *(uint32_t*) (phy_payload + UP_MIC_POS) = mic;

  // Verification step:
  // Serial.printf("MHDR: %02X\n", *phy_payload);
  // Serial.printf("DevAddr: %08X\n", *(uint32_t*)(phy_payload+UP_DEVADDR_POS));
  // Serial.printf("FCtrl: %02X\n", *(phy_payload+UP_FCTRL_POS));
  // Serial.printf("FCnt: %04X\n", *(uint16_t*)(phy_payload+UP_FCNT_POS));
  // Serial.printf("FPort: %02X\n", *(phy_payload+UP_FPORT_POS));

  // Serial.print("FrmPayload(Enc)");
  // for(uint8_t i=UP_FRMPAYLOAD_POS; i<(UP_FRMPAYLOAD_POS+UP_FRM_PAYLOAD_SIZE); i++){
  //   Serial.printf("%02X ", *(phy_payload+i));
  // }
  // Serial.printf("MIC: %08X\n", mic);

  // Transmit the payload

  Serial.println("Sending packet ...");
  transmit();
  
  rf95.setSpreadingFactor(RX_SF);
  rf95.setPayloadCRC(false);      // disable the CRC for reception
  Serial.println("Waiting to receive packet ...");
  if(rf95.waitAvailableTimeout(2000)){
    len = RX_PL_LEN;       // len is a value-result variable for the .recv() method
    if(rf95.recv(rx_buf, &len)){
      process_rx();
      memset(rx_buf, '\0', len+1);
    }
    else
      Serial.println("Reception failed!");
  }
  else{
    Serial.println("No message received!");
  }

  delay(duty_cyc_delay);

}

void transmit(){  // uses global var 'msg'
  rf95.setSpreadingFactor(TX_SF);
  rf95.setPayloadCRC(true);       // enable the CRC for transmission
  if(!is_sync){                 // if this is the first Tx (not already synchronized)
    slot_start_ref = millis(); // take the current moment as a ref (this is just a bet)
    rf95.send((uint8_t *) phy_payload, UP_PHY_PAYLOAD_SIZE);  // send what is in 'msg'
    rf95.waitPacketSent();                 // wait until Tx finishes
    beg = millis();           // take a time ref
    is_sync = true;
    return;
  }

  // Not the first Tx
  delay(T_SLOT - (millis() - slot_start_ref)%T_SLOT);  // wait until the next slot
  // and transmit
  delay(T_B1);
  digitalWrite(OUT_PIN, !state);
  state = ! state;
  Serial.println(millis());
  rf95.send((uint8_t *) phy_payload, UP_PHY_PAYLOAD_SIZE);  
  rf95.waitPacketSent();                 
  beg = millis();           // take a time ref
}

void process_rx(){  // uses global vars 'rx_buf' and 'len'
  if(len == DW_PHY_SIZE_NOSYNC){  // ACK-only packet received
    // check packed validity... (DevAddr, MIC, FCnt)
    if(*(uint32_t*)(rx_buf+DW_DEVADDR_POS) != dev_addr){
      Serial.println("Wrong dev_addr!");
      return;
    }

    fcnt_into_b0(f_cnt, dw_b0);       // update f_cnt field in b0
    dw_b0[15] = (uint8_t) (len - MIC_SIZE); // same for length field

    // Now we compute the mic (using 'mic_calc_buf' again, as it is large enough) 
    memcpy(mic_calc_buf, dw_b0, 16);
    memcpy(mic_calc_buf+16, rx_buf, len-MIC_SIZE);
    cmac.generateMAC(mac, key, mic_calc_buf, 16+len-MIC_SIZE);
    if(*(uint32_t*)mac != *(uint32_t*)(rx_buf+DW_MIC_POS_NOSYNC)){
      Serial.println("Invalid MIC");
      return;
    }

    // Other fields like f_cnt, f_ctrl could be checked here if needed

    f_cnt++;  // f count up
    return;
  }
  
  if(len == DW_PHY_SIZE_SYNC){  // SYNC-info packet received
    if(*(uint32_t*)(rx_buf+DW_DEVADDR_POS) != dev_addr){
      Serial.println("Wrong dev_addr!");
      return;
    }

    fcnt_into_b0(f_cnt, dw_b0);       // update f_cnt field in b0
    dw_b0[15] = (uint8_t) (len - MIC_SIZE); // same for length field

    // Now we compute the mic (using 'mic_calc_buf' again, as it is large enough) 
    memcpy(mic_calc_buf, dw_b0, 16);
    memcpy(mic_calc_buf+16, rx_buf, len-MIC_SIZE);
    cmac.generateMAC(mac, key, mic_calc_buf, 16+len-MIC_SIZE);
    if(*(uint32_t*)mac != *(uint32_t*)(rx_buf+DW_MIC_POS_SYNC)){
      Serial.println("Invalid MIC");
      Serial.printf("Rec MHDR: %02X\n", *(rx_buf+DW_MHDR_POS));
      Serial.printf("Rec dev_addr: %08X\n", *(uint32_t*)(rx_buf+DW_DEVADDR_POS));
      Serial.printf("Rec FCTRL: %02X\n", *(rx_buf+DW_FCTRL_POS));
      Serial.printf("Rec FCNT: %04X\n", *(uint16_t*)(rx_buf+DW_FCNT_POS));
      Serial.printf("Rec FOPTS: %u\n", *(uint16_t*)(rx_buf+DW_FOPTS_POS_SYNC));
      Serial.printf("Rec MIC: %08X\n", *(uint32_t*)(rx_buf+DW_MIC_POS_SYNC));
      Serial.printf("Cal MIC: %08X\n", *(uint32_t*)(mac));
      Serial.printf("dw_b0: ");

      return;
    }

    // Other fields like f_cnt, f_ctrl could be checked here if needed

    // retrieve the remaining time as sent by the Gateway
    elapsed = millis() - beg;
    remaining_time = *(uint16_t*) (rx_buf+DW_FOPTS_POS_SYNC); // retrieve the 2 FOPTS bytes (sync data)

    t = remaining_time - elapsed;   // update the remaining time to the current value it should be
    if(t < 0){
      n = abs(t)/T_SLOT;
      t += (n+1)*T_SLOT;
    }
    delay(t);                   // wait for the remaining time to elapse
    slot_start_ref = millis(); // then takes the start of the next slot as the new ref.

    Serial.printf("Rem_time: %u\n", remaining_time);
    f_cnt++;  // f count up
    return;
  }
  

  Serial.printf("Packet size not supported: %u!\n", len);
}


/**
 * General notes:
 * We can also implement the transmit() function to be non blocking.
 * In that case, it will simply return the start of the next slot
 * that can be used in the main loop.
 * Same for the process_rx() function when the last delay() can be avoided
 * by setting slot_start_ref to millis() + t. The only potential issue here would
 * be the device trying a transmission when the current value of millis() is still
 * less than this new value of slot_start_ref.
*/

void fcnt_into_b0(uint16_t fcnt, uint8_t* b0){
 *(uint32_t*) (b0+10) = (uint32_t) fcnt;
}

void ecb128_encrypt(const uint8_t* p_text, uint8_t len , uint8_t* c_text, AES128 *cipher){
  for(uint8_t i=0; i<len/16; i++){
    cipher->encryptBlock(c_text, p_text);
    p_text += 16;
    c_text += 16;
  }
}

void ecb128_decrypt(const uint8_t* c_text, uint8_t len , uint8_t* output, AES128 *cipher){
  for(uint8_t i=0; i<len/16; i++){
    cipher->decryptBlock(output, c_text);
    c_text += 16;
    output += 16;
  }
}