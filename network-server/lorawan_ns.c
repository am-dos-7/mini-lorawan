#include <stdlib.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <openssl/cmac.h>

#define SERV_PORT 2000      // server port
#define CLT_PORT 2001       // client port
#define MAX_UP_PL  255         // max uplink payload size
#define PL_HD_SIZE 4        // size of the mandatory header included by the RF_95 library. This header will be skipped when processing the payload
#define MAX_DW_PL 15       // max downlink payload size

// >>>>>>> Encryption and LoRaWAN Frame >>>>>>>>>>

const uint8_t key[16] = {
  0x2b, 0x7e, 0x15, 0x16,
  0x28, 0xae, 0xd2, 0xa6,
  0xab, 0xf7, 0x15, 0x88,
  0x09, 0xcf, 0x4f, 0x3c,
};

#define UP_FRM_PAYLOAD_SIZE  176   // Uplink Frame Payload size
#define UP_PHY_PAYLOAD_SIZE  189   // Uplink Physical Payload size; UP_FRM_PAYLOAD_SIZE + 13 (no FOpts)

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

uint8_t dw_mhdr;        // downlink MAC header
// uint32_t dev_addr;
uint8_t f_ctrl;         // frame control
uint16_t f_cnt;         // frame counter
uint8_t f_port;
uint8_t up_frm_payload[UP_FRM_PAYLOAD_SIZE]; // uplink frame payload, once decrypted
uint32_t mic;
uint8_t up_b0[16];
uint8_t dw_b0[16];
uint8_t mic_calc_buf[16+UP_PHY_PAYLOAD_SIZE-MIC_SIZE];
uint8_t mac[16];
size_t mac_len = 16;     // Length of mac

uint8_t pl[MAX_UP_PL]; // Will hold the physical payload on the uplink as well as the downlink
uint8_t dw_pl[1+MAX_DW_PL]; // The +1 (first byte) is to specify the len of the frame (for the 'byteToPMT   ' custom block in GNU Radio)
uint8_t *up_phy_payload;    // points to the beginning of the effective uplink 'phy' payload inside 'pl'
uint8_t *dw_phy_payload;    // ... of the effective downlink 'phy' payload inside 'pl'

CMAC_CTX *ctx;      // object used for CMAC computation

// <<<<<<< Encryption and LoRaWAN Frame <<<<<<<<<<

// >>>>>>>>> Slot length components (in ms)  >>>>>>>>>>>
#define T_TX 306     // 306.432 normally
#define T_RX 91     // 90.624 normally
#define T_B1 180
#define T_B2 180

uint16_t slot_len = T_TX + 1000 + T_RX + T_B1 + T_B2;

#define SYS_LOG_FILE "slot_violation.txt"  // log file of packet arrival and potential slot violation at network server.

struct sockaddr_in client_addr;

// uint32_t packt_ind = 0; // received packet index

FILE* time_log;

void sigint_handler(){  // close the file when Ctrl-C is pressed, so that data in the write buffer could be written to file
    fclose(time_log);
    fprintf(stdout, "\nCtrl-C pressed. Shutting server down ...\n");

    exit(EXIT_SUCCESS);
}

void sub_timespec(const struct timespec *tp1, const struct timespec *tp2, struct timespec *tp_res){   // substraction of timespecs: tp_res = tp1 - tp2
    tp_res->tv_sec = tp1->tv_sec - tp2->tv_sec;
    tp_res->tv_nsec = tp1->tv_nsec - tp2->tv_nsec;
    if(tp_res->tv_nsec < 0){        // handle the case of negative values 
        tp_res->tv_nsec += 1000000000;
        tp_res->tv_sec --;
    }
}

void fcnt_into_b0(uint16_t fcnt, uint8_t* b0){
 *(uint32_t*) (b0+10) = (uint32_t) fcnt;
}

int main(int argc, char *argv){
    
    printf("slot_len: %u\n", slot_len);
    
    struct sockaddr_in server_addr;
    int in_socket_des;     // input socket descriptor, for incoming data
    int out_socket_des;     // output socket descriptor
    struct timespec slot_start_ref;  // time reference of the first slot's start
    struct timespec packt_arrival;   // arrival time of the current packet
    struct timespec time_op_res;          // used to store operation result (between 'slot_start_ref' and 'packt_arrival')
    uint16_t slot_pos;              // position of the ongoing slot which the current packet has been received at. 
    uint16_t time_to_next_slot;     // remaining time until next slot (in ms)

    // Setup
    dw_mhdr = 0b011<<5 | 0x00 | 0x00;   // Mtype(Unconfirmed Data Down), RFU(0), Major(0)
    // f_ctrl = 0b00100000;                // on downlink: ADR(0), ADRAckReq(0), ACK(1), FPending(0), FOptsLen is 0b0000 for NOSYNC and 0b0010 for SYNC (should be set before sending)
    f_port = 198;

    up_b0[0] = 0x49;
    *(uint32_t*)(up_b0+1) = (uint32_t) 0x00;
    up_b0[5] = 0x00;    // Dir=0 for uplink
    // dev_addr shall be added at index 6 as uint32_t once retrieved from uplink frame
    //  same with f_cnt, which shall be added at index 10.
    up_b0[14] = 0x00;
    up_b0[15] = UP_FRM_PAYLOAD_SIZE + 8 + 1;    // MHDR(1), FHDR(7), FPort(1), FRMPayload(UP_FRM_PAYLOAD_SIZE)
                                               // this is because the length of uplink packet is chosen in advance and we do not use any MAC Cmd;
    dw_b0[0] = 0x49;
    *(uint32_t*)(dw_b0+1) = (uint32_t) 0x00;
    dw_b0[5] = 0x01;    // Dir=1 for downlink
    // dev_addr shall be added at index 6 as uint32_t once retrieved from uplink frame
    //  same with f_cnt, which shall be added at index 10.
    dw_b0[14] = 0x00;
    // the len shall be added at index 15 before sending the downlink, depending on whether the SYNC data is included.

    // ctx = CMAC_CTX_new();
    // CMAC_Init(ctx, key, 16, EVP_aes_128_cbc(), NULL);

    signal(SIGINT, sigint_handler); // register handler for SIGINT signal

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = atoi("0.0.0.0");  // all interfaces
    server_addr.sin_port = htons(SERV_PORT);

    if((in_socket_des = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
        perror("Error on socket creation");
        exit(EXIT_FAILURE);
    }

    if(bind(in_socket_des, (struct sockaddr*) &server_addr, sizeof(server_addr)) < 0){
        perror("Error while binding socket to addr");
        exit(EXIT_FAILURE);
    }

    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = atoi("127.0.0.1");
    client_addr.sin_port = htons(CLT_PORT);

    if((out_socket_des = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
        perror("Error on output socket creation");
        exit(EXIT_FAILURE);
    }

    if((time_log = fopen(SYS_LOG_FILE, "w")) == NULL){
        perror("Error while opening time_log file");
        exit(EXIT_FAILURE);
    }

    memset(pl, '\0', 255);
    fprintf(stdout, "Strlen(pl): %lu\n", strlen(pl));
    fprintf(stdout, "Waiting for data reception...\n");
    char out[9] = {'A', 'b', 'C', 'd', 'E', 'f', 'G', 'h', '\0'};   // only bytes at position 0 and 1 are relevant

    if(clock_gettime(CLOCK_REALTIME, &slot_start_ref) != 0){    // initialize the first slot start
        perror("Error on gettimeofday()");
        exit(EXIT_FAILURE);
    }
    

    int n_bytes;      // number of bytes received 
    while(1){
        if((n_bytes = recv(in_socket_des, pl, MAX_UP_PL, 0)) < 0){
            perror("Socket reception error!");
            exit(EXIT_FAILURE);
        }

        fprintf(stdout, "n_bytes:%d\n", n_bytes);
        printf("Headers: ");
        for(int i=0; i<PL_HD_SIZE; i++)
            printf("%u ", *(pl+i));
        printf("\n");

        if(clock_gettime(CLOCK_REALTIME, &packt_arrival) != 0){
            perror("Error on gettimeofday()");
            exit(EXIT_FAILURE);
        }                                          // get packet arrival time

        if(n_bytes != (UP_PHY_PAYLOAD_SIZE+PL_HD_SIZE)){
            fprintf(stdout, "Packet too short: len=%d\n", n_bytes);
            continue;
        }

        // Check packet integrity
        up_phy_payload = pl+PL_HD_SIZE;     // Jump over the first four bytes in 'pl' (Radio Head lib headers)
        *(uint32_t*)(up_b0+6) = *(uint32_t*)(up_phy_payload+UP_DEVADDR_POS);    // get dev_addr into 'up_b0'
        f_cnt = *(uint16_t*)(up_phy_payload+UP_FCNT_POS);
        fcnt_into_b0(f_cnt, up_b0);

        printf("Received FCNT: %04X\n", *(uint16_t*)(up_phy_payload+UP_FCNT_POS));
        fprintf(stdout, "Received MIC: %08X\n", *(uint32_t*)(up_phy_payload+UP_MIC_POS));

        // Now MIC computation
        memcpy(mic_calc_buf, up_b0, 16);    // copy up_b0 
        memcpy(mic_calc_buf+16, up_phy_payload, UP_PHY_PAYLOAD_SIZE-MIC_SIZE); // append the phy payload without the MIC

        ctx = CMAC_CTX_new();
        CMAC_Init(ctx, key, 16, EVP_aes_128_cbc(), NULL);
        CMAC_Update(ctx, mic_calc_buf, 16+UP_PHY_PAYLOAD_SIZE-MIC_SIZE);
        CMAC_Final(ctx, mac, &mac_len);
        CMAC_CTX_free(ctx);

        if(*(uint32_t*)mac != *(uint32_t*)(up_phy_payload+UP_MIC_POS)){
            fprintf(stdout, "MIC does not match\n");
            fprintf(stdout, "Received MIC: %08X Vs Computed MIC: %08X\n", *(uint32_t*)(up_phy_payload+UP_MIC_POS), *(uint32_t*)mac);
            fprintf(stdout, "Received dev_addr: %08X\n", *(uint32_t*)(up_phy_payload+UP_DEVADDR_POS));
            printf("Received MHDR:%02X\n", *(up_phy_payload+UP_MHDR_POS));
            printf("Received FCNT:%04X\n", *(uint16_t*)(up_phy_payload+UP_FCNT_POS));
            printf("Received FCTRL:%02X\n", *(up_phy_payload+UP_FCTRL_POS));
            printf("Received FPORT:%02X\n", *(up_phy_payload+UP_MHDR_POS));
            // printf("FRMPayload");         
            // for(uint8_t i=0; i<UP_FRM_PAYLOAD_SIZE; i++){
            //     printf("%02X ", *(up_phy_payload+UP_FRMPAYLOAD_POS+i));
            // }
            
            continue;
        }
        fprintf(stdout, "MIC matches\n");

        // dev_addr and f_cnt into dw_b0
        *(uint32_t*)(dw_b0+6) = *(uint32_t*)(up_phy_payload+UP_DEVADDR_POS);
        fcnt_into_b0(f_cnt, dw_b0);     // 'f_cnt' was already retrieved when filling 'up_b0'

        dw_phy_payload = dw_pl+1;   // The first byte will be used to specify the length of 'pl' to the Gateway (Radio interface)
                                        

        sub_timespec(&packt_arrival, &slot_start_ref, &time_op_res);                       // compute the time difference
        slot_pos = (time_op_res.tv_sec*1000 + time_op_res.tv_nsec/1000000)%slot_len;  // then deduce the position in the slot
        if((slot_pos < T_TX) || (slot_pos > (T_TX+T_B1+T_B2))){
            time_to_next_slot = slot_len - slot_pos;        // get the time remaining until the next slot

            dw_pl[0] = DW_PHY_SIZE_SYNC;
            f_ctrl = 0b00100010;                // on downlink: ADR(0), ADRAckReq(0), ACK(1), FPending(0), FOptsLen(0010) 

            // fill dowlink payload fields
            dw_phy_payload[0] = dw_mhdr;                                            // m_hdr
            *(uint32_t*)(dw_phy_payload+DW_DEVADDR_POS) = *(uint32_t*)(dw_b0+6);    // dev_addr
            *(dw_phy_payload+DW_FCTRL_POS) = f_ctrl;
            *(uint16_t*)(dw_phy_payload+DW_FCNT_POS) = f_cnt;
            *(uint16_t*)(dw_phy_payload+DW_FOPTS_POS_SYNC) = time_to_next_slot;
            *(dw_phy_payload+DW_FPORT_POS_SYNC) = f_port;

            dw_b0[15] = DW_PHY_SIZE_SYNC - MIC_SIZE;       // Length
            memcpy(mic_calc_buf, dw_b0, 16);
            memcpy(mic_calc_buf+16, dw_phy_payload, DW_PHY_SIZE_SYNC-MIC_SIZE);

            ctx = CMAC_CTX_new();
            CMAC_Init(ctx, key, 16, EVP_aes_128_cbc(), NULL);
            CMAC_Update(ctx, mic_calc_buf, 16+DW_PHY_SIZE_SYNC-MIC_SIZE);
            CMAC_Final(ctx, mac, &mac_len);
            CMAC_CTX_free(ctx);
            *(uint32_t*)(dw_phy_payload+DW_MIC_POS_SYNC) = *(uint32_t*)mac;

            fprintf(stdout, "Resync\n");
            usleep(950000); // should normally take into account the time spent in all the above operations 
            // and send the content of 'out' as a reply
            if((sendto(out_socket_des, dw_pl, 1+MAX_DW_PL, 0, (struct sockaddr *) &client_addr, sizeof(client_addr))) < 0){
                perror("Socket write error");
                pthread_exit((void*) EXIT_FAILURE);
            }
            fprintf(stdout, "time to next:%u\n", time_to_next_slot);
        }
        
        else{
            dw_pl[0] = DW_PHY_SIZE_NOSYNC;
            f_ctrl = 0b00100000;                // on downlink: ADR(0), ADRAckReq(0), ACK(1), FPending(0), FOptsLen(0000) 

            // fill dowlink payload fields
            dw_phy_payload[0] = dw_mhdr;                                            // m_hdr
            *(uint32_t*)(dw_phy_payload+DW_DEVADDR_POS) = *(uint32_t*)(dw_b0+6);    // dev_addr
            *(dw_phy_payload+DW_FCTRL_POS) = f_ctrl;
            *(uint16_t*)(dw_phy_payload+DW_FCNT_POS) = f_cnt;
            *(dw_phy_payload+DW_FPORT_POS_NOSYNC) = f_port;

            dw_b0[15] = DW_PHY_SIZE_NOSYNC - MIC_SIZE;       // Length
            memcpy(mic_calc_buf, dw_b0, 16);
            memcpy(mic_calc_buf+16, dw_phy_payload, DW_PHY_SIZE_NOSYNC-MIC_SIZE);

            ctx = CMAC_CTX_new();
            CMAC_Init(ctx, key, 16, EVP_aes_128_cbc(), NULL);
            CMAC_Update(ctx, mic_calc_buf, 16+DW_PHY_SIZE_NOSYNC-MIC_SIZE);
            CMAC_Final(ctx, mac, &mac_len);
            *(uint32_t*)(dw_phy_payload+DW_MIC_POS_NOSYNC) = *(uint32_t*)mac;

            // fprintf(stdout, "Resync\n");
            usleep(950000); // should normally take into account the time spent in all the above operations 
            // and send the content of 'out' as a reply
            if((sendto(out_socket_des, dw_pl, 1+MAX_DW_PL, 0, (struct sockaddr *) &client_addr, sizeof(client_addr))) < 0){
                perror("Socket write error");
                pthread_exit((void*) EXIT_FAILURE);
            }
        }
        
        fprintf(stdout, "Payload size: %d\n", n_bytes);

        fprintf(stdout, "%ld.%09ld, %u\n", packt_arrival.tv_sec, packt_arrival.tv_nsec, slot_pos);
        fprintf(time_log, "%ld.%09ld, %u\n", packt_arrival.tv_sec, packt_arrival.tv_nsec, slot_pos);
        fprintf(stdout, "\n");

        memset(pl, '\0', 255);

    }
    
    return 0;
}
