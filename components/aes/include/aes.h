#ifndef COMPONENTS_AES_AES_H_
#define COMPONENTS_AES_AES_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BLOCKSIZE 16
#define xtime(a) (((a)<0x80)?(a)<<1:(((a)<<1)^0x1b) )
// if(a<0x80){a<<=1;}else{a=(a<<1)^0x1b;}
#define SHIFT1                                  (1)
#define SHIFT4                                  (4)


void AESEncodeLoRa(unsigned char* block, unsigned char* key);
void AESDecodeLoRa(unsigned char* block, unsigned char* key);
void AESEncode(uint8_t* block, uint8_t* useKey);
void AESDecode(uint8_t* block, uint8_t* useKey);
void AESCalcDecodeKey(unsigned char* key);
void AESCmac(uint8_t* key, uint8_t* output, uint8_t* input, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_AES_AES_H_ */
