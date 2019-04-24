/*
 * lora_keys.h
 *
 * Created: 3/7/2019 2:42:55 PM
 *  Author: m50653
 */ 


#ifndef LORA_KEYS_H_
#define LORA_KEYS_H_

#define OTAA              1
//#define ABP             1

#ifdef OTAA

const char my_deveui[] = "00B5BB3C10A7350E";
const char my_appeui[] = "70B3D57ED0013395";
const char my_appkey[] = "ED1803C54B2C98ACFF0F4333B71CD080";

#elif ABP

const char my_devaddr[] = "260513A1";
const char my_nwkskey[] = "34139BE660B61F0964F2CAD3466D91CD";
const char my_appskey[] = "7B9CC260B81E973CBDC00867A07A055F";

#endif

#endif /* LORA_KEYS_H_ */