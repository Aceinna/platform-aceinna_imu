#ifndef _BASE64_H  
#define _BASE64_H  

#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
  
uint8_t *base64_encode(uint8_t *str);
uint8_t *bae64_decode(uint8_t *code);

#endif
  
