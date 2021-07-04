
#include "CRC16.h"


typedef unsigned char uint8;
typedef unsigned int uint32;
typedef unsigned long uint64;

#define REMOTIC_LITTLE_ENDIAN 1

#define REMOTIC_READ_SUCCESS 0
#define REMOTIC_READ_WAIT 1
#define REMOTIC_READ_INVALID 2

#define REMOTIC_MIM_PACKET_SIZE 9

#define REMOTIC_MESSAGE_EMPTY 0x00
#define REMOTIC_MESSAGE_HELLO 0x01
#define REMOTIC_MESSAGE_STATUS 0x02
#define REMOTIC_MESSAGE_SET 0x03
#define REMOTIC_MESSAGE_READY 0x04
#define REMOTIC_MESSAGE_ERROR 0x05
#define REMOTIC_MESSAGE_SERVER_READY 0x06
#define REMOTIC_MESSAGE_ACK 0x07
//#define REMOTIC_MESSAGE_INPUT_MAP 0x08
//#define REMOTIC_MESSAGE_INPUT_MAP_LOW 0x09
#define REMOTIC_MESSAGE_MODBUS 0x0A
//#define REMOTIC_MESSAGE_READ_INPUT_MAP 0x0B
#define REMOTIC_MESSAGE_GET_SETTINGS 0x0C
#define REMOTIC_MESSAGE_SET_SETTINGS 0x0D
#define REMOTIC_MESSAGE_AUTOMATION 0x0E
#define REMOTIC_MESSAGE_TIMER 0x0F
#define REMOTIC_MESSAGE_INPUT 0x10

struct Buffer {
  char buffer[PROTOCOL_MAX_BUFFER];
  int length = 0;
  int position = 0;
};

static CRC16 crc;

uint64 floatToInt(float f) {
    float normalized;
    int16_t shift;
    int32_t sign, exponent, significand;

    if (f == 0.0)
        return 0; //handle this special case
    //check sign and begin normalization
    if (f < 0) {
        sign = 1;
        normalized = -f;
    } else {
        sign = 0;
        normalized = f;
    }
    //get normalized form of f and track the exponent
    shift = 0;
    while (normalized >= 2.0) {
        normalized /= 2.0;
        shift++;
    }
    while (normalized < 1.0) {
        normalized *= 2.0;
        shift--;
    }
    normalized = normalized - 1.0;
    //calculate binary form (non-float) of significand
    significand = normalized*(0x800000 + 0.5f);
    //get biased exponent
    exponent = shift + 0x7f; //shift + bias
    //combine and return
    return (sign<<31) | (exponent<<23) | significand;
}
#if PROTO_DEBUG
void protocol_display(char * title, Buffer & payload) {
    DPRINT(title);
    DPRINT(" ");
    DPRINT("{len=");
    DPRINT(payload.length);
    DPRINT("} ");
    DPRINT("{pos=");
    DPRINT(payload.position);
    DPRINT("} ");
    if(payload.length) {
      for(int i = 0; i < payload.length; i++) {
        if( i == payload.position ) {
          DPRINT(">");
        }
        if((unsigned char)payload.buffer[i] <= 0xF) {
          DPRINT("0");
        }
        DPRINT((unsigned char)payload.buffer[i], HEX);
        DPRINT(" ");
      }
    }
    DPRINTLN();
}
#endif
// Protocol
bool protocol_available(Buffer & buffer, int size) {
  return buffer.position + size <= buffer.length;
}
bool protocol_free(Buffer & buffer, int size) {
  return buffer.length + size <= PROTOCOL_MAX_BUFFER;
}
void protocol_reset(Buffer & buffer) {
  buffer.position = 0;
  
}
void protocol_erase(Buffer & buffer) {
  buffer.length = 0;
  buffer.position = 0;
}
void protocol_removeRead(Buffer & buffer) {
  for(int i = buffer.position; i < buffer.length; i++) {
    buffer.buffer[i - buffer.position] = buffer.buffer[i];
  }
  buffer.length = buffer.length - buffer.position;
  buffer.position = 0;
}
bool protocol_writeByte(Buffer & buffer, byte n) {
  if(protocol_free(buffer, 1)) {
    buffer.buffer[buffer.length++] = n;
    return true;
  }
  return false;
}
bool protocol_writeNumber(Buffer & buffer, int number, int size = 1) {
  if(protocol_free(buffer, size)) {
    for (int i = 0; i < size; i++) {
#ifdef REMOTIC_LITTLE_ENDIAN
      buffer.buffer[buffer.length++] = (uint8) ((number >> (i * 8)) & 0xFF);
#else
      buffer.buffer[buffer.length++] = (uint8) ((number >> ((size - i - 1) * 8)) & 0xFF);
#endif
    }
    return true;
  }
  return false;
}

bool protocol_writeFloat(Buffer & buffer, float f) {
  if(!protocol_free(buffer, 4)) {
    return false;
  }
  uint64 value = floatToInt(f);
  buffer.buffer[buffer.length++] = (value >> 8 * 0) & 0xFF;
  buffer.buffer[buffer.length++] = (value >> 8 * 1) & 0xFF;
  buffer.buffer[buffer.length++] = (value >> 8 * 2) & 0xFF;
  buffer.buffer[buffer.length++] = (value >> 8 * 3) & 0xFF;
  return true;
}



bool protocol_writeString(Buffer & buffer, char * string, int size) {
  if(protocol_free(buffer, 2 + size)) {
    protocol_writeNumber(buffer, size, 2);
    for (int i = 0; i < size; i++) {
      buffer.buffer[buffer.length++] = string[i];
    }
    return true;
  }
  return false;
}


unsigned int protocol_Hash(Buffer & buffer) {
  crc.init();
  return crc.processBuffer(( char *) buffer.buffer, buffer.length);
}

bool protocol_writeString(Buffer & buffer, char * string) {
  return protocol_writeString(buffer, string, strlen(string));
}

bool protocol_writeMessage(Buffer & payload, Buffer & dst, int messageId, int messageType) {
  if(!protocol_free(dst, payload.length + 3)) {
    return false;
  }
  protocol_writeByte(dst, 0xA5);
  protocol_writeNumber(dst, messageId, 2);
  protocol_writeByte(dst, messageType);
  protocol_writeNumber(dst, payload.length, 2);
  for(int i = 0; i < payload.length; i++) {
    dst.buffer[dst.length++] = payload.buffer[i];
  }
  protocol_writeNumber(dst, protocol_Hash(payload), 2);
  protocol_writeByte(dst, 0xA5);
  return true;
}


byte protocol_readByte(Buffer & buffer) {
  if(!protocol_available(buffer, 1)) return 0;
  return buffer.buffer[buffer.position++];
}
uint64 protocol_readNumber(Buffer & buffer, int size) {
    if(!protocol_available(buffer, size)) return 0;
    uint64 number = 0;
    for (unsigned int i = 0; i < size; i++) {
#ifdef REMOTIC_LITTLE_ENDIAN
        number += (uint64) ((buffer.buffer[buffer.position++] & 0xFF)) << (i * 8);
#else
        number += (uint64) ((buffer.buffer[buffer.position++] & 0xFF)) << ((size - i - 1) * 8);
#endif
    }
    return number;
}

int protocol_read(Buffer & src, char * dst, int size) {
  int read = 0;
  for(read = 0; read < size; read++) {
    dst[read] = src.buffer[src.position++];
  }
  return read;
}
int protocol_write(Buffer & dst, char * src, int size) {
  for(int i = 0; i < size; i++) {
    dst.buffer[dst.length++] = src[i];
  }
  return size;
}

int protocol_copyData(Buffer & src, Buffer & dst, int size) {
  for(int i = 0; i < size; i++) {
    dst.buffer[dst.length++] = src.buffer[src.position++];
  }
  return size;
}

float protocol_readFloat(Buffer & buffer) {
  if(!protocol_available(buffer, 4)) return 0;
  uint32_t b32 = protocol_readNumber(buffer, 4);
#if PROTO_DEBUG
    DPRINT("Read float: ");
    DPRINT(b32);
    DPRINTLN();
#endif
  float result;
  int32_t shift;
  uint16_t bias;

  if (b32 == 0)
      return 0.0;
  //pull significand
  result = (b32&0x7fffff); //mask significand
  result /= (0x800000);    //convert back to float
  result += 1.0f;          //add one back
  //deal with the exponent
  bias = 0x7f;
  shift = ((b32>>23)&0xff) - bias;
  while (shift > 0) {
      result *= 2.0;
      shift--;
  }
  while (shift < 0) {
      result /= 2.0;
      shift++;
  }
  //sign
  result *= (b32>>31)&1 ? -1.0 : 1.0;
  return result;
}


int protocol_readMessage(Buffer & message, Buffer & payload, int &messageId, int &messageType) {
  protocol_erase(payload);
  if(protocol_readByte(message) != 0xA5) {
#if PROTO_DEBUG
    DPRINTLN("Invalid start byte");
#endif
    return REMOTIC_READ_INVALID;
  }
  messageId = protocol_readNumber(message, 2);
  messageType = protocol_readByte(message);
  int length = protocol_readNumber(message, 2);
  if(!protocol_available(message, length + 3)) { // + 3: 2 Hash + 1 Stop
    return REMOTIC_READ_WAIT;
  }
  protocol_copyData(message, payload, length);
#if PROTO_DEBUG
  protocol_display("payload: ", payload);
#endif
  int hash = protocol_readNumber(message, 2);
  if(hash != protocol_Hash(payload)) {
#if PROTO_DEBUG
    DPRINT("Invalid hash: ");
    DPRINT(hash, HEX);
    DPRINT(" != ");
    DPRINT(protocol_Hash(payload), HEX);
    DPRINTLN("");
#endif
    return REMOTIC_READ_INVALID;
  }
  if(protocol_readByte(message) != 0xA5) {
#if PROTO_DEBUG
#endif
    DPRINTLN("Invalid stop byte");
    return REMOTIC_READ_INVALID;
  }
  return REMOTIC_READ_SUCCESS;
}
