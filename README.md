# Example


```
  //digitalWrite(ONLINE_PIN, connected);
#if ENABLE_GPRS
  //digitalWrite(ONLINE_PIN, client.connected());
  if (!client.connected()) {
    
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 30000L) {
      lastReconnectAttempt = t;
      if (clientConnect()) {
        lastReconnectAttempt = 0;
        connectAttempts=0;
      } else {
        connectAttempts++;
      }

      if(connectAttempts > 3) {
        connectAttempts = 0;
        initGPRS();
      }
    }
  }
  readTimeout = millis();
  while (client.available() && millis() - readTimeout < 10000L) {
    char c = client.read();
    if(!protocol_writeByte(rxbuffer, c)) {
#if PROTO_DEBUG
      DPRINT("No more buffer available");
#endif
      protocol_erase(rxbuffer);
      break;
    }
    readTimeout = millis();
  }
#endif

#if ENABLE_SERIAL_PROTO
  timerFunctions();
  while (SERIAL_PROTO.available() > 0) {
    char c = SERIAL_PROTO.read();
    if(!protocol_writeByte(rxbuffer, c)) {
#if PROTO_DEBUG
      DPRINT("No more buffer available");
#endif
      protocol_erase(rxbuffer);
      break;
    }
    readTimeout = millis();
  }
#endif
  if(rxbuffer.length > 0) {
    protocol_reset(rxbuffer);
    while(protocol_available(rxbuffer, REMOTIC_MIM_PACKET_SIZE)) {
      int readResult = protocol_readMessage(rxbuffer, rxmessage, readMessageId, readMessageType);
#if PROTO_DEBUG
      DPRINT("Read result: ");
      DPRINTLN(readResult, HEX);
#endif
      if(readResult == REMOTIC_READ_SUCCESS) {
        processMessage(rxmessage, readMessageId, readMessageType);
        protocol_removeRead(rxbuffer);
#if PROTO_DEBUG
        protocol_display("rx: ", rxbuffer);
#endif
      } else if(readResult == REMOTIC_READ_INVALID) {
        protocol_erase(rxbuffer);
        break;
      } else if(readResult == REMOTIC_READ_WAIT) {
        if(millis() - readTimeout > 5000) {
#if PROTO_DEBUG
          protocol_display("rx: ", rxbuffer);
#endif
          protocol_erase(rxbuffer);
#if PROTO_DEBUG
          DPRINT("Wait timeout");
#endif
        }
        break;
      }
    }
  }
  if(txbuffer.length > 0) {
    
#if ENABLE_GPRS
    if(client.connected()) {
#if PROTO_DEBUG
      protocol_display("tx: ", txbuffer);
#endif
      client.write(txbuffer.buffer, txbuffer.length);
      client.flush();
      delay(100);
    }
#endif
#if ENABLE_SERIAL_PROTO
    protocol_display("tx: ", txbuffer);
    SERIAL_PROTO.write(txbuffer.buffer, txbuffer.length);
    SERIAL_PROTO.flush();
    delay(100);
#endif
    protocol_erase(txbuffer);
  }
```
