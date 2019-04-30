void transferData(){
  //read SD card
  boolean upload = true;
  if (doFTP(upload)) Serial.println(F("FTP OK"));
  else Serial.println(F("FTP FAIL"));
}

//-------------- FTP receive
byte eRcv() {
  byte respCode;
  byte thisByte;

  while (!client.available()) delay(1);

  respCode = client.peek();

  outCount = 0;

  while (client.available()) {
    thisByte = client.read();
    Serial.write(thisByte);

    if (outCount < 127) {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }

  if (respCode >= '4') {
    Serial.println(F("Command disconnected"));
    return 0;
  }
  return 1;
}  // eRcv()

//--------------- FTP handling
byte doFTP(boolean upload) {
  String filename = "TEST"+String(ID)+".csv";
  File transferFile;
  if (upload) {
    transferFile = SD.open(filename);
  } else {
    
  }
  if (!transferFile) {
    Serial.println(F("SD open fail"));
    return 0;
  }

  if (debug) Serial.println(F("SD opened"));

  if (client.connect(server, 21)) {  // 21 = FTP server
    Serial.println(F("Command connected"));
  } else {
    transferFile.close();
    Serial.println(F("Command connection failed"));
    return 0;
  }

  if (!eRcv()) return 0;
  if (debug) Serial.println("Send USER");
  client.println(F("USER pi"));

  if (!eRcv()) return 0;
  if (debug) Serial.println("Send password");
  client.println(F("PASS raspberry"));

  if (!eRcv()) return 0;
  if (debug) Serial.println("Send SYST");
  client.println(F("SYST"));

  if (!eRcv()) return 0;
  if (debug) Serial.println("Send Type I");
  client.println(F("Type I"));

  if (!eRcv()) return 0;
  if (debug) Serial.println("Send PASV");
  client.println(F("PASV"));

  if (!eRcv()) return 0;

  char *tStr = strtok(outBuf, "(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL, "(,");
    array_pasv[i] = atoi(tStr);
    if (tStr == NULL) {
      Serial.println(F("Bad PASV Answer"));
    }
  }

  unsigned int hiPort, loPort;
  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;

  if (debug) Serial.print(F("Data port: "));
  hiPort = hiPort | loPort;
  if (debug) Serial.println(hiPort);

  if (dclient.connect(server, hiPort)) {
    Serial.println(F("Data connected"));
  }
  else {
    Serial.println(F("Data connection failed"));
    client.stop();
    transferFile.close();
    return 0;
  }

  if (upload) { 
    if (debug) Serial.println("Send STOR filename");
    client.print(F("STOR "));
    client.println(filename);
  }

  if (!eRcv()) {
    dclient.stop();
    return 0;
  }

  if (upload) {
    if (debug) Serial.println(F("Writing"));
    // for faster upload increase buffer size to 1460
//#define bufSizeFTP 64
#define bufSizeFTP 1460
    uint8_t clientBuf[bufSizeFTP];
    //unsigned int clientCount = 0;
    size_t clientCount = 0;
  
    while (transferFile.available()) {
      clientBuf[clientCount] = transferFile.read();
      clientCount++;
      if (clientCount > (bufSizeFTP - 1)) {
        dclient.write((const uint8_t *) &clientBuf[0], bufSizeFTP);
        clientCount = 0;
        delay(1);
      }
    }
    if (clientCount > 0) dclient.write((const uint8_t *) &clientBuf[0], clientCount);

  }

  dclient.stop();
  Serial.println(F("Data disconnected"));

  if (!eRcv()) return 0;
  client.println(F("QUIT"));
  if (!eRcv()) return 0;
  client.stop();
  Serial.println(F("Command disconnected"));

  transferFile.close();

  if (debug) Serial.println(F("SPIFS closed"));
  return 1;
}



