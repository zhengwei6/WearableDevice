void initial() {
  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // initial serial
  Serial.begin(9600);
  
  // initial WiFi 
  WiFinitial();
  
  // Read EEPROM
  ID = EEPROMinitial();
  String filename = "TEST"+String(ID)+".csv";
  Serial.print("Test file:");
  Serial.println(filename);
  
  // initial SD
  SDinitial();
  delay(100);

  // open File
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {                                   
       Serial.println("Write to "+filename+"...");         
       myFile.close();                               
  } else {
       Serial.println("\n open file error ");
       return;    
  }

  //Buttom
  pinMode(D3, INPUT_PULLUP);

  //MPU6050
  MPU6050initial();
  
}

void WiFinitial(){
  WiFi.begin(ssid,password);
  Serial.print("Connect WiFi");
  while(WiFi.status()!= WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("WiFi is connected");
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  while(!client.connect(clientID, mqtt_username, mqtt_password)){
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to MQTT Broker!");
  return;
}

int EEPROMinitial(){
  int eeAddress = 0;
  int value = 0;
  EEPROM.begin(512);
  EEPROM.get(eeAddress, value);
  if (value == 255){
      value = 0;
  }
  ID = value + 1;
  EEPROM.put(eeAddress, ID);
  EEPROM.commit();
  return ID;
}

void SDinitial(){
  while (!SD.begin(SDCS_PIN)) {
       delay(500);
       Serial.println("SD Fail!");
  }
  Serial.println("SD is initial");
  return;
}

void MPU6050initial(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  delay(100);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-3842);
  mpu.setYAccelOffset(59);
  mpu.setZAccelOffset(1295);
  mpu.setXGyroOffset(-29);
  mpu.setYGyroOffset(25);
  mpu.setZGyroOffset(52);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
   } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
   }
}


