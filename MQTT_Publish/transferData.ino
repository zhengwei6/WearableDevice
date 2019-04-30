void transferData(){
  //read SD card
  String filename = "TEST"+String(ID)+".csv";
  File transferFile = SD.open(filename);
  if (!transferFile) {
    Serial.println("Can't open "+filename);
    return;
  }
  if (client.publish(mqtt_topic, "#")) {
      Serial.println("Start to transfer data to server");
  }
  else {
      Serial.println("Message failed to send. Reconnecting to MQTT Broker and trying again");
      client.connect(clientID, mqtt_username, mqtt_password);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      client.publish(mqtt_topic, "#");
   }
   int i = 0;
   while(transferFile.available()){
    String temp = transferFile.readStringUntil('\n');
    char buff[100];
    temp.toCharArray(buff,100);
    if(i == 0){
       i = i + 1;
       continue;
    }
    
    if (client.publish(mqtt_topic, buff)) {
    }
    else {
      Serial.println("Message failed to send. Reconnecting to MQTT Broker and trying again");
      client.connect(clientID, mqtt_username, mqtt_password);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      client.publish(mqtt_topic,buff);
    }
   }
  
  if (client.publish(mqtt_topic, "@")) {
      Serial.println("End to transfer data to server");
  }
  else {
      Serial.println("Message failed to send. Reconnecting to MQTT Broker and trying again");
      client.connect(clientID, mqtt_username, mqtt_password);
      delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
      client.publish(mqtt_topic, "@");
   }
   transferFile.close();
}

