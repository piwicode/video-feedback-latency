#define REFERNCE_CELL_PIN A0


class E2ELantencyReader {
  public:
  struct Config {
    int val_receiver_pin;
    int ref_receiver_pin;
    int emmiter_pin;
    int feed_pin;
    int photocell_light_latency_ms;
    int photocell_dark_latency_ms;    
  };
  
  E2ELantencyReader(Config config): config_(config) {}
  
  void configure_io() {
    pinMode(config_.ref_receiver_pin, INPUT);
    pinMode(config_.val_receiver_pin, INPUT);
    pinMode(config_.emmiter_pin, OUTPUT);
    pinMode(config_.feed_pin, OUTPUT);
    digitalWrite(config_.feed_pin, HIGH);
  }

  void monitor_forever(long sampling_period_ms = 10) {
    Serial.print("reference_photocell");
    Serial.print(" ");
    Serial.print("signal_photocell");
    Serial.println();
    for(;;) {
      digitalWrite(config_.emmiter_pin, (millis() / 1000) & 1 ? LOW : HIGH);
      int ref = analogRead(config_.ref_receiver_pin);
      int val = analogRead(config_.val_receiver_pin);
      Serial.print(ref, DEC);
      Serial.print(" ");
      Serial.print(val, DEC);
      Serial.println();
      delayMicroseconds(sampling_period_ms * 2000);
    }
  }
  
  static int sample_to_90(unsigned char plot[500]) {
    const int threshold = (plot[499] * 9 + plot[0] * 1) / 10;
    const bool direction = plot[0] < plot[499];
    for(int i = 0 ; i < 500 ; i++) {
      if( (plot[i] < threshold) != direction ) {       
       return i;
      }
    }
    return -1;
  }
  // Measure photocell response time to led state change.
  long measure_photocell_response(bool plot, int value= HIGH, long sampling_period_us = 1000, int preroll = 50) {
    if(plot) {
      Serial.print("reference_photocell");
      Serial.print(" ");
      Serial.print("signal_photocell");
      Serial.println();
      for(int i = 0 ; i < 10 ; i++) {
        Serial.println("0 0");
      }
    }
    // Let the photocell discharge and stabilize.
    digitalWrite(config_.emmiter_pin, value == HIGH? LOW : HIGH);
    delay(500);
    // Array where to store datapoints.
    unsigned char val_plot[500] = {0};
    unsigned char ref_plot[500] = {0};
    
    // Change the led state and sample analog input.
    for(int i = 0 ; i < preroll ; i++) {
      ref_plot[i] = analogRead(config_.ref_receiver_pin) >> 4;
      val_plot[i] = analogRead(config_.val_receiver_pin) >> 4;
      delayMicroseconds(sampling_period_us);
    }
    digitalWrite(config_.emmiter_pin, value);    
    for(int i = preroll ; i < 500 ; i++) {
      ref_plot[i] = analogRead(config_.ref_receiver_pin) >> 4;
      val_plot[i] = analogRead(config_.val_receiver_pin) >> 4;
      delayMicroseconds(sampling_period_us);
    }
    if(plot) {
      // Print the sampled measure to serial for plotting.
      for(int i = 0 ; i < 500 ; i++) {
        Serial.print(ref_plot[i] << 4, DEC);
        Serial.print(" ");
        Serial.print(val_plot[i] << 4, DEC);
        Serial.println();
      }
    } else {
      Serial.print("reference latency ");
      Serial.print((sample_to_90(ref_plot)  - preroll)* sampling_period_us, DEC);
      Serial.print(" us");
      Serial.print(" value latency ");
      Serial.print((sample_to_90(val_plot) - preroll)* sampling_period_us, DEC);
      Serial.println(" us");
    }
    return -1;
  }

  // Return -1 when timing out.
  int measure_latency_us(int sampling_period_us = 100, unsigned long timeout_ms = 3000) {
    digitalWrite(config_.feed_pin, HIGH);
    // Calibrate the sensor by measuring low voltage and high voltage.    
    digitalWrite(config_.emmiter_pin, HIGH);
    delay(config_.photocell_light_latency_ms * 2);
    const int ref_high_value = analogRead(config_.ref_receiver_pin);
    const int val_high_value = analogRead(config_.val_receiver_pin);
    // Serial.print("high_value:"); Serial.println(high_value);
    
    digitalWrite(config_.emmiter_pin, LOW);
    delay(config_.photocell_dark_latency_ms * 2);
    const int ref_low_value = analogRead(config_.ref_receiver_pin);
    const int val_low_value = analogRead(config_.val_receiver_pin);   

    const int ref_threshold = (ref_low_value + ref_high_value) / 2;
    const int val_threshold = (val_low_value + val_high_value) / 2;
    
    Serial.print("value low:");Serial.print(val_low_value);
    Serial.print(" high:");Serial.print(val_high_value);
    Serial.print(" threshold:");Serial.print(val_threshold);
    Serial.println();
    Serial.print("reference low:");Serial.print(ref_low_value);
    Serial.print(" high:");Serial.print(ref_high_value);
    Serial.print(" threshold:");Serial.print(ref_threshold);
    Serial.println();
    unsigned long ref_stop_time = 0, val_stop_time = 0, start_time = micros();
    const unsigned long timeout_time = start_time + timeout_ms * 1000;
    digitalWrite(config_.emmiter_pin, HIGH);
    while(micros() < timeout_time && (ref_stop_time == 0 || val_stop_time == 0)) {
      //delayMicroseconds(sampling_period_us);
      
      int ref_value = analogRead(config_.ref_receiver_pin);
      if(ref_value > ref_threshold) {
        ref_stop_time = micros();
      }
      int val_value = analogRead(config_.val_receiver_pin);
      if(val_value > val_threshold) {
        val_stop_time = micros();
      }
    }
    if(ref_stop_time == 0) {
      Serial.println("reference signal not detected");
      return -1;
    }
    if(val_stop_time == 0) {
      Serial.println("value signal not detected");
      return -1;
    }
    Serial.print("Reference latency ");
    Serial.print(ref_stop_time - start_time);
    Serial.print("us, value latency ");
    Serial.print(val_stop_time - start_time);
    Serial.print("us, latency ");
    int latency = val_stop_time - ref_stop_time;
    Serial.print(latency);    
    Serial.println("us");    
    digitalWrite(config_.emmiter_pin, LOW);
    return latency;
  }

  private:
  Config config_;
};


E2ELantencyReader latency_reader({
  .val_receiver_pin = A1,
  .ref_receiver_pin = A0,  
  .emmiter_pin = 13,
  .feed_pin = A2,
  .photocell_light_latency_ms = 600,
  .photocell_dark_latency_ms = 600  
});


void setup() {
  Serial.begin(9600);
  latency_reader.configure_io();
  //latency_reader.monitor_forever();
  // latency_reader.measure_photocell_response(true, HIGH);
  
}

void loop() {
  delay(2000); 
  latency_reader.measure_photocell_response(false, HIGH);
  //latency_reader.measure_latency_us();
  //Serial.println();
  //latency_reader.measure_photocell_response(false, HIGH, 500);
}
