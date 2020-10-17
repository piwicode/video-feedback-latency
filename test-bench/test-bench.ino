#define REFERNCE_CELL_PIN A0


class E2ELantencyReader {
  public:
  struct Config {
    int ref_receiver_pin;    
    int emmiter_pin;
    int photocell_light_latency_ms;
    int photocell_dark_latency_ms;    
  };
  
  E2ELantencyReader(Config config): config_(config) {}
  
  void configure_io() {
    pinMode(config_.ref_receiver_pin, INPUT);
    pinMode(config_.emmiter_pin, OUTPUT);
  }

  // Measure photocell response time to led state change.
  long measure_photocell_response(int value= HIGH, long sampling_period_us = 1000) {
    // Let the photocell discharge and stabilize.
    digitalWrite(config_.emmiter_pin, value == HIGH? LOW : HIGH);
    delay(500);
    // Array where to store datapoints.
    int plot[500];
    // Change the led state and sample analog input.
    digitalWrite(config_.emmiter_pin, value);    
    for(int i = 0 ; i < 500 ; i++) {
      plot[i] = analogRead(config_.ref_receiver_pin);
      delayMicroseconds(sampling_period_us);
    }
    // Print the sampled measure to serial for plotting.
    for(int i = 0 ; i < 500 ; i++) Serial.println(plot[i]);

    // Return he number of sample it takes to reach 90% of the final value.
    const int threshold = (plot[499] * 9 + plot[0] * 1) / 10;
    const bool direction = plot[0] < plot[499];
    for(long i = 0 ; i < 500 ; i++) {
      if( (plot[i] < threshold) != direction ) {       
        return i * sampling_period_us;        
      }
    }
    return -1;
  }

  // Return -1 when timing out.
  int measure_latency_us(int sampling_period_us = 100, unsigned long timeout_ms = 3000) {
    // Calibrate the sensor by measuring low voltage and high voltage.    
    digitalWrite(config_.emmiter_pin, HIGH);
    delay(config_.photocell_light_latency_ms * 2);
    const int high_value = analogRead(config_.ref_receiver_pin);
    // Serial.print("high_value:"); Serial.println(high_value);
    
    digitalWrite(config_.emmiter_pin, LOW);
    delay(config_.photocell_dark_latency_ms * 2);
    const int low_value = analogRead(config_.ref_receiver_pin);        
    // Serial.print("low_value:"); Serial.println(low_value);

    const int threshold = (low_value + high_value) / 2;
    // Serial.print("threshold:"); Serial.println(threshold);    
    unsigned long stop_time = 0, start_time = micros();
    const unsigned long timeout_time = start_time + timeout_ms * 1000;
    digitalWrite(config_.emmiter_pin, HIGH);    
    while(micros()<timeout_time) {
      delayMicroseconds(sampling_period_us); 
      int value = analogRead(config_.ref_receiver_pin);      
      if(value > threshold) {
        stop_time = micros();
        digitalWrite(config_.emmiter_pin, LOW);
        return stop_time - start_time;       
      }
    }
    digitalWrite(config_.emmiter_pin, LOW);
    // Serial.println("timeout");
    digitalWrite(config_.emmiter_pin, LOW);
    return -1;
  }

  private:
  Config config_;
};


E2ELantencyReader latency_reader({
  .ref_receiver_pin = A0,
  .emmiter_pin = 13,
  .photocell_light_latency_ms = 20,
  .photocell_dark_latency_ms = 80
});


void setup() {
  Serial.begin(9600);
  latency_reader.configure_io();
  Serial.print("Response time from dark to light is ");
  Serial.print(latency_reader.measure_photocell_response(HIGH));
  Serial.println("us");
  Serial.print("Response time from light to dark is ");
  Serial.print(latency_reader.measure_photocell_response(LOW));
  Serial.println("us");
}

void loop() {
  //int latency = latency_reader.measure_latency_us();
  //Serial.print("latency:");
  //Serial.println(latency);
}
