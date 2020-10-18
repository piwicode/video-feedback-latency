#include <Arduino.h>

// Configures and runs latency measurements device.
//
// `configure_io` should be called before any other method to initialize the device.
class E2ELantencyReader {
    static constexpr int PLOT_SIZE = 500;

   public:
    struct Config {
        // Hardware pin where the value probe analogic voltage is read from.
        // This photocell points at the screen where the signal ends up being displayed.
        int val_receiver_pin;
        // Hardware pin where the reference probe analogic voltage is read from.
        // This second photocell points is pointing at the singal source. It is used to measure let
        // and photocell activation time and substract it from the the delay measured on the value
        // photocell.
        int ref_receiver_pin;
        // Hardware pin where to write `HIGH` in order to turn on a led and send the signal whose
        // propagation delay is measured.
        int emmiter_pin;
        // Hardware pin that should by `HIGH` to feed the latency measurement control device.
        // This is a workaround for the prototype bacause two voltage inputs are required and
        // the Arduino Uno have only one port. To be removed.
        int feed_pin;
        // Expected photocell latency when going from dark to light. This is used to measure the
        // voltage value of a stabilized high signal when calibrating the probe.
        int photocell_light_latency_ms;
        // Expected photocell latency when going from light to dark. This is used to measure the
        // voltage value of a stabilized low signal when calibrating the probe.
        int photocell_dark_latency_ms;
    };

    E2ELantencyReader(Config config) : config_(config) {}

    // Initializes hardware pin for input and output.
    //
    // To be called one before using the other methods of this class.
    void configure_io() {
        pinMode(config_.ref_receiver_pin, INPUT);
        pinMode(config_.val_receiver_pin, INPUT);
        pinMode(config_.emmiter_pin, OUTPUT);
        pinMode(config_.feed_pin, OUTPUT);
        digitalWrite(config_.feed_pin, HIGH);
    }

    // Prints the value from both probes to the serial interface every `sampling_period_ms`.
    //
    // sampling_period_ms: aproximative delay between two samples. The smaller, the higher the
    // resolution is.
    //
    // The signal led is turned on and off periodically. This method never returns.
    void monitor_forever(long sampling_period_ms = 20) {
        // Output each timeserie name separated wit ha sapce.
        Serial.print("reference_photocell");
        Serial.print(" ");
        Serial.print("signal_photocell");
        Serial.println();
        for (;;) {
            // Get both sensors value and print them to the serial port.
            const int ref = analogRead(config_.ref_receiver_pin);
            const int val = analogRead(config_.val_receiver_pin);
            Serial.print(ref, DEC);
            Serial.print(" ");
            Serial.print(val, DEC);
            Serial.println();
            // Turn the signal let on and off every second.
            digitalWrite(config_.emmiter_pin, (millis() / 1000) & 1 ? LOW : HIGH);
            // Sleep until the next sample has to be captured.
            delayMicroseconds(sampling_period_ms * 1000);
        }
    }

    // Captures photocell response curve to signal and return the and either plot it or write the
    // response time to serial.
    //
    // First the system is stabilized to the idle state for 500ms. Then sample value from both
    // sensors starts to be collected. After `preroll` samples the signal is send and the reponse is
    // collected for `500-preroll` samples.
    //
    // The latency is measured by computing the delay between the the time each sensors cross the
    // 90% threshold between they start value and final value.
    //
    // Arguments:
    // plot: when true, prints the response time series of both sensors to serial for plotting.
    //     Otherwise the measured larency is written as human readable text.
    // value: when `HIGH` the latency of bright signal is detected, if `LOW` the latency of dark
    //     signal is detected.
    // sampling_period_us: time delay within two samples. Only 500 samples ar collected.
    // preroll: number of samples collected before sending the sinal.
    void measure_photocell_response(bool plot, int value = HIGH, long sampling_period_us = 1000,
                                    int preroll = 50) {
        if (plot) {
            Serial.print("reference_photocell");
            Serial.print(" ");
            Serial.print("signal_photocell");
            Serial.println();
            for (int i = 0; i < 10; i++) {
                Serial.println("0 0");
            }
        }
        // Let the photocell discharge and stabilize.
        digitalWrite(config_.emmiter_pin, value == HIGH ? LOW : HIGH);
        delay(500);

        // Array where to store datapoints.
        unsigned char val_plot[PLOT_SIZE] = {0};
        unsigned char ref_plot[PLOT_SIZE] = {0};

        // Record `preroll` samples before emitting the signal.
        for (int i = 0; i < preroll; i++) {
            ref_plot[i] = analogRead(config_.ref_receiver_pin) >> 4;
            val_plot[i] = analogRead(config_.val_receiver_pin) >> 4;
            delayMicroseconds(sampling_period_us);
        }
        // Emmit the signal.
        digitalWrite(config_.emmiter_pin, value);
        // Collect response timeserie.
        for (int i = preroll; i < PLOT_SIZE; i++) {
            ref_plot[i] = analogRead(config_.ref_receiver_pin) >> 4;
            val_plot[i] = analogRead(config_.val_receiver_pin) >> 4;
            delayMicroseconds(sampling_period_us);
        }
        if (plot) {
            // Print the sampled measure to serial for plotting.
            for (int i = 0; i < PLOT_SIZE; i++) {
                Serial.print(ref_plot[i] << 4, DEC);
                Serial.print(" ");
                Serial.print(val_plot[i] << 4, DEC);
                Serial.println();
            }
        } else {
            // Compute the difference between 90% responses time.
            Serial.print("reference latency ");
            Serial.print((sample_to_90(ref_plot) - preroll) * sampling_period_us, DEC);
            Serial.print(" us");
            Serial.print(" value latency ");
            Serial.print((sample_to_90(val_plot) - preroll) * sampling_period_us, DEC);
            Serial.println(" us");
        }
    }
    // Calibrates sensors, send signal and returns the latency in microseconds.
    //
    // First the photocell high and low value are measured. For each sensor the trigger threshold
    // that is considered to be the detection time is set at the middle point between those two
    // values.
    //
    // Then, a HIGH signal is sent and the difference between detection times of the value censor
    // and reference sensor @50% is returned, or -1 if the detection timed out.
    //
    // timeout_ms: time to way for the signal before giving up.
    int measure_latency_us(unsigned long timeout_ms = 3000) {
        // Calibrate the sensor by measuring low voltage and high voltage.
        digitalWrite(config_.emmiter_pin, HIGH);

        delay(config_.photocell_light_latency_ms * 2);
        const int ref_high_value = analogRead(config_.ref_receiver_pin);
        const int val_high_value = analogRead(config_.val_receiver_pin);

        digitalWrite(config_.emmiter_pin, LOW);
        delay(config_.photocell_dark_latency_ms * 2);
        const int ref_low_value = analogRead(config_.ref_receiver_pin);
        const int val_low_value = analogRead(config_.val_receiver_pin);

        // Compute the 50% threshold and display debug information.
        const int ref_threshold = (ref_low_value + ref_high_value) / 2;
        const int val_threshold = (val_low_value + val_high_value) / 2;

        Serial.print("value low:");
        Serial.print(val_low_value);
        Serial.print(" high:");
        Serial.print(val_high_value);
        Serial.print(" threshold:");
        Serial.print(val_threshold);
        Serial.println();
        Serial.print("reference low:");
        Serial.print(ref_low_value);
        Serial.print(" high:");
        Serial.print(ref_high_value);
        Serial.print(" threshold:");
        Serial.print(ref_threshold);
        Serial.println();

        unsigned long ref_stop_time = 0, val_stop_time = 0, start_time = micros();
        const unsigned long timeout_time = start_time + timeout_ms * 1000;
        digitalWrite(config_.emmiter_pin, HIGH);
        // Poll the sensors until both have detected the signal or the timeout is reached.
        while (micros() < timeout_time && (ref_stop_time == 0 || val_stop_time == 0)) {
            int ref_value = analogRead(config_.ref_receiver_pin);
            if (ref_value > ref_threshold) {
                ref_stop_time = micros();
            }
            int val_value = analogRead(config_.val_receiver_pin);
            if (val_value > val_threshold) {
                val_stop_time = micros();
            }
        }
        if (ref_stop_time == 0) {
            Serial.println("reference signal not detected");
            return -1;
        }
        if (val_stop_time == 0) {
            Serial.println("value signal not detected");
            return -1;
        }
        Serial.print("Reference latency ");
        Serial.print(ref_stop_time - start_time);
        Serial.print("us, value latency ");
        Serial.print(val_stop_time - start_time);
        Serial.print("us, latency ");
        const int latency = val_stop_time - ref_stop_time;
        Serial.print(latency);
        Serial.println("us");
        digitalWrite(config_.emmiter_pin, LOW);
        return latency;
    }

   private:
    // Returns the 90% response time of the samples.
    static int sample_to_90(unsigned char plot[PLOT_SIZE]) {
        const int threshold = (plot[PLOT_SIZE - 1] * 9 + plot[0] * 1) / 10;
        const bool direction = plot[0] < plot[PLOT_SIZE - 1];
        for (int i = 0; i < PLOT_SIZE; i++) {
            if ((plot[i] < threshold) != direction) {
                return i;
            }
        }
        return -1;
    }
    Config config_;
};

E2ELantencyReader latency_reader({.val_receiver_pin = A1,
                                  .ref_receiver_pin = A0,
                                  .emmiter_pin = 13,
                                  .feed_pin = A2,
                                  .photocell_light_latency_ms = 600,
                                  .photocell_dark_latency_ms = 600});

void setup() {
    Serial.begin(9600);
    latency_reader.configure_io();
    // latency_reader.monitor_forever();
    // latency_reader.measure_photocell_response(true, HIGH);
}

void loop() {
    delay(2000);
    latency_reader.measure_photocell_response(false, HIGH);
    // latency_reader.measure_latency_us();
    // Serial.println();
    // latency_reader.measure_photocell_response(false, HIGH, 500);
}
