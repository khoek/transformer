#include <Arduino.h>
#include <avr/wdt.h>

#define PIN_SSR 2
#define PIN_LED 13
#define PIN_WAVEFORM A5

#define WINDOW_COUNT 600
// Note: A rule of thumb for below is:
//          1 + (WINDOW_COUNT * SAMPLE_SPACING_US microseconds * 60Hz * 2)
#define MAX_INTERVALS 20

#define SAMPLE_SPACING_US 220

int window[WINDOW_COUNT];

struct interval {
    int start;
    int end;

    int center;
};

int interval_count = 0;
struct interval intervals[MAX_INTERVALS];

void fail(const char *reason) {
    Serial.println(reason);

    // Hard rest after 250ms.
    wdt_enable(WDTO_250MS);
    while (1)
        ;
}

int inline __attribute__((always_inline)) read_waveform() {
    return analogRead(PIN_WAVEFORM);
}

#define NULL_SENSE_THRESHOLD 10
#define NULL_SHUTDOWN_COUNT 10

void inline __attribute__((always_inline)) hold_until_nonnull(unsigned long neccesary_duration_millis) {
    Serial.println("HOLD START");

hold_until_nonnull_retry:
    unsigned long capture_start_millis = millis();

    int null_contiguous = 0;
    while (millis() - capture_start_millis < neccesary_duration_millis) {
        if (read_waveform() < NULL_SENSE_THRESHOLD) {
            null_contiguous++;

            if (null_contiguous > NULL_SHUTDOWN_COUNT) {
                Serial.println("HOLD RETRY");
                goto hold_until_nonnull_retry;
            }
        } else {
            null_contiguous = 0;
        }
    }
}

void inline __attribute__((always_inline)) trigger() {
    digitalWrite(PIN_SSR, true);
    digitalWrite(PIN_LED, true);

    Serial.println("TRIGGER");

    int null_contiguous = 0;
    while (1) {
        if (read_waveform() < NULL_SENSE_THRESHOLD) {
            null_contiguous++;

            if (null_contiguous > NULL_SHUTDOWN_COUNT) {
                digitalWrite(PIN_SSR, false);
                digitalWrite(PIN_LED, false);

                fail("SHUTDOWN");
            }
        } else {
            null_contiguous = 0;
        }
    }
}

#define MIN_BUFFER_US 5

double capture_window() {
    long capture_start_time = micros();

    long next_capture = capture_start_time + SAMPLE_SPACING_US;
    for (int i = 0; i < WINDOW_COUNT; i++) {
        long now = micros();
        // Note: This is robust to `micros()` overflow.
        if (next_capture - now < MIN_BUFFER_US) {
            Serial.println(i);
            Serial.println(next_capture - now);
            Serial.println(MIN_BUFFER_US);
            Serial.println(SAMPLE_SPACING_US);
            fail("capture window missed!");
        }

        // Note: This is robust to `micros()` overflow.
        delayMicroseconds(next_capture - micros());

        window[i] = read_waveform();
        next_capture += SAMPLE_SPACING_US;
    }

    return SAMPLE_SPACING_US;
}

#define PEAK_SAMPLES 20
#define UNACCEPTABLE_JUMP_THRESHOLD 25

int calc_peak_amplitude() {
    if (WINDOW_COUNT < PEAK_SAMPLES) {
        fail("window too small for peak samples");
    }

    // Wait until we actually detect a signal, since if we have just reset from
    // a temporary power cycle then we don't want to capture as soon as power comes back,
    // because there will likely be inital switch bounce (hence the `delay()` after this).
    while (read_waveform() < NULL_SENSE_THRESHOLD)
        ;

    delay(10);

    int seen[PEAK_SAMPLES];
    int sum = 0;
    int last_greatest = -1;
    for (int i = 0; i < PEAK_SAMPLES; i++) {
        int greatest = -1;
        int greatest_idx = 0;
        for (int j = 0; j < WINDOW_COUNT; j++) {
            bool seen_already = false;
            for (int k = 0; k < i; k++) {
                if (seen[k] == j) {
                    seen_already = true;
                    break;
                }
            }

            if (seen_already) {
                continue;
            }

            if (window[j] > greatest) {
                greatest = window[j];
                greatest_idx = j;
            }
        }

        if (last_greatest >= 0) {
            if (last_greatest - greatest > UNACCEPTABLE_JUMP_THRESHOLD) {
                Serial.println(last_greatest);
                Serial.println(greatest);
                Serial.println(UNACCEPTABLE_JUMP_THRESHOLD);
                fail("unacceptable jump");
            }
        }

        seen[i] = greatest_idx;
        last_greatest = greatest;
        sum += greatest;
    }

    return sum / PEAK_SAMPLES;
}

// Note: detect peak too low (something funky happening, or)
// peak has clipped, in which case we may not be able to accurately
// compute the period.
#define PEAK_MIN 200
#define PEAK_MAX 1024

void validate_peak_amplitude(int peak) {
    if (peak < PEAK_MIN || peak == PEAK_MAX) {
        Serial.println(peak);
        fail("peak amplitude out of bounds");
    }
}

#define INTERVAL_START_MIN_IDX 3
#define INTERVAL_THRESHOLD_DIVISOR 20
#define MIN_INTERVALS 8

int find_peak_threshold_and_intervals(int peak) {
    int threshold = peak - (peak / INTERVAL_THRESHOLD_DIVISOR);

    int interval_start = -1;
    for (int i = 0; i < WINDOW_COUNT; i++) {
        if (interval_start < 0 && threshold <= window[i]) {
            interval_start = i;
        }

        // Note: We don't attempt to handle a noisy measurement here.
        if (interval_start >= 0 && threshold > window[i]) {
            if (interval_count == MAX_INTERVALS) {
                fail("interval count overflow");
            }

            // Note: We intentionally skip intervals which started at the very beginning of our sample,
            // since we cannot be sure when they actually started in time. Moreover, we don't record
            // a final interval if one is currently matched at the time we run out of samples.
            if (interval_start >= INTERVAL_START_MIN_IDX) {
                intervals[interval_count].start = interval_start;
                intervals[interval_count].end = i;
                intervals[interval_count].center = (i + interval_start) / 2;

                interval_count++;
            }

            interval_start = -1;
        }
    }

    // Note: We forget the last interval if it has been truncated, since failure
    // to do this will mess up interval width/center hence period calculations.

    for (int i = interval_count; i < MAX_INTERVALS; i++) {
        intervals[i].start = -1;
        intervals[i].end = -1;
    }

    if (interval_count < MIN_INTERVALS) {
        Serial.println(interval_count);
        fail("not enough intervals");
    }

    return threshold;
}

void calc_mean_interval_width_and_period(double *mean_width, double *mean_period) {
    int sum;

    sum = 0;
    for (int i = 0; i < interval_count; i++) {
        sum += intervals[i].end - intervals[i].start;
    }
    *mean_width = ((double) sum) / ((double) interval_count);

    sum = 0;
    for (int i = 0; i < interval_count - 1; i++) {
        sum += intervals[i + 1].center - intervals[i].center;
    }
    *mean_period = ((double) sum) / ((double) (interval_count - 1));
}

// FIXME calibrate for 115V
#define INTWIDTH_MIN_US 900
#define INTWIDTH_MAX_US 1800

#define INTWIDTH_MAX_SAMPLE_DEV 2

// Note: mains is rectified, so we expect 100Hz or 120Hz.
// 7ms  = 130Hz
#define PERIOD_MIN_US 7000
// 12ms =  90Hz
#define PERIOD_MAX_US 12000

void validate_interval_width_and_period(double intwidth_samples, double intwidth_us, double period_us) {
    if (intwidth_us < ((double) INTWIDTH_MIN_US) || intwidth_us > ((double) INTWIDTH_MAX_US)) {
        Serial.println(intwidth_us);
        fail("intwidth out of bounds");
    }

    for (int i = 0; i < interval_count; i++) {
        if (abs(intwidth_samples - ((double) (intervals[i].end - intervals[i].start))) > INTWIDTH_MAX_SAMPLE_DEV) {
            Serial.println(intwidth_samples);
            Serial.println(intervals[i].start);
            Serial.println(intervals[i].end);
            Serial.println(abs(intwidth_samples - ((double) (intervals[i].end - intervals[i].start))));
            fail("interval deviation exceeded");
        }
    }

    if (period_us < ((double) PERIOD_MIN_US) || period_us > ((double) PERIOD_MAX_US)) {
        Serial.println(period_us);
        fail("period out of bounds");
    }
}

#define PHASE_DELAY_MULTIPLIER 0.136

void synchronize_to_peak(int threshold, double us_per_sample, double intwidth_us, double period_us) {
    int taken = 0;

    // Wait until we first enter the peak thresh...
    while (read_waveform() < threshold && taken < WINDOW_COUNT) {
        taken++;
    }

    if (((double) taken) * us_per_sample > 2 * period_us) {
        Serial.println(((double) taken) * us_per_sample);
        Serial.println(period_us);
        fail("sync1: taken too long");
    }

    taken = 0;
    while (read_waveform() >= threshold && taken < WINDOW_COUNT) {
        taken++;
    }

    if (((double) taken) * us_per_sample > 2 * period_us) {
        Serial.println(((double) taken) * us_per_sample);
        Serial.println(period_us);
        fail("sync2: taken too long");
    }

    taken = 0;
    while (read_waveform() < threshold && taken < WINDOW_COUNT) {
        taken++;
    }

    if (((double) taken) * us_per_sample > 2 * period_us) {
        Serial.println(((double) taken) * us_per_sample);
        Serial.println(period_us);
        fail("sync3: taken too long");
    }

    delayMicroseconds((int) ((intwidth_us / 2.) + (PHASE_DELAY_MULTIPLIER * period_us)));
    trigger();
}

void setup() {
    // DEBUG
    Serial.begin(9600);

    pinMode(PIN_SSR, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_WAVEFORM, INPUT);

    digitalWrite(PIN_SSR, false);
    digitalWrite(PIN_LED, false);
    pinMode(PIN_SSR, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_SSR, false);
    digitalWrite(PIN_LED, false);
}

#define HOLD_OFF_SECONDS 5

void loop() {
    // 10 second hold off, during which AC must be present
    hold_until_nonnull(HOLD_OFF_SECONDS * 1000);

    double us_per_sample = capture_window();

    Serial.print("us per sample: ");
    Serial.print(us_per_sample);
    Serial.println();

    int peak = calc_peak_amplitude();
    validate_peak_amplitude(peak);
    int threshold = find_peak_threshold_and_intervals(peak);

    Serial.print("peak=");
    Serial.print(peak);
    Serial.print(", threshold=");
    Serial.print(threshold);
    Serial.print(", interval count=");
    Serial.print(interval_count);
    Serial.println();

    double mean_width;
    double mean_period;
    calc_mean_interval_width_and_period(&mean_width, &mean_period);

    double intwidth_us = mean_width * us_per_sample;
    double period_us = mean_period * us_per_sample;

    Serial.print("width: ");
    Serial.print(mean_width);
    Serial.print(" - ");
    Serial.print((int) intwidth_us);
    Serial.println();
    Serial.print("period: ");
    Serial.print(mean_period);
    Serial.print(" - ");
    Serial.print((int) period_us);
    Serial.println();

    validate_interval_width_and_period(mean_width, intwidth_us, period_us);
    synchronize_to_peak(threshold, us_per_sample, intwidth_us, period_us);

    fail("unreachable");
}