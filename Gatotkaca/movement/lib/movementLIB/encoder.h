#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>

class Encoder{
    private:
        int pinA, pinB;
        float ppr;

        volatile long count = 0;
        long lastCount = 0;
        unsigned long Tlama = 0;
        float RPMsekarang = 0;

        // Static array untuk menyimpan instance pointers
        static Encoder* instances[4];
        static int instanceCount;
        int instanceIndex;

        // Static ISR handlers
        static void IRAM_ATTR ISR0();
        static void IRAM_ATTR ISR1();
        static void IRAM_ATTR ISR2();
        static void IRAM_ATTR ISR3();

    public:
        Encoder(int a, int b, float pulses);
        void begin();

        /* IRAM_ATTR adalah instruksi khusus untuk ESP32 agar fungsi 
           diletakkan di RAM internal (bukan Flash memory). Biar lebih cepat */ 

        void IRAM_ATTR handleInterrupt();
        void update(); // menghitung rpm dari ticks setiap 50ms
        float getRPM() const; // mengoutputkan rpm
};

#endif