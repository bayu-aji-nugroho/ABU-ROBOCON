#ifndef TOOLS_H
#define TOOLS_H

/**
 * @class Tools
 * @brief Mengendalikan mekanisme pengambilan/pelepasan KFS dan tool lainnya.
 */
class Tools {
public:
    Tools();

    /**
     * @brief Mengambil KFS (atau tombak) dengan tekanan tertentu.
     * @param pressure Persen tekanan (0-100) atau nilai analog.
     * @param duration Durasi gerakan grab dalam ms.
     * @return true jika berhasil mengambil.
     */
    bool grabKFS(float pressure, int duration);

    /**
     * @brief Melepaskan KFS (menempatkan atau menembak).
     * @param duration Durasi gerakan release dalam ms.
     * @return true jika berhasil melepas.
     */
    bool releaseKFS(int duration);

    /**
     * @brief Mengaktifkan tool spesifik (misal arm extender atau stabilizer).
     * @param toolID ID tool (0: extender, 1: stabilizer, dll).
     * @return true jika tool berhasil diaktifkan.
     */
    bool deployTool(int toolID);

    /**
     * @brief Mengembalikan semua mekanisme ke posisi default/home.
     */
    void resetMechanism();

    /**
     * @brief Memvalidasi keberadaan objek menggunakan sensor internal (limit switch/IR).
     * @param sensorID ID sensor (0: gripper, 1: lift, dll).
     * @return true jika objek terdeteksi.
     */
    bool checkSensor(int sensorID);

private:
    bool isGripperClosed;    // status gripper
    bool isToolDeployed[2];  // status tool (maks 2 untuk contoh)

    // Fungsi abstraksi hardware (ganti sesuai platform)
    void setGripperMotor(float speedPercent);    // speed positif = close, negatif = open
    void setToolActuator(int toolID, bool activate);
    void homeAllActuators();
    bool readSensorValue(int sensorID);          // baca limit switch/IR
};

#endif