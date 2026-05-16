#ifndef AI_H
#define AI_H

#include <vector>
#include <utility> // for std::pair

/**
 * @struct KFSObject
 * @brief Representasi objek KFS yang terdeteksi
 */
struct KFSObject {
    int id;                // ID unik objek
    float x, y;            // koordinat dalam cm
    int zoneID;            // zona tempat objek berada
    bool isValid;          // apakah masih terdeteksi
};

/**
 * @class AI
 * @brief Mengelola deteksi objek, halangan, dan pelacakan target.
 */
class AI {
public:
    AI();

    /**
     * @brief Mencari objek KFS di zona tertentu.
     * @param zoneID ID zona (0-...)
     * @return true jika ditemukan setidaknya satu objek di zona tersebut.
     */
    bool scanKFS(int zoneID);

    /**
     * @brief Mendapatkan koordinat objek berdasarkan ID.
     * @param objectID ID objek.
     * @return std::pair<float,float> (x,y) dalam cm, atau (-1,-1) jika tidak ada.
     */
    std::pair<float,float> getKFSCoordinate(int objectID);

    /**
     * @brief Cek apakah path di depan robot bersih hingga jarak tertentu.
     * @param distance Jarak dalam cm.
     * @return true jika tidak ada halangan, false jika ada.
     */
    bool isPathClear(float distance);

    /**
     * @brief Memposisikan robot agar presisi di depan objek (alignment).
     * @param objectID ID objek target.
     * @param tolerance Toleransi jarak dalam cm.
     * @return true jika alignment berhasil.
     */
    bool alignToObject(int objectID, float tolerance);

    /**
     * @brief Mengunci target agar sensor tetap mengarah ke objek saat robot bergerak.
     * @param objectID ID objek yang dilacak.
     * @return true jika tracking aktif.
     */
    bool trackTarget(int objectID);

    // Untuk keperluan simulasi / debugging
    void addFakeKFS(int id, float x, float y, int zone);

private:
    std::vector<KFSObject> detectedObjects; // daftar objek yang terdeteksi
    int currentTrackedID;                  // ID objek yang sedang dilacak

    // Simulasi sensor (harus diganti dengan hardware nyata)
    bool virtualScan(int zoneID, std::vector<KFSObject>& results);
    float readFrontUltrasonic();            // jarak depan dalam cm
};

#endif