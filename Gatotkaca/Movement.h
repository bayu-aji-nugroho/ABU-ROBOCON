#ifndef MOVEMENT_H
#define MOVEMENT_H

/**
 * @class Movement
 * @brief Mengontrol lokomosi (maju/mundur/rotasi) dan elevator robot.
 */
class Movement {
public:
    Movement();

    // Inisialisasi posisi awal (x, y dalam cm, theta dalam radian)
    void setPose(float x, float y, float theta);

    // Bergerak ke koordinat absolut, timeout ms, return true jika sukses
    bool moveTo(float x, float y, float theta, float speed, int timeout);

    // Bergerak relatif terhadap pose saat ini selama durasi ms
    void moveRelative(float dx, float dy, float dTheta, int duration);

    // Berputar di tempat sebesar angle (rad), kecepatan speed (0-100%), timeout ms
    bool pivot(float angle, float speed, int timeout);

    // Mengatur ketinggian elevator: level 0=Ground, 1=Middle, 2=High
    void setLiftHeight(int level, float speed);

    // Hentikan semua motor seketika
    void stop();

private:
    float currentX, currentY, currentTheta;  // pose aktual (cm, rad)
    int   currentLiftLevel;                  // 0,1,2

    // Fungsi abstraksi hardware (harus diimplementasi sesuai platform)
    void setMotorSpeeds(float leftPercent, float rightPercent);
    void setLiftMotorSpeed(float percent);
    void updatePoseFromEncoders();           // baca encoder, update currentX/Y/Theta
    bool isLiftAtLevel(int level);           // cek limit switch
    unsigned long getCurrentTimeMs();        // waktu dalam milidetik (platform independent)

    // Helper
    float normalizeAngle(float angle);
};

#endif