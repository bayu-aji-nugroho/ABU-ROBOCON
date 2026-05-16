#include "AI.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>

AI::AI() : currentTrackedID(-1) {}

bool AI::scanKFS(int zoneID) {
    std::vector<KFSObject> newObjects;
    if (virtualScan(zoneID, newObjects)) {
        // Merge atau update detectedObjects
        for (auto& obj : newObjects) {
            auto it = std::find_if(detectedObjects.begin(), detectedObjects.end(),
                [&obj](const KFSObject& o) { return o.id == obj.id; });
            if (it != detectedObjects.end()) {
                it->x = obj.x;
                it->y = obj.y;
                it->isValid = true;
            } else {
                detectedObjects.push_back(obj);
            }
        }
        // Tandai objek yang tidak terdeteksi ulang sebagai tidak valid? Bisa sesuai kebutuhan
        return !newObjects.empty();
    }
    return false;
}

std::pair<float,float> AI::getKFSCoordinate(int objectID) {
    for (const auto& obj : detectedObjects) {
        if (obj.id == objectID && obj.isValid) {
            return {obj.x, obj.y};
        }
    }
    return {-1, -1};
}

bool AI::isPathClear(float distance) {
    float frontDist = readFrontUltrasonic();
    return (frontDist > distance);
}

bool AI::alignToObject(int objectID, float tolerance) {
    // Asumsikan robot memiliki kemampuan untuk menggerakkan dirinya relatif terhadap objek.
    // Di sini kita hanya simulasi: dapatkan koordinat objek, lalu bandingkan dengan pose robot.
    // Karena Movement class tidak diinclude di sini (agar decoupling), kita asumsikan ada antarmuka.
    // Untuk contoh, kita gunakan dummy: selalu berhasil jika objek valid.
    for (const auto& obj : detectedObjects) {
        if (obj.id == objectID && obj.isValid) {
            // Simulasi: hitung jarak antara robot (0,0) dan objek (x,y)
            float dist = std::sqrt(obj.x*obj.x + obj.y*obj.y);
            return (dist < tolerance);
        }
    }
    return false;
}

bool AI::trackTarget(int objectID) {
    // Cek apakah objek dengan ID tersebut masih terdeteksi
    for (const auto& obj : detectedObjects) {
        if (obj.id == objectID && obj.isValid) {
            currentTrackedID = objectID;
            return true;
        }
    }
    currentTrackedID = -1;
    return false;
}

// ----- Simulasi sensor (ganti dengan hardware nyata) -----
bool AI::virtualScan(int zoneID, std::vector<KFSObject>& results) {
    // Contoh: mendeteksi beberapa objek palsu untuk testing
    // Di implementasi riil, baca kamera atau sensor KFS.
    // Di sini kita kembalikan false (tidak ada objek) agar tidak mengganggu.
    // Untuk demonstrasi, bisa diisi dummy.
    return false;
}

float AI::readFrontUltrasonic() {
    // Placeholder: baca sensor jarak depan
    return 100.0f; // misal 100 cm
}

void AI::addFakeKFS(int id, float x, float y, int zone) {
    KFSObject obj;
    obj.id = id;
    obj.x = x;
    obj.y = y;
    obj.zoneID = zone;
    obj.isValid = true;
    detectedObjects.push_back(obj);
}