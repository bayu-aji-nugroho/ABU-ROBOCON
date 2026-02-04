

/*
cekkfs: digunakan untuk cek kfs apakah ada atau tidak
        jika ada menunggu hingga objeck terambil
sudut(derajat): robot berotasi ke derajat yang diinginkan.
naik: robot naik 20cm dan jalan sampe ke tengah
turun: robot turun 20 cm dan jalan sampe ke tengah
*/

class Jungle{
    public:
        Jungle(bool objekterdeteksi, bool isObjeckTerambil): 
        objekterdeteksi(objekterdeteksi), isObjeckTerambil(isObjeckTerambil){}
    private:
        bool objekterdeteksi, isObjeckTerambil;
        void cekKFS(){
            if(objekterdeteksi){
                for(int i = 0; i < 1000; i++){
                    if(isObjeckTerambil){
                        break;
                    }
                }
            }
            this->isObjeckTerambil == false;
            this->objekterdeteksi == false;
        }
}
void Hutan(){
    sudut(90);
    cekKFS();
    sudut(-90);
    cekKFS();
    sudut(0);
    cekKFS()

}
void main(){
    jungle Hutan
    Hutan.sudut
}