/********************************************************************
 * Açıklama: CoreXY Kinematiği Uygulaması
 * 
 * Bu dosya, 3D yazıcılarda yaygın olarak kullanılan Delta  mekanizmasının kinematiğini uygular.
 * delta mekanizması, X ve Y eksenlerinin hareketini koordineli bir şekilde kontrol etmek için iki motor kullanır.
 * Bu uygulama, basit bir kinematik örneği olan trivkins.c dosyasından uyarlanmıştır.
 * 
 * Referans: http://corexy.com/theory.html  referans 
 * 
 * kinematicsForward fonksiyonu, eklem pozisyonlarına dayanarak Kartezyen koordinatları (X, Y, Z, vb.) hesaplar.
 * kinematicsInverse fonksiyonu, Kartezyen koordinatlarına dayanarak eklem pozisyonlarını hesaplar.
 * 
 * HAL (Donanım Soyutlama Katmanı), yazılımın motorları kontrol etmesini sağlamak için donanım ile arayüz oluşturur.
 ********************************************************************/

#include "motion.h"       // Hareket kontrol tanımlamaları
#include "hal.h"         // Donanım Soyutlama Katmanı (HAL) tanımlamaları
#include "rtapi.h"       // Gerçek Zamanlı API tanımlamaları
#include "rtapi_app.h"   // RTAPI uygulama tanımlamaları
#include "rtapi_math.h"  // RTAPI için matematik fonksiyonları
#include "rtapi_string.h"// RTAPI için string fonksiyonları
#include "kinematics.h"  // Kinematik tanımlamaları
 static double L1,L2,L3;
// Veri yapısı: Motor hareketlerini saklamak için kullanılır
static struct data {
    hal_s32_t joints[EMCMOT_MAX_JOINTS]; // Eklem pozisyonları
    hal_float_t motor1x; // Birinci motorun hareket değeri
    hal_float_t motor1y; // Birinci motorun hareket değeri
    hal_float_t motor2x; // İkinci motorun hareket değeri
    hal_float_t motor2y; // İkinci motorun hareket değeri
    hal_float_t motor3x; // üçüncü motorun hareket değeri
    hal_float_t motor3y; // üçüncü motorun hareket değeri
} *data;
// Motor konumları
//Point2D M1 = {0, 400};    // Motor 1 konumu
//Point2D M2 = {300, 400};  // Motor 2 konumu
//Point2D M3 = {150, 0};    // Motor 3 konumu

// Motor hareket değerlerine erişim için kısayollar
#define Motor1x (data->motor1x) // Birinci motorun hareket değeri
#define Motor1y (data->motor1y) // Birinci motorun hareket değeri
#define Motor2x (data->motor2x) // İkinci motorun hareket değeri
#define Motor2y (data->motor2y) // İkinci motorun hareket değeri
#define Motor3x (data->motor3x) // İkinci motorun hareket değeri
#define Motor3y (data->motor3y) // İkinci motorun hareket değeri
// İki nokta arasındaki mesafeyi hesapla
double distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

/********************************************************************
 * kinematicsForward: İleri Kinematik Hesaplama
 * 
 * Bu fonksiyon, eklem pozisyonlarını (joints) alır ve Kartezyen koordinatlarını (pos) hesaplar.
 * CoreXY den referans  delta mekanizması için:
 * - X pozisyonu, iki motorun hareketlerinin ortalamasıdır.
 * - Y pozisyonu, iki motorun hareketlerinin farkının yarısıdır.
 * - Diğer eksenler (Z, A, B, C, U, V, W) doğrudan eklem pozisyonlarına eşittir.
 ********************************************************************/
int kinematicsForward(const double *joints, EmcPose *pos,
                     const KINEMATICS_FORWARD_FLAGS *fflags,
                     KINEMATICS_INVERSE_FLAGS *iflags) {
    (void)fflags; // Kullanılmayan parametreler
    (void)iflags; // Kullanılmayan parametreler
    //motor eklem koordinatlarını (joint) makine koordinat sistemine (X, Y, Z) dönüştürür.
    /* joints[0], joints[1], joints[2] = r1, r2, r3 (ip uzunlukları) */
    /* cartesians[0], cartesians[1] = X, Y */
    /* İleri Kinematik: (r1, r2, r3) → (X, Y) */
    double A = 2 * (Motor2x - Motor1x);
    double B = 2 * (Motor2y - Motor1y);
    double C = pow(joints[0], 2) - pow(joints[1], 2) - pow(Motor1x, 2) + pow(Motor2x, 2) - pow(Motor1y, 2) + pow(Motor2y, 2);

    double D = 2 * (Motor3x - Motor1x);
    double E = 2 * (Motor3y - Motor1y);
    double F = pow(joints[0], 2) - pow(joints[2], 2) - pow(Motor1x, 2) + pow(Motor3x, 2) - pow(Motor1y, 2) + pow(Motor3y, 2);

    pos->tran.x =  (C*E - B*F) / (A*E - B*D);
    pos->tran.y =  (C*D - A*F) / (B*D - A*E);
    

    
    /*
pos->tran.x = joints[0];
pos->tran.y = joints[1];
pos->tran.z = joints[2];

pos->tran.a = 0;
pos->tran.b = 0;
pos->tran.c = 0;
pos->tran.u = 0;
pos->tran.v = 0;
pos->tran.w = 0;
*/

    return 0; // Başarılı
}

/********************************************************************
 * kinematicsInverse: Ters Kinematik Hesaplama
 * 
 * Bu fonksiyon, Kartezyen koordinatlarını (pos) alır ve eklem pozisyonlarını (joints) hesaplar.
 * delta  mekanizması için:
 * - Motor1 =distance(kalem- m1)
 * - Motor1 =distance(kalem- m2)
 * - Motor1 =distance(kalem- m3)
 * - Diğer eksenler (Z, A, B, C, U, V, W) doğrudan Kartezyen koordinatlarına eşittir.
 ********************************************************************/
int kinematicsInverse(const EmcPose *pos, double *joints,
                     const KINEMATICS_INVERSE_FLAGS *iflags,
                     KINEMATICS_FORWARD_FLAGS *fflags) {
    (void)iflags; // Kullanılmayan parametreler
    (void)fflags; // Kullanılmayan parametreler
    
    joints[0] = sqrt(pow(pos->tran.x - Motor1x, 2) + pow(pos->tran.y - Motor1y, 2));
    joints[1] = sqrt(pow(pos->tran.x - Motor2x, 2) + pow(pos->tran.y - Motor2y, 2));
    joints[2] = sqrt(pow(pos->tran.x - Motor3x, 2) + pow(pos->tran.y - Motor3y, 2));

    

  
    joints[3] = pos->a;      // A doğrudan Kartezyen pozisyonuna eşit
    joints[4] = pos->b;      // B doğrudan Kartezyen pozisyonuna eşit
    joints[5] = pos->c;      // C doğrudan Kartezyen pozisyonuna eşit
    joints[6] = pos->u;      // U doğrudan Kartezyen pozisyonuna eşit
    joints[7] = pos->v;      // V doğrudan Kartezyen pozisyonuna eşit
    joints[8] = pos->w;      // W doğrudan Kartezyen pozisyonuna eşit

    return 0; // Başarılı
}

/********************************************************************
 * kinematicsHome: Başlangıç Pozisyonu Hesaplama
 * 
 * Bu fonksiyon, başlangıç pozisyonunu (home) hesaplar ve kinematicsForward fonksiyonunu çağırır.
 ********************************************************************/
int kinematicsHome(EmcPose *world, double *joint,
                  KINEMATICS_FORWARD_FLAGS *fflags,
                  KINEMATICS_INVERSE_FLAGS *iflags) {
    *fflags = 0; // Bayrakları sıfırla
    *iflags = 0; // Bayrakları sıfırla
    return kinematicsForward(joint, world, fflags, iflags); // İleri kinematik hesapla
}

/********************************************************************
 * kinematicsType: Kinematik Türünü Belirleme
 * 
 * Bu fonksiyon, kinematik türünü belirler. Bu durumda, hem ileri hem de ters kinematik desteklenir.
 ********************************************************************/
KINEMATICS_TYPE kinematicsType() {
    return KINEMATICS_BOTH; // Hem ileri hem de ters kinematik desteklenir
}

// Kinematik fonksiyonlarını dışa aktar
KINS_NOT_SWITCHABLE
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL"); // Lisans bilgisi

// HAL bileşen kimliği
static int comp_id;

/********************************************************************
 * rtapi_app_main: Uygulama Başlatma Fonksiyonu
 * 
 * Bu fonksiyon, HAL bileşenini başlatır ve motor hareket değerlerini başlangıç durumuna getirir.
 ********************************************************************/
int rtapi_app_main(void) {
    int res = 0;
    comp_id = hal_init("3dkinamtil"); // HAL bileşenini başlat
    if (comp_id < 0) return comp_id; // Hata durumunda çık

    data = hal_malloc(sizeof(struct data)); // Veri yapısı için bellek ayır

    // Motor hareket değerlerini başlangıç durumuna getir
    Motor1x = 0.0;
    Motor1y = 0.0;
    Motor2x = 0.0;
    Motor2y = 0.0;
    Motor3x = 0.0;
    Motor3y = 0.0;
    // HAL parametrelerini oluştur
    if ((res = hal_param_float_new("3dkinamtil.Motor1x", HAL_RW, &data->motor1x, comp_id)) < 0) goto error;
    if ((res = hal_param_float_new("3dkinamtil.Motor1y", HAL_RW, &data->motor1y, comp_id)) < 0) goto error;
    if ((res = hal_param_float_new("3dkinamtil.Motor2x", HAL_RW, &data->motor2x, comp_id)) < 0) goto error;
    if ((res = hal_param_float_new("3dkinamtil.Motor2y", HAL_RW, &data->motor2y, comp_id)) < 0) goto error;
    if ((res = hal_param_float_new("3dkinamtil.Motor3x", HAL_RW, &data->motor3x, comp_id)) < 0) goto error;
    if ((res = hal_param_float_new("3dkinamtil.Motor3y", HAL_RW, &data->motor3y, comp_id)) < 0) goto error;
    hal_ready(comp_id); // HAL bileşenini hazır hale getir
    return 0; // Başarılı

error:
    // Hata durumunda HAL bileşenini kapat
    hal_exit(comp_id);
    return res;
}

/********************************************************************
 * rtapi_app_exit: Uygulama Çıkış Fonksiyonu
 * 
 * Bu fonksiyon, HAL bileşenini kapatır.
 ********************************************************************/
void rtapi_app_exit(void) {
    hal_exit(comp_id); // HAL bileşenini kapat
}
