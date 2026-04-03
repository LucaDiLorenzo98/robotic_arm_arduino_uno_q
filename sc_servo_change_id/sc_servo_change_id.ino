#include <SCServo.h>

// Configurazione Seriale Hardware 2 per ESP32
HardwareSerial SerialServo(2); 
SMS_STS st;

// --- CONFIGURAZIONE ID ---
int idAttuale = 1;  // L'ID che il servo ha ora (di fabbrica è 1)
int idNuovo = 3;    // L'ID che vuoi assegnare (es. 2, 3, 4...)
// --------------------------

void setup() {
  Serial.begin(115200);
  
  // Avvio comunicazione: 1 Mbps, RX=16, TX=17
  SerialServo.begin(1000000, SERIAL_8N1, 16, 17); 
  st.pSerial = &SerialServo;
  
  delay(2000);
  Serial.println("Inizio procedura cambio ID...");

  // 1. Verifica se il servo risponde all'ID attuale
  if (st.Ping(idAttuale) != -1) {
    Serial.print("Servo ID ");
    Serial.print(idAttuale);
    Serial.println(" trovato!");

    // 2. Sblocca la memoria EPROM per permettere la scrittura
    st.unLockEprom(idAttuale);
    
    // 3. Scrivi il nuovo ID nel registro corrispondente
    st.writeByte(idAttuale, SMS_STS_ID, idNuovo);
    
    // 4. Blocca di nuovo la EPROM (usando il NUOVO ID)
    st.LockEprom(idNuovo);

    Serial.print("SUCCESSO! Il servo ora ha l'ID: ");
    Serial.println(idNuovo);
    Serial.println("Ora puoi scollegarlo e passare al prossimo.");
  } else {
    Serial.println("ERRORE: Servo non rilevato.");
    Serial.println("- Controlla i 12V");
    Serial.println("- Controlla i pin 16 (RX) e 17 (TX)");
    Serial.print("- Verifica se l'ID attuale è davvero ");
    Serial.println(idAttuale);
  }
}

void loop() {
  // Nulla qui, l'operazione avviene una sola volta all'avvio
}