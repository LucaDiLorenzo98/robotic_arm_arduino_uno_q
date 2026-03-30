![Arduino Robotic Arm Cover](images/arduino_robotic_arm_github_cover.png)

# A Robot Arm That Sees You — Built with Arduino UNO Q & Modulino LedMatrix

A robot arm that recognizes people and delivers gadgets to them — no buttons, no commands, just interaction. Built with the new Arduino UNO Q 4GB and the new Modulino LedMatrix, which gives the robot its friendly face. The arm runs on Feetech digital servo motors with a Waveshare control board, and a camera on the head handles the recognition. From high school tinkering to projects like this — this is what Arduino made possible for me.

---

## Hardware necessario

| Componente | Dettagli |
|---|---|
| **Arduino UNO Q** | Board principale (MPU + MCU, onboard LED matrix 8×13) |
| **4× servo STS3215** | Bus seriale half-duplex, IDs 1–4 |
| **Servo PWM standard** | Gripper su pin digitale **D9** |
| **2× sensore di pressione** | Analogici su **A0** e **A1** |
| **Modulino LED Matrix** | Display "testa" 12×8 (opzionale ma consigliato) |
| **Adattatore half-duplex** | Per collegare i STS3215 alla porta Serial dell'UNO Q |

### Schema pin

```
D9         → segnale servo gripper (PWM)
A0         → sensore pressione 1
A1         → sensore pressione 2
Serial     → bus STS3215 (1 Mbit/s, half-duplex)
I2C (SDA/SCL) → Modulino LED Matrix (testa)
```

---

## Librerie Arduino da installare

Apri **Tools → Manage Libraries** in Arduino IDE / Arduino Lab e installa:

| Libreria | Versione minima | Note |
|---|---|---|
| `Arduino_RouterBridge` | ultima | Bridge MCU↔MPU |
| `SCServo` | ultima | Controllo STS3215 |
| `Arduino_LED_Matrix` | ultima | Matrice onboard UNO Q |
| `ArduinoGraphics` | ultima | Dipendenza di LED Matrix |
| `Modulino` oppure `Arduino_Modulino` | ultima | Per la testa LED (opzionale) |

---

## Dipendenze Python

Il progetto gira sul lato MPU (Linux) dell'UNO Q tramite l'ambiente Arduino Python. Non occorre installare nulla manualmente: le dipendenze (`arduino.app_utils`, `arduino.app_bricks.video_objectdetection`) fanno parte dell'SDK Arduino.

Le sole librerie standard usate sono: `datetime`, `time`, `threading`, `math`, `os`, `sys`, `functools`.

---

## Come caricare e avviare

1. Apri **Arduino Lab** (o Arduino IDE con supporto UNO Q).
2. Carica lo sketch `sketch/sketch.ino` sulla board.
3. L'app Python `python/main.py` viene avviata automaticamente dall'ambiente Arduino sul lato MPU.
4. I log in tempo reale vengono scritti anche su `python/robot.log` (con timestamp).

---

## Setup del robot (prima accensione o reset)

All'avvio il robot entra automaticamente in **modalità setup**. La matrice onboard mostra il passo corrente (**S1**, **S2**, **S3**).

Il torque dei servo STS3215 viene **disabilitato**: puoi muovere il braccio liberamente a mano.

### Passo S1 – Posizione HOME

1. Muovi il braccio a mano nella posizione HOME (in piedi, centrato).
2. Tienilo fermo per **5 secondi** → la matrice lampeggia il numero del passo durante il conto.
3. Quando catturata, il robot fa un "cenno" con il servo 4 per confermare, poi passa a S2.

### Passo S2 – Posizioni Slot (fino a 9)

1. Muovi il braccio verso il **primo slot** (dove si trovano le penne).
2. Tienilo fermo per **5 secondi** → slot catturato, il robot torna a HOME e fa il cenno.
3. Ripeti per ogni slot successivo (fino a un massimo di **9 slot**).
4. Dopo l'ultimo slot, il robot torna a HOME e mostra **S3** → setup completo.

> Il sistema ignora piccole vibrazioni e microaggiustamenti: attende un vero movimento manuale prima di iniziare il conto.

### Passo S3 – Setup completato

Il robot entra nello stato **detect** ed è operativo.

---

## Funzionamento normale

```
setup → detect → grab → delivery → (viso rilevato) → release → detect → ...
```

| Stato | Cosa fa | Display |
|---|---|---|
| **detect** | Attende il rilevamento di un viso; animazione idle sui servo | Occhi che lampeggiano, matrice **D** |
| **grab** | Raccoglie la penna dallo slot corrente (rotazione base → discesa → chiusura gripper → HOME) | Occhi fissi, matrice **G** |
| **delivery_waiting** | Attende al HOME con la penna in mano che arrivi un viso | Occhi fissi |
| **release** | Rileva il viso → mostra cuori, apre il gripper, consegna la penna | Occhi a cuore, matrice **R** |

### Dettaglio ciclo grab

1. Rotazione della base verso lo slot
2. Discesa parziale (50% del percorso)
3. Apertura stretta del gripper (~165°)
4. Discesa completa allo slot
5. Chiusura gripper (180° × 3 tentativi)
6. Ritorno a HOME
7. Attesa del viso per la consegna

I sensori di pressione su A0/A1 vengono letti prima e dopo la chiusura del gripper (log: `[GRAB][PRESSURE]`).

### Animazione idle

Quando è in stato **detect**, il braccio esegue un'oscillazione sinusoidale morbida su tutti e 4 i servo (periodo 10 s), poi torna alla HOME e si ferma 30 s prima di ripetere.

---

## Codici errore (display onboard)

| Display | Codice | Causa |
|---|---|---|
| `ER1` | Setup pose unavailable | Uno o più STS3215 non rispondono al ping durante il setup |
| `ER2` | Grab slot not configured | Grab richiesto ma nessuno slot catturato |
| `ER3` | STS move failed | Comando di movimento STS fallito |
| `ER4` | *(deprecato)* | Matrix unavailable |
| `ER5` | Pressure read failed | Errore lettura sensori di pressione |

---

## Log e debug

- I log vengono scritti sia su console che su `python/robot.log` (ogni riga ha timestamp `HH:MM:SS.mmm`).
- Puoi impostare un percorso custom con la variabile d'ambiente `ROBOT_LOG_PATH`.
- I log STS3215 mostrano ping e posizione di ogni servo all'avvio (`[STS3215] Check servo`).

---

## Struttura del progetto

```
sketch/
  sketch.ino         # Firmware Arduino (MCU): servo, STS3215, LED matrix, Bridge
python/
  main.py            # App Python (MPU): face detection, state machine, idle animation
  robot.log          # Log runtime (generato automaticamente)
images/
  arduino_robotic_arm_github_cover.png
```
