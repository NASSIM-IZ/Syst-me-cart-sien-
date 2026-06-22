

// -- Axes XY --
const int stepPin1   = A0;
const int dirPin1    = 49;
const int enablePin1 = 12;
const int sensorPin1 = 51;   // capteur fin de course Y

const int stepPin2   = A1;
const int dirPin2    = 48;
const int enablePin2 = 7;
const int sensorPin2 = 52;   // capteur fin de course X

const int stepPinZ   = A2;
const int dirPinZ    = 47;
const int sensorPinZ = 50; 

// ============================================================
//  PARAMÈTRES
// ============================================================

const int   stepsPerRevolution = 200;
const float R_MM               = 6.375;
const long  DELAY_MIN_US       = 2000;
const float HOMING_RPM         = 5.0;
const long  STEP_PULSE_US      = 5;
const int   MARGIN_STEPS_Z     = 10;
const float R_PIGNON_MM = 22.5;  // rayon pignon Z (diametre 45mm / 2)
const float ACCEL_XY_MM_S2 = 80.0;

// ============================================================
//  VARIABLES D'ÉTAT
// ============================================================

// -- XY --
long  currentSteps1 = 0;
long  currentSteps2 = 0;
float currentX      = 0.0;
float currentY      = 0.0;
float theta1        = 0.0;
float theta2        = 0.0;
float Xmax          = 0.0;
float Ymax          = 0.0;

// -- Z --
long  currentZ      = 0;
long  Zmax          = 0;
bool  homingZDone   = false;

// ============================================================
//  SECURITE + RECALAGE POSITION PAR CAPTEURS
// ============================================================

void stopMoteursXY() {
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);
}

void stopMoteurZ() {
  digitalWrite(stepPinZ, LOW);
}

void mettreAJourXYDepuisPas() {
  theta1 = (float)currentSteps1 * TWO_PI / stepsPerRevolution;
  theta2 = (float)currentSteps2 * TWO_PI / stepsPerRevolution;

  currentX = (R_MM / 2.0) * (theta1 + theta2);
  currentY = (R_MM / 2.0) * (theta1 - theta2);
}

void synchroniserPasDepuisXY() {
  theta1 = (currentX + currentY) / R_MM;
  theta2 = (currentX - currentY) / R_MM;

  currentSteps1 = (long)round(theta1 * stepsPerRevolution / TWO_PI);
  currentSteps2 = (long)round(theta2 * stepsPerRevolution / TWO_PI);
}
// ============================================================
//  SETUP
// ============================================================

void setup() {
  // Broches XY
  pinMode(stepPin1,   OUTPUT);
  pinMode(dirPin1,    OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(sensorPin1, INPUT_PULLUP);

  pinMode(stepPin2,   OUTPUT);
  pinMode(dirPin2,    OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(sensorPin2, INPUT_PULLUP);

  digitalWrite(stepPin1,   LOW);
  digitalWrite(stepPin2,   LOW);
  digitalWrite(enablePin1, LOW);   // driver ON
  digitalWrite(enablePin2, LOW);   // driver ON

  // Broches Z
  pinMode(stepPinZ,   OUTPUT);
  pinMode(dirPinZ,    OUTPUT);
  pinMode(sensorPinZ, INPUT_PULLUP);
  digitalWrite(stepPinZ, LOW);
  digitalWrite(dirPinZ,  LOW);

  Serial.begin(9600);
  Serial.println("=== SYSTEME 3 AXES ===");
  Serial.println("Initialisation en cours...");
  Serial.println("");

  // Homing XY puis Z
  initialisation_Z();
  homing_XY();
 

  Serial.println("");
  Serial.println("=== SYSTEME PRET ===");
  Serial.println("Commandes XY : v[vitesse]x[X]y[Y]   ex: v50x100y50");
  Serial.println("Commandes Z  : m[pas]v[rpm]          ex: m50v10");
  Serial.println("               b  → Z=0 (position initiale)");
  Serial.println("               h  → Z=Zmax (proche capteur)");
}
long mmToPasZ(float mm) {
  // Circonference = 2 * PI * R
  // 1 tour = stepsPerRevolution pas = 2*PI*R mm
  // donc : pas = mm * stepsPerRevolution / (2*PI*R)
  return (long)(mm * stepsPerRevolution / (TWO_PI * R_PIGNON_MM));
}
// ============================================================
//  LOOP
// ============================================================

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

Serial.print("CMD_LUE:");
Serial.println(input);

    // ── MODE B : Z retour à 0 ──────────────────────────────
    if (input == "b" || input == "B") {
      Serial.println("[Z - MODE B] Retour a Z=0...");
      allerAZ(0, HOMING_RPM);
      return;
    }

    // ── MODE H : Z retour à Zmax ───────────────────────────
    if (input == "h" || input == "H") {
      Serial.println("[Z - MODE H] Retour a Z=Zmax...");
      allerAZ(Zmax, HOMING_RPM);
      return;
    }

    // ── MODE MANUEL Z : m[pas]v[rpm] ───────────────────────
    if (input.indexOf('m') != -1 && input.indexOf('v') != -1
        && input.indexOf('x') == -1 && input.indexOf('y') == -1) {

      int mIdx = input.indexOf('m');
      int vIdx = input.indexOf('v');
      float targetMM = input.substring(mIdx + 1, vIdx).toFloat();
      float rpm      = input.substring(vIdx + 1).toFloat();
      long  targetZ  = mmToPasZ(targetMM);
      float ZmaxMM   = (float)Zmax * (TWO_PI * R_PIGNON_MM) / stepsPerRevolution;

      if (rpm <= 0) {
  Serial.println("ERREUR : RPM invalide !");
  Serial.print("CMD_ERR:");
  Serial.println(input);
  return;
}

      if (targetZ < 0 || targetZ > Zmax) {
        Serial.print("ERREUR : Z="); Serial.print(targetZ);
        Serial.print(" hors limites [0, "); Serial.print(Zmax);
        Serial.println("] pas !");
        Serial.print("CMD_ERR:");
        Serial.println(input);  
        return;
      }

      Serial.print("[Z - MANUEL] Aller a Z="); Serial.print(targetZ);
      Serial.print(" pas | "); Serial.print(rpm, 1); Serial.println(" RPM");
      allerAZ(targetZ, rpm);
      Serial.print("CMD_OK:");
      Serial.println(input);
      return;
    }

    // ── MODE XY : v[V]x[X]y[Y] ─────────────────────────────
    if (input.indexOf('v') != -1 && input.indexOf('x') != -1
        && input.indexOf('y') != -1) {

      int vIdx = input.indexOf('v');
      int xIdx = input.indexOf('x');
      int yIdx = input.indexOf('y');

      float V = input.substring(vIdx + 1, xIdx).toFloat();
      float X = input.substring(xIdx + 1, yIdx).toFloat();
      float Y = input.substring(yIdx + 1).toFloat();

      if (V <= 0) {
  Serial.println("ERREUR : Vitesse invalide !");
  Serial.print("CMD_ERR:");
  Serial.println(input);
  return;
}

      if (X < 0.0 || X > Xmax) {
        Serial.print("ERREUR : X="); Serial.print(X);
        Serial.print(" hors limites [0, "); Serial.print(Xmax, 2);
        Serial.println("] mm !");
        Serial.print("CMD_ERR:");
        Serial.println(input);
        return;
      }
      if (Y < 0.0 || Y > Ymax) {
        Serial.print("ERREUR : Y="); Serial.print(Y);
        Serial.print(" hors limites [0, "); Serial.print(Ymax, 2);
        Serial.println("] mm !");
        Serial.print("CMD_ERR:");
        Serial.println(input);
        return;
      }

      Serial.print("[XY] V="); Serial.print(V);
      Serial.print(" X=");     Serial.print(X);
      Serial.print(" Y=");     Serial.println(Y);
      executeMouvement_XY(X, Y, V);
      Serial.print("CMD_OK:");
      Serial.println(input);
      return;
    }

    // ── Commande inconnue ───────────────────────────────────
    Serial.println("Commande inconnue !");
    Serial.print("CMD_ERR:");
    Serial.println(input);
    Serial.println("  v[V]x[X]y[Y]  → XY   ex: v50x100y50");
    Serial.println("  m[pas]v[rpm]  → Z    ex: m50v10");
    Serial.println("  b             → Z=0");
    Serial.println("  h             → Z=Zmax");
  }
}

// ============================================================
//  HOMING XY — mesure Xmax et Ymax
// ============================================================

void homing_XY() {
  float speed      = 30.0;
  long  pulseDelay = (long)((1000000.0 / ((speed * stepsPerRevolution) / 60.0)) / 4.0);

  // ── 1. AXE Y ─────────────────────────────────────────────
  Serial.println("[HOMING XY] Axe Y...");
  long stepsY = 0;

  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);

  while (true) {
    if (digitalRead(sensorPin1) == LOW) break;
    digitalWrite(stepPin1, HIGH); digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseDelay);
    if (digitalRead(sensorPin1) == LOW) break;
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPin1, LOW);  digitalWrite(stepPin2, LOW);
    if (digitalRead(sensorPin1) == LOW) break;
    delayMicroseconds(pulseDelay * 2);
    stepsY++;
  }
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);

  //Ymax = (stepsY * (TWO_PI / stepsPerRevolution)) * R_MM;
  Ymax = 280 ; 
  Serial.print("[HOMING XY] Ymax = "); Serial.print(Ymax, 2); Serial.println(" mm");

  // Recul capteur Y
  digitalWrite(dirPin1, LOW); digitalWrite(dirPin2, HIGH);
  for (int i = 0; i < 80; i++) {
    digitalWrite(stepPin1, HIGH); digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseDelay * 2);
    digitalWrite(stepPin1, LOW);  digitalWrite(stepPin2, LOW);
    delayMicroseconds(pulseDelay * 2);
  }

  // ── 2. AXE X ─────────────────────────────────────────────
  Serial.println("[HOMING XY] Axe X...");
  long stepsX = 0;

  digitalWrite(dirPin1, HIGH); digitalWrite(dirPin2, HIGH);

  while (true) {
    if (digitalRead(sensorPin2) == LOW) break;
    digitalWrite(stepPin1, HIGH); digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseDelay);
    if (digitalRead(sensorPin2) == LOW) break;
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPin1, LOW);  digitalWrite(stepPin2, LOW);
    if (digitalRead(sensorPin2) == LOW) break;
    delayMicroseconds(pulseDelay * 2);
    stepsX++;
  }
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);

  //Xmax = (stepsX * (TWO_PI / stepsPerRevolution)) * R_MM;
  Xmax = 300 ;
  Serial.print("[HOMING XY] Xmax = "); Serial.print(Xmax, 2); Serial.println(" mm");

  // Recul capteur X
  digitalWrite(dirPin1, LOW); digitalWrite(dirPin2, LOW);
  for (int i = 0; i < 80; i++) {
    digitalWrite(stepPin1, HIGH); digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseDelay * 2);
    digitalWrite(stepPin1, LOW);  digitalWrite(stepPin2, LOW);
    delayMicroseconds(pulseDelay * 2);
  }

  // Reset
  currentSteps1 = 0; currentSteps2 = 0;
  currentX = 0.0;    currentY = 0.0;
  theta1   = 0.0;    theta2   = 0.0;

  Serial.println("[HOMING XY] Termine. Position = (0, 0)");
  Serial.print("Limites : Xmax="); Serial.print(Xmax, 2);
  Serial.print(" mm  Ymax=");      Serial.print(Ymax, 2); Serial.println(" mm");
}

// ============================================================
//  INITIALISATION Z — mesure Zmax
// ============================================================

void initialisation_Z() {
  Serial.println("[INIT Z] Recherche du capteur...");

  long pulseDelay = calculDelaiZ(HOMING_RPM);
  long delayHigh  = STEP_PULSE_US;
  long delayLow   = pulseDelay - STEP_PULSE_US;
  long stepsCount = 0;
  long a = 0;

  // Reculer vers capteur
  digitalWrite(dirPinZ, HIGH);
  delayMicroseconds(10);

  while (digitalRead(sensorPinZ) == HIGH) {
    faireUnPasZ(delayHigh, delayLow);
    stepsCount++;
    a=stepsCount++;
  }

  Serial.print("[INIT Z] Capteur touche apres "); Serial.print(stepsCount); Serial.println(" pas.");

  // Reculer encore MARGIN_STEPS
  Serial.print("[INIT Z] Recul supplementaire de "); Serial.print(MARGIN_STEPS_Z); Serial.println(" pas...");
  digitalWrite(dirPinZ, LOW);
  for (int i = 0; i < MARGIN_STEPS_Z; i++) {
    faireUnPasZ(delayHigh, delayLow);
  }
  //Zmax = stepsCount - MARGIN_STEPS_Z;
  Zmax=130;
  currentZ = Zmax;

  Serial.print("[INIT Z] Zmax = "); Serial.print(Zmax); Serial.println(" pas");
  Serial.print("[INIT Z] a = "); Serial.print(a); Serial.println(" pas");
  Serial.println("[INIT Z] Retour a Z=0 (position initiale)...");


  homingZDone = true;
  Serial.println("[INIT Z] Termine. Z=0 (position initiale)");
}

// ============================================================
//  MOUVEMENT XY — lois cinématiques incrémentales
//  Δθ1 = (dx + dy) / R_MM
//  Δθ2 = (dx - dy) / R_MM
// ============================================================
void attendreUS(unsigned long us) {
  while (us > 16000UL) {
    delayMicroseconds(16000);
    us -= 16000UL;
  }
  if (us > 0) delayMicroseconds(us);
}
void executeMouvement_XY(float Xn, float Yn, float V) {

  float dx = Xn - currentX;
  float dy = Yn - currentY;

  float distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.001) {
    Serial.println("Distance nulle - deja en position.");
    return;
  }

  float deltaTheta1 = (dx + dy) / R_MM;
  float deltaTheta2 = (dx - dy) / R_MM;

  long stepsToMove1 = (long)round(deltaTheta1 * stepsPerRevolution / TWO_PI);
  long stepsToMove2 = (long)round(deltaTheta2 * stepsPerRevolution / TWO_PI);

  digitalWrite(dirPin1, (stepsToMove1 >= 0) ? LOW : HIGH);
  digitalWrite(dirPin2, (stepsToMove2 >= 0) ? LOW : HIGH);
  delayMicroseconds(5);

  long absMove1 = abs(stepsToMove1);
  long absMove2 = abs(stepsToMove2);
  long totalSteps = (absMove1 > absMove2) ? absMove1 : absMove2;

  if (totalSteps <= 0) {
    Serial.println("Aucun pas a effectuer.");
    return;
  }

  float stepDistance = distance / (float)totalSteps;

  float accelDist = (V * V) / (2.0 * ACCEL_XY_MM_S2);
  float vitesseMaxProfil = V;
  bool profilTriangulaire = false;

  if ((2.0 * accelDist) > distance) {
    accelDist = distance / 2.0;
    vitesseMaxProfil = sqrt(distance * ACCEL_XY_MM_S2);
    profilTriangulaire = true;
  }

  Serial.print("dx="); Serial.print(dx, 3);
  Serial.print("mm  dy="); Serial.print(dy, 3);
  Serial.print("mm  distance="); Serial.print(distance, 3);
  Serial.print("mm  Vmax="); Serial.print(vitesseMaxProfil, 2);
  Serial.print("mm/s  accel="); Serial.print(ACCEL_XY_MM_S2, 1);
  Serial.println("mm/s2");

  if (profilTriangulaire) {
    Serial.println("Profil vitesse : TRIANGULAIRE");
  } else {
    Serial.println("Profil vitesse : TRAPEZOIDAL");
  }

  long acc1 = 0;
  long acc2 = 0;

  for (long stepIndex = 0; stepIndex < totalSteps; stepIndex++) {

    bool capteurY = digitalRead(sensorPin1) == LOW;
    bool capteurX = digitalRead(sensorPin2) == LOW;

    if (capteurY || capteurX) {
      stopMoteursXY();
      mettreAJourXYDepuisPas();

      if (capteurY) {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);

        for (int i = 0; i < 80; i++) {
          digitalWrite(stepPin1, HIGH);
          digitalWrite(stepPin2, HIGH);
          delayMicroseconds(20000);
          digitalWrite(stepPin1, LOW);
          digitalWrite(stepPin2, LOW);
          delayMicroseconds(20000);
        }

        currentY = 0.0;
        Serial.println("CAPTEUR 1 DECLENCHE : Y recale a 0");
      }

      if (capteurX) {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, LOW);

        for (int i = 0; i < 80; i++) {
          digitalWrite(stepPin1, HIGH);
          digitalWrite(stepPin2, HIGH);
          delayMicroseconds(20000);
          digitalWrite(stepPin1, LOW);
          digitalWrite(stepPin2, LOW);
          delayMicroseconds(20000);
        }

        currentX = 0.0;
        Serial.println("CAPTEUR 2 DECLENCHE : X recale a 0");
      }

      synchroniserPasDepuisXY();

      Serial.print("Nouvelle position XY : X=");
      Serial.print(currentX, 2);
      Serial.print(" mm  Y=");
      Serial.print(currentY, 2);
      Serial.println(" mm");

      return;
    }

    float s = ((float)stepIndex + 0.5) * stepDistance;
    float vLocal;

    if (s < accelDist) {
      vLocal = sqrt(2.0 * ACCEL_XY_MM_S2 * s);
    } else if (s > (distance - accelDist)) {
      vLocal = sqrt(2.0 * ACCEL_XY_MM_S2 * (distance - s));
    } else {
      vLocal = vitesseMaxProfil;
    }

    if (vLocal < 0.5) vLocal = 0.5;
    if (vLocal > vitesseMaxProfil) vLocal = vitesseMaxProfil;

    unsigned long stepDelay = (unsigned long)((stepDistance / vLocal) * 1000000.0);

    if (stepDelay < DELAY_MIN_US) {
      stepDelay = DELAY_MIN_US;
    }

    bool doStep1 = false;
    bool doStep2 = false;

    acc1 += absMove1;
    acc2 += absMove2;

    if (acc1 >= totalSteps) {
      acc1 -= totalSteps;
      doStep1 = true;
    }

    if (acc2 >= totalSteps) {
      acc2 -= totalSteps;
      doStep2 = true;
    }

    if (doStep1) digitalWrite(stepPin1, HIGH);
    if (doStep2) digitalWrite(stepPin2, HIGH);

    delayMicroseconds(STEP_PULSE_US);

    if (doStep1) {
      digitalWrite(stepPin1, LOW);
      if (stepsToMove1 >= 0) currentSteps1++;
      else currentSteps1--;
    }

    if (doStep2) {
      digitalWrite(stepPin2, LOW);
      if (stepsToMove2 >= 0) currentSteps2++;
      else currentSteps2--;
    }

    if (stepDelay > STEP_PULSE_US) {
      attendreUS(stepDelay - STEP_PULSE_US);
    }
  }

  currentX = Xn;
  currentY = Yn;
  synchroniserPasDepuisXY();

  Serial.println("Position XY atteinte !");
  Serial.print("X="); Serial.print(currentX, 2);
  Serial.print(" mm  Y="); Serial.print(currentY, 2);
  Serial.println(" mm");
}

// ============================================================
//  ALLER À une position Z absolue (en pas)
// ============================================================

void allerAZ(long targetSteps, float rpm) {
  long stepsToMove = targetSteps - currentZ;

  if (stepsToMove == 0) {
    Serial.println("Z deja en position !");
    afficherPositionZ();
    return;
  }

  // stepsToMove > 0 → s'éloigne du capteur → HIGH
  // stepsToMove < 0 → vers capteur          → LOW
  digitalWrite(dirPinZ, (stepsToMove > 0) ? HIGH : LOW);
  delayMicroseconds(10);

  long pulseDelay = calculDelaiZ(rpm);
  long delayHigh  = STEP_PULSE_US;
  long delayLow   = pulseDelay - STEP_PULSE_US;
  long absSteps   = abs(stepsToMove);

  for (long i = 0; i < absSteps; i++) {
    if (digitalRead(sensorPinZ) == LOW) {
    stopMoteurZ();
    
    currentZ = Zmax;
     digitalWrite(dirPinZ, LOW);
  for (int i = 0; i < MARGIN_STEPS_Z; i++) {
    faireUnPasZ(delayHigh, delayLow);
  }
    Serial.println("CAPTEUR 3 DECLENCHE : Z recale a Zmax");
    afficherPositionZ();
    
    return;
  }
    faireUnPasZ(delayHigh, delayLow);
    if (stepsToMove > 0) currentZ++;
    else                  currentZ--;
  }

  Serial.println("Position Z atteinte !");
  afficherPositionZ();
}

// ============================================================
//  UTILITAIRES Z
// ============================================================

void faireUnPasZ(long delayHigh, long delayLow) {
  digitalWrite(stepPinZ, HIGH);
  delayMicroseconds(delayHigh);
  digitalWrite(stepPinZ, LOW);
  delayMicroseconds(delayLow);
}

long calculDelaiZ(float rpm) {
  float stepsPerSecond = (rpm * stepsPerRevolution) / 60.0;
  long  d = (long)(1000000.0 / stepsPerSecond);
  if (d < 2 * STEP_PULSE_US) d = 2 * STEP_PULSE_US;
  return d;
}

void afficherPositionZ() {
  Serial.print("Z="); Serial.print(currentZ);
  Serial.print(" pas");
  if (Zmax > 0) {
    float pct = (float)currentZ / (float)Zmax * 100.0;
    Serial.print("  ("); Serial.print(pct, 1);
    Serial.print("% de Zmax="); Serial.print(Zmax); Serial.print(" pas)");
  }
  Serial.println("");
}