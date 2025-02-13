#include "Robot.h"
#include "moteurs.h"
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

#if !defined(LINE_FOLLOWER_LEFT_PIN)
#error The LineFollower program requires that LINE_FOLLOWER_[LEFT,MID,RIGHT]_PIN are defined
#endif

// ===========================================
// Constructeur et initialisation
// ===========================================

Robot::Robot(int base_speed)
  : robot_speed(base_speed),
    base_speed(base_speed),
    movementState(MOVEMENT_IDLE),
    rotationState(ROTATION_IDLE),
    rotationStartTime(0),
    rotationDuration(0),
    CurrentLineSensorState(0),
    lastTurnDirection(NONE),
    moteurD(base_speed, MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_BACKWARD_PIN, MOTOR_RIGHT_SPEED_PIN, 0.94),
    moteurG(base_speed, MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_BACKWARD_PIN, MOTOR_LEFT_SPEED_PIN, 1) {
  initialize_ir();
  initialize_line_pin();
}

// Fonction d'initialisation 
void Robot::initialize_ir() {
  Serial.println("Initialisation IR");
  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);  // Démarrer le récepteur IR
}

void Robot::initialize_line_pin(){
  pinMode(LINE_FOLLOWER_LEFT_PIN, INPUT);
  pinMode(LINE_FOLLOWER_MID_PIN, INPUT);
  pinMode(LINE_FOLLOWER_RIGHT_PIN, INPUT);
}

// ===========================================
// Fonctions de déplacement linéaire
// ===========================================

// Définit la vitesse du robot et met à jour l'état de déplacement
void Robot::set_robot_speed(int speed) {
  robot_speed = abs(speed);  // Met à jour la vitesse de base (toujours positive)
  if (movementState != LINEFOLLOWING){
    changeMovementState((speed > 0) ? FORWARD : (speed < 0) ? BACKWARD
                                                          : MOVEMENT_IDLE);
  }
  moteurD.set_speed(speed);
  moteurG.set_speed(speed);
}

// Arrête le robot
void Robot::stop() {
  set_robot_speed(0);
}

// Accélère le robot
void Robot::accel() {
  int new_speed = this->robot_speed + ROBOT_ACCELERATION_INCREMENT;  // Augmente la vitesse
  if (new_speed > ROBOT_MAX_SPEED) {
    new_speed = ROBOT_MAX_SPEED;  // Limite à la vitesse max
  }
  set_robot_speed(new_speed);
}

// Décélère le robot
void Robot::deccel() {
  int new_speed = this->robot_speed - ROBOT_ACCELERATION_INCREMENT;  // Réduit la vitesse
  if (new_speed < 0) {
    new_speed = 0;  // Évite une vitesse négative
  }
  set_robot_speed(new_speed);
}

// ===========================================
// Fonctions de rotation / virage
// ===========================================

// Tourne à droite (virage ou pivot)
void Robot::right(bool pivot) {
  changeRotationState(TURNING);

  int speedG = ROBOT_MAX_SPEED;
  int speedD = (pivot) ? 0 : this->robot_speed / 2;

  // Si le robot est en marche arrière, inverser les vitesses
  if (movementState == BACKWARD) {
    speedG = -speedG;
    speedD = -speedD;
  }

  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);

  // Enregistrer le temps de début et la durée du virage
  rotationStartTime = millis();
  rotationDuration = 200;
}

// Tourne à gauche (virage ou pivot)
void Robot::left(bool pivot) {
  changeRotationState(TURNING);

  int speedG = (pivot) ? 0 : this->robot_speed / 2;
  int speedD = ROBOT_MAX_SPEED;

  if (movementState == BACKWARD) {
    speedG = -speedG;
    speedD = -speedD;
  }

  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);

  rotationStartTime = millis();
  rotationDuration = 200;
}

// Effectue une rotation avec angle spécifié
// move indique si la rotation se fait en mouvement (avancer/reculer) et backward si c'est en marche arrière
void Robot::rotate(int angle, bool move, bool backward) {
  float timePerDegree = 200.0 / 90.0;  // Temps pour tourner 90° (à ajuster)
  rotationDuration = abs(angle) * timePerDegree;
  rotationStartTime = millis();

  int speedG, speedD;

  if (!move) {
    // Rotation sur place
    speedG = (angle > 0) ? ROBOT_MAX_SPEED : -ROBOT_MAX_SPEED;
    speedD = (angle > 0) ? -ROBOT_MAX_SPEED : ROBOT_MAX_SPEED;
  } else {
    // Rotation en avançant ou en reculant
    int moveSpeed = backward ? -robot_speed : robot_speed;
    speedG = (angle > 0) ? moveSpeed : moveSpeed / 2;
    speedD = (angle > 0) ? moveSpeed / 2 : moveSpeed;
  }

  moteurG.set_speed(speedG);
  moteurD.set_speed(speedD);

  changeRotationState(ROTATING);
}

// ===========================================
// Gestion des états
// ===========================================

// Change l'état de déplacement
void Robot::changeMovementState(RobotMovementState newMovementState) {
    if (newMovementState == LINEFOLLOWING) {
        resetPID();
    }
    movementState = newMovementState;
}

// Change l'état de rotation
void Robot::changeRotationState(RobotRotationState newRotationState) {
  rotationState = newRotationState;
}

// Affiche les états actuels
void Robot::printState() {
  String RotationstateStr = (rotationState == ROTATION_IDLE) ? "IDLE" : (rotationState == TURNING)  ? "TURNING"
                                                                      : (rotationState == ROTATING) ? "ROTATING"
                                                                                                    : "UNKNOWN";

  String movementStr = (movementState == MOVEMENT_IDLE) ? "IDLE" 
                     : (movementState == FORWARD)  ? "FORWARD"
                     : (movementState == BACKWARD) ? "BACKWARD"
                     : (movementState == LINEFOLLOWING) ? "LINEFOLLOWING"
                     : (movementState == SHARPTURNING) ? "SHARPTURNING"
                     : "UNKNOWN";  // Cas par défaut

  Serial.print("Rotation State: ");
  Serial.print(RotationstateStr);
  Serial.print(" | Movement State: ");
  Serial.println(movementStr);
}

// ===========================================
// Mise à jour (appelée dans la boucle principale)
// ===========================================

void Robot::update() {
  long int current_time = millis();
  // On lit l'état des capteurs une seule fois

  if (rotationState == ROTATING || rotationState == TURNING) {
    if (current_time - rotationStartTime >= rotationDuration) {
      // Appliquer la vitesse en fonction de l'état de déplacement
      int appliedSpeed = (movementState == BACKWARD) ? -robot_speed : robot_speed;
      moteurG.set_speed(appliedSpeed);
      moteurD.set_speed(appliedSpeed);
      rotationState = ROTATION_IDLE;
    }
  }
  if (movementState == SHARPTURNING){
    sharpturn();
  }
  if (movementState == LINEFOLLOWING) {
      line_follower_pid();
  }
}

// ===========================================
// Fonctions de debug
// ===========================================

void Robot::debug() {
  Serial.println("speed :" + String(this->robot_speed));
  this->moteurD.debug();
  this->moteurG.debug();
}

// ===========================================
// Fonctions de mouvement complexes
// ===========================================

// Exécute un mouvement en forme de "8"
void Robot::move_eight() {
  // Premier virage à droite
  right();
  delay(500);   // Temps pour le premier quart du "8"
  delay(1000);  // Demi-boucle

  // Virage à gauche
  left();
  delay(500);   // Deuxième quart du "8"
  delay(1000);  // Deuxième demi-boucle

  // Dernier virage à droite pour terminer le "8"
  right();
  delay(500);  // Dernier virage
}

// ===========================================
// Fonctions liées aux capteurs et à l'IR
// ===========================================

// Décodage de l'IR et exécution de commandes
void Robot::decode_ir() {
  static unsigned long last = millis();
  static bool on = false;

  if (IrReceiver.decode()) {
    auto codeRecu = IrReceiver.decodedIRData.decodedRawData;

    if (millis() - last > 250) {
      on = !on;
      digitalWrite(13, on ? HIGH : LOW);

      // Conversion du code reçu en type CommandeIR
      CommandeIR commande = static_cast<CommandeIR>(codeRecu);

      // Exécuter la commande correspondante
      switch (commande) {
        case CommandeIR::FORWARD:
          set_robot_speed(this->base_speed);
          break;
        case CommandeIR::BACKWARD:
          set_robot_speed(-this->base_speed);
          break;
        case CommandeIR::LEFT:
          left(true);
          break;
        case CommandeIR::RIGHT:
          right(true);
          break;
        case CommandeIR::STOP:
          stop();
          break;
        case CommandeIR::ACCEL:
          accel();
          break;
        case CommandeIR::DECCEL:
          deccel();
          break;
        case CommandeIR::DEMITOUR:
          rotate(180, false, false);
          break;
        case CommandeIR::LINEFOLLOWER:
          if (movementState == LINEFOLLOWING) {
            changeMovementState(MOVEMENT_IDLE);
            stop();
          } else {
            resetPID();
            set_robot_speed(base_speed);  // Réinitialise la vitesse à la valeur de base
            changeMovementState(LINEFOLLOWING);
          }
          break;
        default:
          Serial.println("Commande inconnue !");
          break;
      }
      printState();
      last = millis();
    }

    IrReceiver.resume();  // Préparer pour la prochaine réception
  }
}

uint8_t Robot::getSensorState(){
  uint8_t sensorState = 0;
  if (digitalRead(LINE_FOLLOWER_LEFT_PIN) == HIGH)  sensorState |= 1;  // bit 0
  if (digitalRead(LINE_FOLLOWER_MID_PIN) == HIGH)   sensorState |= 2;  // bit 1
  if (digitalRead(LINE_FOLLOWER_RIGHT_PIN) == HIGH) sensorState |= 4;  // bit 2
  return sensorState;
}

// Line follower via switch case est appelé 
void Robot::line_follower(){
  uint8_t NewSensorState = getSensorState();
  /*
     * According to the 8 different states of the 3 sensor inputs, we perform the following actions:
     * 0 - All sensors are white or not connected -> stop 
     * 1 - Only left sensor is black -> Sharp turn left
     * 2 - Only mid sensor is black -> forward 
     * 3 - Mid and left sensor are black -> left forward
     * 4 - Only right sensor is black -> Sharp turn right
     * 5 - Left and right sensor black -> panic stop
     * 6 - right and mid sensor are black-> right forward
     * 7 - All sensors are black  -> stop or go backward after turn
     */
  /*
     * If sensor input does not change, we do not need to change movement!
     */
  if (this->CurrentLineSensorState != NewSensorState) {
    printState();
    Serial.print(F("SensorState="));
    Serial.print(NewSensorState);
    Serial.print(F(" -> "));
    switch (NewSensorState) {
      case 0:
        stop();
        Serial.println(F("Stop"));
        break;
      case 1:
        changeMovementState(SHARPTURNING);
        Serial.println(F("sharp turn left"));
        break;
      case 2:
        set_robot_speed(this->base_speed);
        Serial.println(F("Forward"));
        break;
      case 3:
        lastTurnDirection = LEFT;
        rotate(-180, true);
        Serial.println(F("left forward"));
        break;
      case 4:
        changeMovementState(SHARPTURNING);
        Serial.println(F("sharp turn left"));
        break;
      case 5:
        stop();
        Serial.println(F("Panic stop"));
        break;
      case 6:
        lastTurnDirection = RIGHT;
        rotate(180, true);
        Serial.println(F("right forward"));
        break;
      case 7:
        sharpturn();
        break;
    }
    this->CurrentLineSensorState = NewSensorState;
  }
}

void Robot::sharpturn(){
  Serial.println("last direction: " + String(lastTurnDirection));
  uint8_t sensorstate = getSensorState();
  if (sensorstate != 1) {
    rotate((lastTurnDirection == RIGHT) ? 5: -5);
  }
  else if (sensorstate == 1){
    changeMovementState(LINEFOLLOWING);
  }
}
// reset des coefficients du PID 
void Robot::resetPID() {
    pidIntegral = 0.0;
    pidLastError = 0.0;
    pidLastTime = 0;
    // Serial.println("pid factor: " + String(pidIntegral) + String(pidLastError) + String(pidLastTime));
}

// Line follower via PID 
void Robot::line_follower_pid() {
  // Lecture des capteurs individuels
  int leftVal  = digitalRead(LINE_FOLLOWER_LEFT_PIN);   // 1 si ligne détectée, 0 sinon
  int midVal   = digitalRead(LINE_FOLLOWER_MID_PIN);
  int rightVal = digitalRead(LINE_FOLLOWER_RIGHT_PIN);

  // Calcul de l'erreur en fonction des poids
  // Ici, on suppose que HIGH (1) signifie détection de la ligne.
  // On attribue des poids : gauche = -1, centre = 0, droite = +1.
  float error = 0.0;
  int count = 0;
  
  if(leftVal == HIGH)  { error += -1; count++; }
  if(midVal == HIGH)   { error += 0;  count++; }
  if(rightVal == HIGH) { error += 1;  count++; }

if(count > 0) {
  error = error / count;
} else if (count == 3) {
  return;
}
else {
  changeMovementState(SHARPTURNING);
  return;
}

  // Calcul du temps écoulé
  unsigned long currentTime = millis();
  float dt;
  if(pidLastTime == 0) {
    dt = 0.01;  // une petite valeur par défaut la première fois
  } else {
    dt = (currentTime - pidLastTime) / 1000.0;  // dt en secondes
  }
  pidLastTime = currentTime;

  // Calcul PID
  pidIntegral += error * dt;
  float derivative = (error - pidLastError) / dt;
  float correction = pidKp * error + pidKi * pidIntegral + pidKd * derivative;
  pidLastError = error;
  lastTurnDirection = (error > 0) ? RIGHT: LEFT;

  // Calcul des vitesses pour les moteurs
  // On part d'une vitesse de base (robot_speed) et on ajuste avec la correction.
  // Par exemple, augmenter la vitesse du moteur gauche et diminuer celle du moteur droit si correction positive.
  int leftSpeed  = robot_speed + correction;
  int rightSpeed = robot_speed - correction;

  // On s'assure que les vitesses restent dans les limites
  if(leftSpeed > ROBOT_MAX_SPEED)  leftSpeed = ROBOT_MAX_SPEED;
  if(rightSpeed > ROBOT_MAX_SPEED) rightSpeed = ROBOT_MAX_SPEED;
  if(leftSpeed < 0)  leftSpeed = 0;
  if(rightSpeed < 0) rightSpeed = 0;

  // Appliquer les vitesses aux moteurs
  // On suppose ici que set_speed() gère la direction correctement (les valeurs positives entraînent l'avance)
  moteurG.set_speed(leftSpeed);
  moteurD.set_speed(rightSpeed);

  // Debug : afficher les valeurs
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" Correction: "); Serial.print(correction);
  Serial.print(" | LeftSpeed: "); Serial.print(leftSpeed);
  Serial.print(" RightSpeed: "); Serial.println(rightSpeed);


}


// (Ici vous pouvez ajouter d'autres fonctions, par exemple pour le détecteur d'objet)

// approche hybride 