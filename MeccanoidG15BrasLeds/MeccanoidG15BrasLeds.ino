/*
  ESP32 + MECCANOID G15 V1 - 4 servomoteurs de bras + LED RGB de la tete

  Bibliotheque requise :
    https://github.com/alexfrederiksen/MeccanoidForArduino

  Branchement propose (ESP32 DevKit / WROOM-32 classique) :
    GPIO25 -> chaine bras gauche (servos IDs 0 et 1, puis yeux ID 2)
    GPIO27 -> chaine bras droit  (2 servos, IDs 0 et 1)
    Les yeux Meccanoid doivent etre places en dernier dans la chaine gauche.
    GND ESP32 et GND alimentation Meccanoid en commun.

  Aucun module HC-06 externe : le programme utilise le BLE/GATT integre de
  l'ESP32 sous le nom "Mecanoid", avec le meme service UART que yeuxBLE.
  Ne jamais utiliser GPIO6 a GPIO11 sur un ESP32-WROOM : ils sont relies a la
  memoire flash de la carte.

  Le robot demarre en mode LIM : les bras sont libres, sans mouvement impose.
  Les LED des servos indiquent le mode : rouge = enregistrement, vert = lecture.
  Les yeux restent independants et changent seulement avec une balise YEUX_*.
  On apprend les positions directement en placant les bras a la main :
    [LIM]              libere les bras
    [POSITIONS]        affiche les 4 angles lus
    [MEM_REPOS]        memorise la pose de repos
    [MEM_BONJOUR]      memorise la pose main tendue
    [MEM_ETONNEMENT]   memorise la pose bras ouverts

  Les poses sont conservees dans l'EEPROM, meme apres extinction.
  Pour enregistrer un mouvement nomme dans la memoire flash de l'ESP32 :
    [ENREGISTRE_NOM]
    bouger les bras a la main
    [FIN_ENREGISTREMENT]
  Le geste est ensuite relu par [NOM], meme apres une extinction.
  Exemple : [ENREGISTRE_SALUT], [FIN_ENREGISTREMENT], puis [SALUT].
  Console USB : 9600 bauds, commandes entre crochets, puis Envoyer.
  L'aide des commandes s'affiche automatiquement au demarrage.
    [LISTE_MOUVEMENTS] affiche les noms enregistres
    [SUPPRIME_SALUT]   efface uniquement le mouvement SALUT
    [SUPPRIME_1]       efface uniquement le mouvement 1
  Remplacer le suffixe par le nom exact du mouvement a supprimer.
  Suppression immediate, sans confirmation dans la console.
  Le chatbot doit demander confirmation AVANT d'envoyer SUPPRIME_NOM.
  Suppression refusee pendant une lecture, une sequence, un enregistrement
  ou si un mouvement prioritaire attend : terminer l'action puis reessayer.
  STOP pendant un enregistrement tente de le sauvegarder.
  Les autres mouvements et les poses memorisees sont conserves.

  Une fois les 3 poses apprises :
    [REPOS]
    [BONJOUR]
    [ETONNEMENT]
    [TEST_GAUCHE] petit aller-retour du bras gauche, sans pose memorisee
    [TEST_DROIT]  petit aller-retour du bras droit, sans pose memorisee
    [DIAGNOSTIC]  affiche les modules reellement detectes
    [STOP]
    [YEUX_BLEU] [YEUX_VERT] [YEUX_ROUGE] [YEUX_BLANC] [YEUX_OFF]

  Les memes balises sont acceptees depuis le moniteur serie USB et le
  Bluetooth integre de l'ESP32.
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <Preferences.h> // Necessaire au BLE ESP32 3.3.11 lors de l'edition des liens.
#include <LittleFS.h>
#include "MeccanoidESP32.h"
#include <string.h>

// ---------- Broches faciles a modifier ----------
const byte PIN_BRAS_GAUCHE = 25;
const byte PIN_BRAS_DROIT  = 27;
const byte HEAD_LED_ID     = 2;  // Apres les deux servos de la chaine gauche.

const char NOM_BLUETOOTH[] = "Mecanoid";
const char VERSION_PROGRAMME[] = "14 suppression des mouvements";

// Memes UUID BLE UART que le programme yeuxBLE compatible Bubblai.
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ---------- Materiel Meccanoid ----------
Chain chaineGauche(PIN_BRAS_GAUCHE);
Chain chaineDroite(PIN_BRAS_DROIT);

// Correspondance volontairement inversee pour respecter le montage mecanique :
// le premier moteur dans chaque chaine est le moteur logique 1, et le second
// est le moteur logique 0. On ne change donc plus l'ordre physique des cables.
MeccanoServo gauche0 = chaineGauche.getServo(1);
MeccanoServo gauche1 = chaineGauche.getServo(0);
MeccanoServo droite0 = chaineDroite.getServo(1);
MeccanoServo droite1 = chaineDroite.getServo(0);
MeccanoLed yeux = chaineGauche.getLed(HEAD_LED_ID);

BLECharacteristic *caracteristiqueTx = NULL;
bool appareilConnecte = false;
bool ancienEtatConnexion = false;

// File de reception : le callback BLE reste tres court. Les gestes sont
// executes ensuite dans loop(), sans bloquer la pile Bluetooth.
const uint16_t TAILLE_FILE_BLE = 256;
char fileBLE[TAILLE_FILE_BLE];
volatile uint16_t teteFileBLE = 0;
volatile uint16_t queueFileBLE = 0;
portMUX_TYPE verrouFileBLE = portMUX_INITIALIZER_UNLOCKED;

// ---------- Poses apprises ----------
struct Pose {
  byte gauche0;
  byte gauche1;
  byte droite0;
  byte droite1;
};

struct MemoirePoses {
  uint16_t signature;
  byte version;
  byte posesValides; // bit 0=repos, bit 1=bonjour, bit 2=etonnement
  Pose repos;
  Pose bonjour;
  Pose etonnement;
};

const uint16_t SIGNATURE = 0x4D47; // "MG"
// Version augmentee car l'ordre logique des deux moteurs de chaque bras est inverse.
const byte VERSION_MEMOIRE = 2;
const byte POSE_REPOS_OK = 0x01;
const byte POSE_BONJOUR_OK = 0x02;
const byte POSE_ETONNEMENT_OK = 0x04;
const byte TOUTES_POSES_OK = 0x07;

MemoirePoses memoire;
Pose poseCourante = {90, 90, 90, 90};
bool enModeLim = false;
bool mouvementEnCours = false;
bool sequenceEnCours = false;
bool stopDemande = false;

// ---------- Enregistrement autonome d'une trajectoire ----------
// Grille temporelle souhaitee. Si le bus Meccanoid prend plus de 50 ms, le
// temps absolu reste prioritaire et la lecture rejoint le bon point sans
// decaler toute la suite du geste.
const unsigned long INTERVALLE_CAPTURE_MS = 50;
const unsigned long INTERVALLE_LECTURE_MS = 50;
const unsigned long DUREE_MAX_MOUVEMENT_MS = 60000;
const uint16_t MAX_ECHANTILLONS = 1200;

struct EchantillonMouvement {
  uint32_t tempsMs;
  Pose pose;
  byte mesuresFraiches; // bits G0/G1/D0/D1 reellement renouveles par le bus
};

struct EnteteMouvement {
  uint32_t signature;
  uint16_t version;
  uint16_t nombre;
  byte moteursActifs;
  byte reserve[3];
};

const uint32_t SIGNATURE_MOUVEMENT = 0x4D563031; // "MV01"
const uint16_t VERSION_MOUVEMENT = 2;
const byte MOTEUR_G0 = 0x01;
const byte MOTEUR_G1 = 0x02;
const byte MOTEUR_D0 = 0x04;
const byte MOTEUR_D1 = 0x08;

EchantillonMouvement echantillons[MAX_ECHANTILLONS];
uint16_t nombreEchantillons = 0;
byte masqueMoteursActifs = 0;
bool memoireMouvementsPrete = false;
bool enregistrementEnCours = false;
unsigned long debutEnregistrement = 0;
unsigned long derniereCapture = 0;
const byte LONGUEUR_NOM_MOUVEMENT = 24;
char nomEnregistrement[LONGUEUR_NOM_MOUVEMENT + 1] = "";
uint32_t derniersCompteursMesure[4] = {0, 0, 0, 0};

// ---------- Reception des balises ----------
const uint16_t LONGUEUR_COMMANDE_MAX = 192;
const byte MAX_COMMANDES_SEQUENCE = 16;
char commande[LONGUEUR_COMMANDE_MAX];
uint16_t longueurCommande = 0;
bool dansBalise = false;
char mouvementPrioritaire[LONGUEUR_NOM_MOUVEMENT + 1] = "";
bool mouvementPrioritaireEnAttente = false;

void traiterCaractere(char c);
int interpolation(int depart, int arrivee, byte etape, byte total);
void pauseAvecStop(unsigned long dureeMs);
void terminerEnregistrement();

void ajouterCaractereBLE(char c) {
  portENTER_CRITICAL(&verrouFileBLE);
  uint16_t prochaine = (teteFileBLE + 1) % TAILLE_FILE_BLE;
  if (prochaine != queueFileBLE) {
    fileBLE[teteFileBLE] = c;
    teteFileBLE = prochaine;
  }
  portEXIT_CRITICAL(&verrouFileBLE);
}

bool retirerCaractereBLE(char &c) {
  bool disponible = false;
  portENTER_CRITICAL(&verrouFileBLE);
  if (queueFileBLE != teteFileBLE) {
    c = fileBLE[queueFileBLE];
    queueFileBLE = (queueFileBLE + 1) % TAILLE_FILE_BLE;
    disponible = true;
  }
  portEXIT_CRITICAL(&verrouFileBLE);
  return disponible;
}

class RappelsServeurBLE : public BLEServerCallbacks {
  void onConnect(BLEServer *serveur) {
    appareilConnecte = true;
    Serial.println(F("BLE : Bubblai connecte."));
  }

  void onDisconnect(BLEServer *serveur) {
    appareilConnecte = false;
    Serial.println(F("BLE : Bubblai deconnecte."));
  }
};

class RappelsReceptionBLE : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *caracteristique) {
    String texteRecu = caracteristique->getValue().c_str();
    texteRecu.trim();
    if (texteRecu.length() == 0) return;

    Serial.print(F("BLE recu : "));
    Serial.println(texteRecu);

    bool aCrochetOuvrant = texteRecu.indexOf('[') >= 0;
    bool aCrochetFermant = texteRecu.indexOf(']') >= 0;

    // Le canal Actions de certaines versions de l'application retire les
    // crochets. Le robot les reconstruit donc a la frontiere du message BLE.
    if (!aCrochetOuvrant) ajouterCaractereBLE('[');
    for (unsigned int i = 0; i < texteRecu.length(); i++) {
      ajouterCaractereBLE(texteRecu[i]);
    }
    if (!aCrochetFermant) ajouterCaractereBLE(']');

    if (!aCrochetOuvrant || !aCrochetFermant)
      Serial.println(F("BLE : crochets ajoutes automatiquement."));
  }
};

void afficher(const __FlashStringHelper *texte) {
  Serial.println(texte);
}

void actualiserChaines(byte repetitions = 1) {
  for (byte i = 0; i < repetitions; i++) {
    chaineGauche.update();
    chaineDroite.update();
  }
}

void eteindreLedsServomoteurs() {
  // Les petites LED des servomoteurs ne servent pas d'indicateur de mode.
  // Seul le module RGB de la tete doit afficher rouge (LIM) ou vert (lecture).
  gauche0.setColor(0, 0, 0);
  gauche1.setColor(0, 0, 0);
  droite0.setColor(0, 0, 0);
  droite1.setColor(0, 0, 0);
  actualiserChaines(8);
}

void mettreLedsServomoteursRouges() {
  gauche0.setColor(1, 0, 0);
  gauche1.setColor(1, 0, 0);
  droite0.setColor(1, 0, 0);
  droite1.setColor(1, 0, 0);
  actualiserChaines(8);

  // La couleur ne doit pas verrouiller les bras : on confirme ensuite LIM.
  gauche0.setLim(true);
  gauche1.setLim(true);
  droite0.setLim(true);
  droite1.setLim(true);
  actualiserChaines(8);
}

void mettreLedsServomoteursVertes() {
  gauche0.setColor(0, 1, 0);
  gauche1.setColor(0, 1, 0);
  droite0.setColor(0, 1, 0);
  droite1.setColor(0, 1, 0);
  actualiserChaines(8);
}

void traiterNouveauxServomoteurs() {
  bool nouveau = false;
  if (gauche0.justConnected()) nouveau = true;
  if (gauche1.justConnected()) nouveau = true;
  if (droite0.justConnected()) nouveau = true;
  if (droite1.justConnected()) nouveau = true;
  if (!nouveau) return;

  eteindreLedsServomoteurs();

  // Une commande de couleur utilise momentanement le canal de commande du
  // servo. En mode apprentissage, on renvoie donc ensuite l'ordre LIM.
  if (enregistrementEnCours) {
    mettreLedsServomoteursRouges();
  } else if (enModeLim) {
    gauche0.setLim(true);
    gauche1.setLim(true);
    droite0.setLim(true);
    droite1.setLim(true);
    actualiserChaines(8);
  } else {
    mettreLedsServomoteursVertes();
  }
  afficher(F("Nouveau servomoteur detecte : sa LED interne est eteinte."));
}

bool brasConnectes() {
  return gauche0.isConnected() && gauche1.isConnected()
      && droite0.isConnected() && droite1.isConnected();
}

void reglerYeux(byte rouge, byte vert, byte bleu, byte fondu = 1) {
  yeux.setColor(rouge, vert, bleu, fondu);
  // La couleur et le fondu occupent deux messages du protocole Meccanoid.
  chaineGauche.update();
  chaineGauche.update();
}

void chargerMemoire() {
  EEPROM.get(0, memoire);
  if (memoire.signature != SIGNATURE || memoire.version != VERSION_MEMOIRE) {
    memset(&memoire, 0, sizeof(memoire));
    memoire.signature = SIGNATURE;
    memoire.version = VERSION_MEMOIRE;
  }
}

void sauverMemoire() {
  EEPROM.put(0, memoire);
  EEPROM.commit(); // EEPROM emulee dans la memoire flash de l'ESP32
}

void entrerEnLim() {
  eteindreLedsServomoteurs();
  gauche0.setLim(true);
  gauche1.setLim(true);
  droite0.setLim(true);
  droite1.setLim(true);
  actualiserChaines(8);
  enModeLim = true;
  afficher(F("Mode LIM actif : moteurs libres, LED moteurs eteintes."));
}

Pose lirePose() {
  actualiserChaines(8);
  Pose p;
  p.gauche0 = constrain(gauche0.getPosition(), 0, 180);
  p.gauche1 = constrain(gauche1.getPosition(), 0, 180);
  p.droite0 = constrain(droite0.getPosition(), 0, 180);
  p.droite1 = constrain(droite1.getPosition(), 0, 180);
  return p;
}

Pose lirePoseRapide() {
  // Quatre echanges par chaine : un retour frais pour chacun des IDs 0 a 3.
  for (byte i = 0; i < 4; i++) {
    chaineGauche.update();
    chaineDroite.update();
  }
  Pose p;
  p.gauche0 = constrain(gauche0.getPosition(), 0, 180);
  p.gauche1 = constrain(gauche1.getPosition(), 0, 180);
  p.droite0 = constrain(droite0.getPosition(), 0, 180);
  p.droite1 = constrain(droite1.getPosition(), 0, 180);
  return p;
}

void afficherPose(const Pose &p) {
  Serial.print(F("G0=")); Serial.print(p.gauche0);
  Serial.print(F("  G1=")); Serial.print(p.gauche1);
  Serial.print(F("  D0=")); Serial.print(p.droite0);
  Serial.print(F("  D1=")); Serial.println(p.droite1);
}

void afficherEtatServo(const __FlashStringHelper *nom, MeccanoServo &servo) {
  Serial.print(nom);
  if (servo.isConnected()) {
    Serial.print(F("OK, position="));
    Serial.println(constrain(servo.getPosition(), 0, 180));
  } else {
    Serial.println(F("NON DETECTE"));
  }
}

void diagnosticModules() {
  // Une recherche assez longue est necessaire pour attribuer les IDs de tous
  // les modules d'une chaine, surtout juste apres une remise sous tension.
  actualiserChaines(64);
  traiterNouveauxServomoteurs();
  Serial.println(F("--- DIAGNOSTIC MECCANOID ---"));
  afficherEtatServo(F("Bras gauche chaine ID0 : "), gauche1);
  afficherEtatServo(F("Bras gauche chaine ID1 : "), gauche0);
  afficherEtatServo(F("Bras droit  chaine ID0 : "), droite1);
  afficherEtatServo(F("Bras droit  chaine ID1 : "), droite0);
  Serial.print(F("Yeux ID2 apres les moteurs gauches : "));
  Serial.println(yeux.isConnected() ? F("OK") : F("NON DETECTES"));
}

int petiteCibleTest(int position) {
  // Toujours rester entre 0 et 180 degres et s'eloigner de la butee la plus proche.
  return position <= 90 ? min(position + 12, 180) : max(position - 12, 0);
}

void envoyerPoseTest(Chain &chaine, MeccanoServo &servo0, MeccanoServo &servo1,
                     bool servo0OK, bool servo1OK,
                     int depart0, int depart1, int cible0, int cible1,
                     byte etape, byte nombreEtapes) {
  if (servo0OK) servo0.setPosition(interpolation(depart0, cible0, etape, nombreEtapes));
  if (servo1OK) servo1.setPosition(interpolation(depart1, cible1, etape, nombreEtapes));

  // Une chaine comporte quatre emplacements : quatre actualisations garantissent
  // que les ordres destines aux IDs 0 et 1 sont effectivement transmis.
  for (byte i = 0; i < 4; i++) chaine.update();
}

void testerBras(const __FlashStringHelper *nom, Chain &chaine,
                MeccanoServo &servo0, MeccanoServo &servo1) {
  for (byte i = 0; i < 12; i++) chaine.update();
  const bool servo0OK = servo0.isConnected();
  const bool servo1OK = servo1.isConnected();

  Serial.print(F("TEST BRAS "));
  Serial.print(nom);
  Serial.print(F(" : ID0=")); Serial.print(servo0OK ? F("OK") : F("ABSENT"));
  Serial.print(F(", ID1=")); Serial.println(servo1OK ? F("OK") : F("ABSENT"));

  if (!servo0OK && !servo1OK) {
    afficher(F("Aucun moteur detecte sur ce bras : verifier signal, alimentation et chainage."));
    return;
  }

  const int depart0 = servo0OK ? constrain(servo0.getPosition(), 0, 180) : 90;
  const int depart1 = servo1OK ? constrain(servo1.getPosition(), 0, 180) : 90;
  const int cible0 = petiteCibleTest(depart0);
  const int cible1 = petiteCibleTest(depart1);
  const byte etapes = 6;

  stopDemande = false;
  mouvementEnCours = true;
  mettreLedsServomoteursVertes();

  for (byte i = 1; i <= etapes && !stopDemande; i++) {
    envoyerPoseTest(chaine, servo0, servo1, servo0OK, servo1OK,
                    depart0, depart1, cible0, cible1, i, etapes);
    pauseAvecStop(45);
  }
  for (byte i = 1; i <= etapes && !stopDemande; i++) {
    envoyerPoseTest(chaine, servo0, servo1, servo0OK, servo1OK,
                    cible0, cible1, depart0, depart1, i, etapes);
    pauseAvecStop(45);
  }

  // Le test fini, rendre de nouveau tous les moteurs manipulables a la main.
  entrerEnLim();
  mouvementEnCours = false;

  if (stopDemande) afficher(F("TEST interrompu par STOP."));
  else afficher(F("TEST termine : petit aller-retour effectue, retour en mode LIM."));
}

void quitterLimEnGardantLaPose() {
  poseCourante = lirePose();
  gauche0.setLim(false);
  gauche1.setLim(false);
  droite0.setLim(false);
  droite1.setLim(false);
  actualiserChaines(2);
  mettreLedsServomoteursVertes();
  enModeLim = false;
  afficher(F("Mode lecture actif : LED moteurs vertes. Yeux inchanges."));
}

void memoriserPose(Pose &destination, byte drapeau, const __FlashStringHelper *nom) {
  if (!brasConnectes()) {
    afficher(F("ERREUR : les 4 servos ne sont pas tous detectes."));
    return;
  }
  if (!enModeLim) entrerEnLim();
  destination = lirePose();
  memoire.posesValides |= drapeau;
  sauverMemoire();

  Serial.print(F("Pose memorisee : ")); Serial.println(nom);
  afficherPose(destination);
}

void commencerEnregistrement(const char *nom) {
  if (!memoireMouvementsPrete) {
    afficher(F("ERREUR : memoire flash des mouvements indisponible."));
    return;
  }
  if (!brasConnectes()) {
    afficher(F("ERREUR : les 4 servos ne sont pas tous detectes."));
    return;
  }

  if (!enModeLim) entrerEnLim();
  strncpy(nomEnregistrement, nom, LONGUEUR_NOM_MOUVEMENT);
  nomEnregistrement[LONGUEUR_NOM_MOUVEMENT] = '\0';
  nombreEchantillons = 0;

  derniersCompteursMesure[0] = gauche0.getInputSequence();
  derniersCompteursMesure[1] = gauche1.getInputSequence();
  derniersCompteursMesure[2] = droite0.getInputSequence();
  derniersCompteursMesure[3] = droite1.getInputSequence();

  // Rouge sur les quatre servos pendant la capture. Les yeux restent independants.
  mettreLedsServomoteursRouges();

  // Le chronometre commence seulement quand les voyants sont prets.
  enregistrementEnCours = true;
  debutEnregistrement = millis();
  derniereCapture = 0;

  Serial.println();
  Serial.print(F("DEBUT_MOUVEMENT "));
  Serial.println(nomEnregistrement);
  Serial.println(F("Format : numero;temps_ms;G0;G1;D0;D1;F=mesures_fraiches"));
  afficher(F("ENREGISTREMENT ROUGE : bouge les bras, puis [FIN_ENREGISTREMENT]."));
}

bool nomMouvementValide(const char *nom) {
  size_t longueur = strlen(nom);
  if (longueur == 0 || longueur > LONGUEUR_NOM_MOUVEMENT) return false;
  for (size_t i = 0; i < longueur; i++) {
    char c = nom[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_'))
      return false;
  }
  const char *reserves[] = {
    "LIM", "STOP", "POSITIONS", "REPOS", "DIAGNOSTIC",
    "TEST_GAUCHE", "TEST_DROIT", "LISTE_MOUVEMENTS",
    "FIN_ENREGISTREMENT", "YEUX_BLEU", "YEUX_VERT",
    "YEUX_ROUGE", "YEUX_BLANC", "YEUX_OFF", "YEUX_ORANGE",
    "YEUX_JAUNE", "YEUX_AMBRE", "YEUX_TURQUOISE", "YEUX_CYAN",
    "YEUX_VIOLET", "YEUX_ROSE"
  };
  for (byte i = 0; i < sizeof(reserves) / sizeof(reserves[0]); i++)
    if (strcmp(nom, reserves[i]) == 0) return false;
  return true;
}

void cheminMouvement(const char *nom, char *chemin, size_t taille) {
  snprintf(chemin, taille, "/MV_%s.BIN", nom);
}

byte lireValeurMoteur(const Pose &p, byte moteur) {
  if (moteur == 0) return p.gauche0;
  if (moteur == 1) return p.gauche1;
  if (moteur == 2) return p.droite0;
  return p.droite1;
}

void ecrireValeurMoteur(Pose &p, byte moteur, byte valeur) {
  if (moteur == 0) p.gauche0 = valeur;
  else if (moteur == 1) p.gauche1 = valeur;
  else if (moteur == 2) p.droite0 = valeur;
  else p.droite1 = valeur;
}

byte interpolerValeurTimeline(byte moteur, uint32_t tempsCible) {
  const byte bits[4] = {MOTEUR_G0, MOTEUR_G1, MOTEUR_D0, MOTEUR_D1};
  uint16_t precedent = 0;
  int prochain = -1;

  // Le premier point est toujours une ancre valide pour les quatre moteurs.
  for (uint16_t i = 1; i < nombreEchantillons; i++) {
    if (!(echantillons[i].mesuresFraiches & bits[moteur])) continue;
    if (echantillons[i].tempsMs <= tempsCible)
      precedent = i;
    else {
      prochain = i;
      break;
    }
  }

  byte valeurAvant = lireValeurMoteur(echantillons[precedent].pose, moteur);
  if (prochain < 0) return valeurAvant;

  uint32_t tempsAvant = echantillons[precedent].tempsMs;
  uint32_t tempsApres = echantillons[prochain].tempsMs;
  byte valeurApres = lireValeurMoteur(echantillons[prochain].pose, moteur);
  if (tempsApres <= tempsAvant || tempsCible <= tempsAvant) return valeurAvant;

  return constrain(
    valeurAvant + ((long)(valeurApres - valeurAvant)
      * (long)(tempsCible - tempsAvant)) / (long)(tempsApres - tempsAvant),
    0, 180
  );
}

Pose calculerPoseTimeline(uint32_t tempsCible) {
  Pose p;
  p.gauche0 = interpolerValeurTimeline(0, tempsCible);
  p.gauche1 = interpolerValeurTimeline(1, tempsCible);
  p.droite0 = interpolerValeurTimeline(2, tempsCible);
  p.droite1 = interpolerValeurTimeline(3, tempsCible);
  return p;
}

void filtrerPicsIsoles() {
  if (nombreEchantillons < 3) return;
  for (byte moteur = 0; moteur < 4; moteur++) {
    for (uint16_t i = 1; i + 1 < nombreEchantillons; i++) {
      int avant = lireValeurMoteur(echantillons[i - 1].pose, moteur);
      int milieu = lireValeurMoteur(echantillons[i].pose, moteur);
      int apres = lireValeurMoteur(echantillons[i + 1].pose, moteur);
      if (abs(milieu - avant) >= 20
          && abs(milieu - apres) >= 20
          && abs(avant - apres) <= 6) {
        ecrireValeurMoteur(echantillons[i].pose, moteur, (avant + apres) / 2);
      }
    }
  }
}

byte calculerMoteursActifs() {
  byte masque = 0;
  const byte bits[4] = {MOTEUR_G0, MOTEUR_G1, MOTEUR_D0, MOTEUR_D1};
  for (byte moteur = 0; moteur < 4; moteur++) {
    byte minimum = 180;
    byte maximum = 0;
    byte reference = lireValeurMoteur(echantillons[0].pose, moteur);
    uint16_t pointsDifferents = 0;
    for (uint16_t i = 0; i < nombreEchantillons; i++) {
      // Les lignes intermediaires peuvent contenir une copie en cache. Pour
      // decider si un moteur a vraiment bouge, seules ses nouvelles mesures
      // physiques comptent (le premier point reste l'ancre de depart).
      if (i != 0 && !(echantillons[i].mesuresFraiches & bits[moteur])) continue;
      byte valeur = lireValeurMoteur(echantillons[i].pose, moteur);
      minimum = min(minimum, valeur);
      maximum = max(maximum, valeur);
      if (abs((int)valeur - (int)reference) >= 4) pointsDifferents++;
    }
    // Une variation doit durer au moins trois points pour ne pas prendre un
    // parasite isole pour un mouvement volontaire.
    if (maximum - minimum >= 4 && pointsDifferents >= 3) masque |= bits[moteur];
  }
  return masque;
}

bool lireEnteteMouvement(const char *nom, EnteteMouvement &entete) {
  if (!memoireMouvementsPrete || !nomMouvementValide(nom)) return false;
  char chemin[36];
  cheminMouvement(nom, chemin, sizeof(chemin));
  File fichier = LittleFS.open(chemin, "r");
  if (!fichier) return false;
  bool ok = fichier.read((uint8_t *)&entete, sizeof(entete)) == sizeof(entete)
      && entete.signature == SIGNATURE_MOUVEMENT
      && entete.version == VERSION_MOUVEMENT
      && entete.nombre >= 2
      && entete.nombre <= MAX_ECHANTILLONS;
  fichier.close();
  return ok;
}

bool mouvementDisponible(const char *nom) {
  EnteteMouvement entete;
  return lireEnteteMouvement(nom, entete);
}

bool sauvegarderMouvement(const char *nom) {
  if (!memoireMouvementsPrete || !nomMouvementValide(nom) || nombreEchantillons < 2)
    return false;

  filtrerPicsIsoles();
  masqueMoteursActifs = calculerMoteursActifs();
  if (masqueMoteursActifs == 0) return false;

  EnteteMouvement entete;
  entete.signature = SIGNATURE_MOUVEMENT;
  entete.version = VERSION_MOUVEMENT;
  entete.nombre = nombreEchantillons;
  entete.moteursActifs = masqueMoteursActifs;
  memset(entete.reserve, 0, sizeof(entete.reserve));

  char chemin[36];
  cheminMouvement(nom, chemin, sizeof(chemin));
  File fichier = LittleFS.open(chemin, "w");
  if (!fichier) return false;
  size_t taillePoints = nombreEchantillons * sizeof(EchantillonMouvement);
  bool ok = fichier.write((const uint8_t *)&entete, sizeof(entete)) == sizeof(entete)
      && fichier.write((const uint8_t *)echantillons, taillePoints) == taillePoints;
  fichier.close();
  return ok;
}

bool chargerMouvement(const char *nom) {
  EnteteMouvement entete;
  if (!lireEnteteMouvement(nom, entete)) return false;
  char chemin[36];
  cheminMouvement(nom, chemin, sizeof(chemin));
  File fichier = LittleFS.open(chemin, "r");
  if (!fichier || !fichier.seek(sizeof(EnteteMouvement))) return false;
  size_t taillePoints = entete.nombre * sizeof(EchantillonMouvement);
  bool ok = fichier.read((uint8_t *)echantillons, taillePoints) == taillePoints;
  fichier.close();
  if (!ok) return false;
  nombreEchantillons = entete.nombre;
  masqueMoteursActifs = entete.moteursActifs;
  return true;
}

// La confirmation utilisateur est geree par le chatbot avant l'envoi.
// Ne supprime que le fichier du nom exact, jamais toute la memoire.
void supprimerMouvement(const char *nom) {
  if (!memoireMouvementsPrete) {
    afficher(F("ERREUR SUPPRESSION : memoire indisponible."));
    return;
  }
  if (!nomMouvementValide(nom)) {
    afficher(F("ERREUR SUPPRESSION : nom invalide."));
    return;
  }
  if (enregistrementEnCours || mouvementEnCours || sequenceEnCours
      || mouvementPrioritaireEnAttente) {
    afficher(F("SUPPRESSION REFUSEE : termine l'action ou envoie STOP, puis renvoie SUPPRIME_NOM."));
    return;
  }
  char chemin[36];
  cheminMouvement(nom, chemin, sizeof(chemin));
  if (!LittleFS.exists(chemin)) {
    Serial.print(F("MOUVEMENT INTROUVABLE : "));
    Serial.println(nom);
    return;
  }
  if (!LittleFS.remove(chemin)) {
    afficher(F("ERREUR SUPPRESSION : echec de l'effacement."));
    return;
  }
  Serial.print(F("MOUVEMENT SUPPRIME : "));
  Serial.println(nom);
}

void listerMouvements() {
  if (!memoireMouvementsPrete) {
    afficher(F("Memoire des mouvements indisponible."));
    return;
  }
  afficher(F("--- MOUVEMENTS EN MEMOIRE ---"));
  File racine = LittleFS.open("/");
  File fichier = racine.openNextFile();
  bool trouve = false;
  while (fichier) {
    String nom = fichier.name();
    if (nom.startsWith("/")) nom.remove(0, 1);
    if (nom.startsWith("MV_") && nom.endsWith(".BIN")) {
      nom = nom.substring(3, nom.length() - 4);
      Serial.print('['); Serial.print(nom); Serial.println(']');
      trouve = true;
    }
    fichier.close();
    fichier = racine.openNextFile();
  }
  racine.close();
  if (!trouve) afficher(F("Aucun mouvement enregistre."));
}

void afficherMoteursActifs(byte masque) {
  Serial.print(F("Moteurs reellement bouges : "));
  bool premier = true;
  const char *noms[4] = {"G0", "G1", "D0", "D1"};
  const byte bits[4] = {MOTEUR_G0, MOTEUR_G1, MOTEUR_D0, MOTEUR_D1};
  for (byte i = 0; i < 4; i++) {
    if (masque & bits[i]) {
      if (!premier) Serial.print(',');
      Serial.print(noms[i]);
      premier = false;
    }
  }
  if (premier) Serial.print(F("AUCUN"));
  Serial.println();
}

void enregistrerEchantillonSiNecessaire() {
  if (!enregistrementEnCours) return;

  unsigned long maintenant = millis();
  unsigned long tempsRelatif = maintenant - debutEnregistrement;
  if (tempsRelatif >= DUREE_MAX_MOUVEMENT_MS || nombreEchantillons >= MAX_ECHANTILLONS) {
    afficher(F("Duree maximale atteinte : sauvegarde automatique."));
    terminerEnregistrement();
    return;
  }
  if (nombreEchantillons != 0 && maintenant - derniereCapture < INTERVALLE_CAPTURE_MS) return;

  derniereCapture = maintenant;
  // Les chaines viennent d'etre actualisees dans loop(). On utilise ces valeurs
  // en cache pour ne plus ajouter les longs echanges qui limitaient la capture.
  Pose p;
  p.gauche0 = constrain(gauche0.getPosition(), 0, 180);
  p.gauche1 = constrain(gauche1.getPosition(), 0, 180);
  p.droite0 = constrain(droite0.getPosition(), 0, 180);
  p.droite1 = constrain(droite1.getPosition(), 0, 180);

  uint32_t compteurs[4] = {
    gauche0.getInputSequence(), gauche1.getInputSequence(),
    droite0.getInputSequence(), droite1.getInputSequence()
  };
  byte mesuresFraiches = nombreEchantillons == 0 ? 0x0F : 0x00;
  const byte bitsMesures[4] = {MOTEUR_G0, MOTEUR_G1, MOTEUR_D0, MOTEUR_D1};
  for (byte moteur = 0; moteur < 4; moteur++) {
    if (compteurs[moteur] != derniersCompteursMesure[moteur])
      mesuresFraiches |= bitsMesures[moteur];
    derniersCompteursMesure[moteur] = compteurs[moteur];
  }

  EchantillonMouvement &point = echantillons[nombreEchantillons];
  point.tempsMs = tempsRelatif;
  point.pose = p;
  point.mesuresFraiches = mesuresFraiches;

  // Sortie uniquement sur USB pour ne pas saturer la liaison Bluetooth.
  Serial.print(nombreEchantillons++);
  Serial.print(';'); Serial.print(tempsRelatif);
  Serial.print(';'); Serial.print(p.gauche0);
  Serial.print(';'); Serial.print(p.gauche1);
  Serial.print(';'); Serial.print(p.droite0);
  Serial.print(';'); Serial.print(p.droite1);
  Serial.print(F(";F=")); Serial.println(mesuresFraiches, HEX);
}

void terminerEnregistrement() {
  if (!enregistrementEnCours) {
    afficher(F("Aucun mouvement n'est en cours d'enregistrement."));
    return;
  }

  enregistrementEnCours = false;
  bool sauvegardeOK = sauvegarderMouvement(nomEnregistrement);
  Serial.print(F("FIN_MOUVEMENT "));
  Serial.print(nomEnregistrement);
  Serial.print(F(" - "));
  Serial.print(nombreEchantillons);
  Serial.println(F(" echantillons"));
  Serial.println();

  eteindreLedsServomoteurs();
  gauche0.setLim(true);
  gauche1.setLim(true);
  droite0.setLim(true);
  droite1.setLim(true);
  actualiserChaines(8);
  enModeLim = true;

  if (sauvegardeOK) {
    afficherMoteursActifs(masqueMoteursActifs);
    afficher(F("Mouvement sauvegarde dans l'ESP32. Il peut etre lu immediatement."));
  } else
    afficher(F("ERREUR : mouvement trop court ou echec de sauvegarde."));
}

int interpolation(int depart, int arrivee, byte etape, byte total) {
  return depart + ((long)(arrivee - depart) * etape) / total;
}

void traiterCaractere(char c);

void ecouterEntrees() {
  while (Serial.available()) traiterCaractere((char)Serial.read());
  char c;
  while (retirerCaractereBLE(c)) traiterCaractere(c);
}

void pauseAvecStop(unsigned long dureeMs) {
  unsigned long debut = millis();
  while (!stopDemande && millis() - debut < dureeMs) {
    ecouterEntrees();
    actualiserChaines();
  }
}

bool allerVers(const Pose &cible, unsigned long dureeMs) {
  if (!brasConnectes()) {
    afficher(F("ERREUR : les 4 servos ne sont pas tous detectes."));
    return false;
  }

  if (enModeLim) quitterLimEnGardantLaPose();

  const Pose depart = poseCourante;
  const byte etapes = 20;
  stopDemande = false;
  mouvementEnCours = true;

  for (byte i = 1; i <= etapes && !stopDemande; i++) {
    Pose intermediaire;
    intermediaire.gauche0 = interpolation(depart.gauche0, cible.gauche0, i, etapes);
    intermediaire.gauche1 = interpolation(depart.gauche1, cible.gauche1, i, etapes);
    intermediaire.droite0 = interpolation(depart.droite0, cible.droite0, i, etapes);
    intermediaire.droite1 = interpolation(depart.droite1, cible.droite1, i, etapes);

    gauche0.setPosition(intermediaire.gauche0);
    gauche1.setPosition(intermediaire.gauche1);
    droite0.setPosition(intermediaire.droite0);
    droite1.setPosition(intermediaire.droite1);
    chaineGauche.update();
    chaineDroite.update();
    poseCourante = intermediaire;

    unsigned long attente = dureeMs / etapes;
    unsigned long debutEtape = millis();
    while (!stopDemande && millis() - debutEtape < attente) {
      ecouterEntrees();
    }
  }

  mouvementEnCours = false;
  if (stopDemande) afficher(F("STOP : mouvement interrompu et position maintenue."));
  return !stopDemande;
}

void commanderPoseMasquee(const Pose &p, byte masque) {
  if (masque & MOTEUR_G0) gauche0.setPosition(p.gauche0);
  if (masque & MOTEUR_G1) gauche1.setPosition(p.gauche1);
  if (masque & MOTEUR_D0) droite0.setPosition(p.droite0);
  if (masque & MOTEUR_D1) droite1.setPosition(p.droite1);
  // Un seul paquet transmet simultanement les deux positions d'un meme bras.
  // Ne pas interroger une chaine inactive libere du temps pour la timeline.
  if (masque & (MOTEUR_G0 | MOTEUR_G1)) chaineGauche.update();
  if (masque & (MOTEUR_D0 | MOTEUR_D1)) chaineDroite.update();
  if (masque & MOTEUR_G0) poseCourante.gauche0 = p.gauche0;
  if (masque & MOTEUR_G1) poseCourante.gauche1 = p.gauche1;
  if (masque & MOTEUR_D0) poseCourante.droite0 = p.droite0;
  if (masque & MOTEUR_D1) poseCourante.droite1 = p.droite1;
}

bool allerVersMasque(const Pose &cible, unsigned long dureeMs, byte masque) {
  if (enModeLim) quitterLimEnGardantLaPose();
  const Pose depart = poseCourante;
  const byte etapes = 20;
  stopDemande = false;
  mouvementEnCours = true;

  for (byte i = 1; i <= etapes && !stopDemande; i++) {
    Pose intermediaire = depart;
    if (masque & MOTEUR_G0)
      intermediaire.gauche0 = interpolation(depart.gauche0, cible.gauche0, i, etapes);
    if (masque & MOTEUR_G1)
      intermediaire.gauche1 = interpolation(depart.gauche1, cible.gauche1, i, etapes);
    if (masque & MOTEUR_D0)
      intermediaire.droite0 = interpolation(depart.droite0, cible.droite0, i, etapes);
    if (masque & MOTEUR_D1)
      intermediaire.droite1 = interpolation(depart.droite1, cible.droite1, i, etapes);
    commanderPoseMasquee(intermediaire, masque);

    unsigned long attente = dureeMs / etapes;
    unsigned long debutEtape = millis();
    while (!stopDemande && millis() - debutEtape < attente) ecouterEntrees();
  }
  mouvementEnCours = false;
  return !stopDemande;
}

bool rejouerMouvement(const char *nom) {
  if (!brasConnectes()) {
    afficher(F("ERREUR : les 4 servos ne sont pas tous detectes."));
    return false;
  }
  if (!chargerMouvement(nom)) return false;

  Serial.print(F("LECTURE_MOUVEMENT "));
  Serial.print(nom);
  Serial.print(F(" - "));
  Serial.print(nombreEchantillons);
  Serial.println(F(" echantillons"));
  afficherMoteursActifs(masqueMoteursActifs);

  // On rejoint doucement la premiere position avant de lancer le chronometre.
  sequenceEnCours = true;
  if (!allerVersMasque(echantillons[0].pose, 1000, masqueMoteursActifs)) {
    sequenceEnCours = false;
    return true;
  }

  stopDemande = false;
  mouvementEnCours = true;
  const uint32_t tempsOrigine = echantillons[0].tempsMs;
  const uint32_t dureeTimeline =
    echantillons[nombreEchantillons - 1].tempsMs - tempsOrigine;
  const unsigned long debutLecture = millis();

  uint32_t prochaineCommande = 0;
  while (!stopDemande) {
    uint32_t tempsEcoule = millis() - debutLecture;
    if (tempsEcoule >= dureeTimeline) break;

    if (tempsEcoule >= prochaineCommande) {
      // Calculer depuis le chronometre absolu : si le bus prend du retard, on
      // saute directement a la bonne position sans ralentir le mouvement.
      Pose p = calculerPoseTimeline(tempsOrigine + tempsEcoule);
      commanderPoseMasquee(p, masqueMoteursActifs);
      prochaineCommande += INTERVALLE_LECTURE_MS;
      if (prochaineCommande <= tempsEcoule)
        prochaineCommande = tempsEcoule + INTERVALLE_LECTURE_MS;
    } else {
      ecouterEntrees();
      delay(1);
    }
  }

  if (!stopDemande)
    commanderPoseMasquee(
      calculerPoseTimeline(echantillons[nombreEchantillons - 1].tempsMs),
      masqueMoteursActifs
    );

  mouvementEnCours = false;
  sequenceEnCours = false;
  if (stopDemande)
    afficher(F("STOP : lecture interrompue et position maintenue."));
  else
    afficher(F("Lecture terminee : vitesse et chronologie enregistrees respectees."));
  return true;
}

struct CommandeYeux {
  const char *commande;
  const char *nomCouleur;
  byte rouge;
  byte vert;
  byte bleu;
  byte fondu;
};

const CommandeYeux COMMANDES_YEUX[] = {
  {"YEUX_ROUGE",     "ROUGE",     7, 0, 0, 1},
  {"YEUX_ORANGE",    "ORANGE",    7, 2, 0, 1},
  {"YEUX_JAUNE",     "JAUNE",     7, 7, 0, 1},
  {"YEUX_AMBRE",     "AMBRE",     7, 4, 0, 1},
  {"YEUX_VERT",      "VERT",      0, 7, 0, 2},
  {"YEUX_TURQUOISE", "TURQUOISE", 0, 7, 4, 2},
  {"YEUX_CYAN",      "CYAN",      0, 7, 7, 2},
  {"YEUX_BLEU",      "BLEU",      0, 1, 7, 2},
  {"YEUX_VIOLET",    "VIOLET",    5, 0, 7, 2},
  {"YEUX_ROSE",      "ROSE",      7, 0, 3, 2},
  {"YEUX_BLANC",     "BLANC",     7, 7, 7, 1},
  {"YEUX_OFF",       "ETEINTS",   0, 0, 0, 2}
};

struct CommandeEmotion {
  const char *commande;
  const char *nomCouleur;
  byte rouge;
  byte vert;
  byte bleu;
  byte fondu;
};

// Une emotion change toujours les yeux. Si un mouvement du meme nom a ete
// enregistre, il est aussi joue automatiquement.
const CommandeEmotion COMMANDES_EMOTIONS[] = {
  {"JOIE",       "JAUNE",     7, 7, 0, 1},
  {"TRISTESSE",  "BLEU",      0, 1, 7, 3},
  {"COLERE",     "ROUGE",     7, 0, 0, 0},
  {"PEUR",       "VIOLET",    5, 0, 7, 1},
  {"SURPRISE",   "BLANC",     7, 7, 7, 0},
  {"CALME",      "CYAN",      0, 5, 7, 4},
  {"TENDRESSE",  "ROSE",      7, 0, 3, 3},
  {"CURIOSITE",  "TURQUOISE", 0, 7, 4, 1},
  {"CONFUSION",  "ORANGE",    7, 2, 0, 2},
  {"FIERTE",     "AMBRE",     7, 4, 0, 2},
  {"DEGOUT",     "VERT",      1, 5, 0, 2},
  {"NEUTRE",     "BLANC",     3, 3, 3, 3}
};

bool executerCommandeYeux(const char *cmd) {
  for (byte i = 0; i < sizeof(COMMANDES_YEUX) / sizeof(COMMANDES_YEUX[0]); i++) {
    const CommandeYeux &couleur = COMMANDES_YEUX[i];
    if (strcmp(cmd, couleur.commande) != 0) continue;
    reglerYeux(couleur.rouge, couleur.vert, couleur.bleu, couleur.fondu);
    Serial.print(F("RESULTAT : yeux = ")); Serial.println(couleur.nomCouleur);
    return true;
  }
  return false;
}

bool executerCommandeEmotion(const char *cmd, bool lancerMouvementAssocie = true) {
  for (byte i = 0; i < sizeof(COMMANDES_EMOTIONS) / sizeof(COMMANDES_EMOTIONS[0]); i++) {
    const CommandeEmotion &emotion = COMMANDES_EMOTIONS[i];
    if (strcmp(cmd, emotion.commande) != 0) continue;

    reglerYeux(emotion.rouge, emotion.vert, emotion.bleu, emotion.fondu);
    Serial.print(F("EMOTION : ")); Serial.print(emotion.commande);
    Serial.print(F(" - yeux ")); Serial.println(emotion.nomCouleur);

    if (mouvementDisponible(emotion.commande)) {
      if (lancerMouvementAssocie) {
        Serial.println(F("Mouvement associe trouve : lecture."));
        rejouerMouvement(emotion.commande);
      } else {
        Serial.println(F("Mouvement associe trouve : selectionne."));
      }
    } else {
      Serial.println(F("Aucun mouvement associe : yeux seulement."));
    }
    return true;
  }
  return false;
}

bool estCommandeMouvement(const char *cmd) {
  if (strncmp(cmd, "SUPPRIME_", 9) == 0) return false;
  return mouvementDisponible(cmd)
      || strcmp(cmd, "REPOS") == 0
      || strcmp(cmd, "BONJOUR") == 0
      || strcmp(cmd, "ETONNEMENT") == 0
      || strcmp(cmd, "TEST_GAUCHE") == 0
      || strcmp(cmd, "TEST_DROIT") == 0;
}

void demanderMouvementPrioritaire(const char *cmd) {
  strncpy(mouvementPrioritaire, cmd, LONGUEUR_NOM_MOUVEMENT);
  mouvementPrioritaire[LONGUEUR_NOM_MOUVEMENT] = '\0';
  mouvementPrioritaireEnAttente = true;
  stopDemande = true;
  Serial.print(F("PRIORITE : interruption, prochain mouvement = "));
  Serial.println(mouvementPrioritaire);
}

void gesteBonjour() {
  if (mouvementDisponible("BONJOUR")) {
    rejouerMouvement("BONJOUR");
    return;
  }
  if ((memoire.posesValides & (POSE_REPOS_OK | POSE_BONJOUR_OK))
      != (POSE_REPOS_OK | POSE_BONJOUR_OK)) {
    afficher(F("Il faut d'abord memoriser REPOS et BONJOUR."));
    return;
  }
  sequenceEnCours = true;
  if (allerVers(memoire.bonjour, 1000)) {
    pauseAvecStop(900);
    if (!stopDemande) allerVers(memoire.repos, 1000);
  }
  sequenceEnCours = false;
}

void gesteEtonnement() {
  if (mouvementDisponible("ETONNEMENT")) {
    rejouerMouvement("ETONNEMENT");
    return;
  }
  if ((memoire.posesValides & (POSE_REPOS_OK | POSE_ETONNEMENT_OK))
      != (POSE_REPOS_OK | POSE_ETONNEMENT_OK)) {
    afficher(F("Il faut d'abord memoriser REPOS et ETONNEMENT."));
    return;
  }
  sequenceEnCours = true;
  if (allerVers(memoire.etonnement, 700)) {
    pauseAvecStop(1100);
    if (!stopDemande) {
      allerVers(memoire.repos, 1100);
    }
  }
  sequenceEnCours = false;
}

void executerCommande(const char *cmd) {
  Serial.print(F("BALISE RECUE : ["));
  Serial.print(cmd);
  Serial.println(']');

  if (strncmp(cmd, "SUPPRIME_", 9) == 0) {
    supprimerMouvement(cmd + 9);
    return;
  }

  if (strcmp(cmd, "FIN_ENREGISTREMENT") == 0) {
    terminerEnregistrement();
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    if (enregistrementEnCours) terminerEnregistrement();
    mouvementPrioritaireEnAttente = false;
    stopDemande = true;
    return;
  }

  if (enregistrementEnCours) {
    afficher(F("Enregistrement actif : utilise [FIN_ENREGISTREMENT] ou [STOP]."));
    return;
  }

  // Les yeux sont independants : leur couleur peut changer pendant que les
  // bras continuent leur trajectoire.
  if (executerCommandeYeux(cmd)) return;

  if (mouvementEnCours || sequenceEnCours) {
    // Une emotion agit tout de suite sur les yeux. Si elle possede aussi un
    // mouvement enregistre, ce mouvement remplace celui qui joue actuellement.
    if (executerCommandeEmotion(cmd, false)) {
      if (mouvementDisponible(cmd)) demanderMouvementPrioritaire(cmd);
      return;
    }

    if (estCommandeMouvement(cmd)) {
      demanderMouvementPrioritaire(cmd);
      return;
    }

    afficher(F("Mouvement en cours : commande ignoree, sauf yeux, emotion, mouvement ou STOP."));
    return;
  }

  if (strncmp(cmd, "ENREGISTRE_", 11) == 0) {
    const char *nom = cmd + 11;
    if (nomMouvementValide(nom)) commencerEnregistrement(nom);
    else afficher(F("Nom invalide : utilise 1 a 24 caracteres A-Z, 0-9 ou _."));
    return;
  }

  if (strcmp(cmd, "LIM") == 0) entrerEnLim();
  else if (strcmp(cmd, "LISTE_MOUVEMENTS") == 0) listerMouvements();
  else if (strcmp(cmd, "POSITIONS") == 0) afficherPose(lirePose());
  else if (strcmp(cmd, "MEM_REPOS") == 0)
    memoriserPose(memoire.repos, POSE_REPOS_OK, F("REPOS"));
  else if (strcmp(cmd, "MEM_BONJOUR") == 0)
    memoriserPose(memoire.bonjour, POSE_BONJOUR_OK, F("BONJOUR"));
  else if (strcmp(cmd, "MEM_ETONNEMENT") == 0)
    memoriserPose(memoire.etonnement, POSE_ETONNEMENT_OK, F("ETONNEMENT"));
  else if (strcmp(cmd, "DIAGNOSTIC") == 0) diagnosticModules();
  else if (strcmp(cmd, "TEST_GAUCHE") == 0)
    testerBras(F("GAUCHE"), chaineGauche, gauche0, gauche1);
  else if (strcmp(cmd, "TEST_DROIT") == 0)
    testerBras(F("DROIT"), chaineDroite, droite0, droite1);
  else if (strcmp(cmd, "REPOS") == 0) {
    if (memoire.posesValides & POSE_REPOS_OK) allerVers(memoire.repos, 1000);
    else afficher(F("La pose REPOS n'est pas encore memorisee."));
  }
  else if (strcmp(cmd, "BONJOUR") == 0) gesteBonjour();
  else if (strcmp(cmd, "ETONNEMENT") == 0) gesteEtonnement();
  else if (executerCommandeEmotion(cmd)) return;
  else if (mouvementDisponible(cmd)) rejouerMouvement(cmd);
  else afficher(F("Balise inconnue."));
}

void executerSequenceCommandes(char *liste) {
  // Dans une commande composee, les yeux sont appliques immediatement et seul
  // le dernier mouvement est conserve. Cela evite d'empiler des gestes qui
  // seraient deja en retard sur la conversation en temps reel.
  byte nombreCommandes = 0;
  char *debut = liste;
  char dernierMouvement[LONGUEUR_NOM_MOUVEMENT + 1] = "";
  bool mouvementTrouve = false;

  while (debut != NULL && *debut != '\0' && nombreCommandes < MAX_COMMANDES_SEQUENCE) {
    char *virgule = strchr(debut, ',');
    if (virgule != NULL) *virgule = '\0';

    while (*debut == ' ' || *debut == '\t') debut++;
    char *fin = debut + strlen(debut);
    while (fin > debut && (fin[-1] == ' ' || fin[-1] == '\t')) *--fin = '\0';

    if (*debut != '\0') {
      nombreCommandes++;
      Serial.print(F("SEQUENCE "));
      Serial.print(nombreCommandes);
      Serial.print(F(" : "));
      Serial.println(debut);

      if (executerCommandeYeux(debut)) {
        // Couleur appliquee tout de suite, sans toucher au mouvement courant.
      } else if (executerCommandeEmotion(debut, false)) {
        if (mouvementDisponible(debut)) {
          strncpy(dernierMouvement, debut, LONGUEUR_NOM_MOUVEMENT);
          dernierMouvement[LONGUEUR_NOM_MOUVEMENT] = '\0';
          mouvementTrouve = true;
        }
      } else if (estCommandeMouvement(debut)) {
        strncpy(dernierMouvement, debut, LONGUEUR_NOM_MOUVEMENT);
        dernierMouvement[LONGUEUR_NOM_MOUVEMENT] = '\0';
        mouvementTrouve = true;
      } else {
        executerCommande(debut);
      }
    }

    debut = virgule == NULL ? NULL : virgule + 1;
  }

  if (debut != NULL && *debut != '\0')
    afficher(F("ATTENTION : sequence limitee a 16 commandes."));

  if (mouvementTrouve) {
    if (mouvementEnCours || sequenceEnCours)
      demanderMouvementPrioritaire(dernierMouvement);
    else {
      stopDemande = false;
      if (mouvementDisponible(dernierMouvement))
        rejouerMouvement(dernierMouvement);
      else
        executerCommande(dernierMouvement);
    }
  }
}

void executerMouvementPrioritaireSiPret() {
  if (!mouvementPrioritaireEnAttente || mouvementEnCours || sequenceEnCours
      || enregistrementEnCours) return;

  char prochain[LONGUEUR_NOM_MOUVEMENT + 1];
  strncpy(prochain, mouvementPrioritaire, sizeof(prochain));
  prochain[sizeof(prochain) - 1] = '\0';
  mouvementPrioritaireEnAttente = false;
  stopDemande = false;

  Serial.print(F("PRIORITE : lancement immediat de "));
  Serial.println(prochain);
  if (mouvementDisponible(prochain))
    rejouerMouvement(prochain);
  else
    executerCommande(prochain);
}

void traiterCaractere(char c) {
  if (c == '[') {
    dansBalise = true;
    longueurCommande = 0;
    return;
  }

  if (!dansBalise) return;

  if (c == ']') {
    commande[longueurCommande] = '\0';
    dansBalise = false;
    // Copie locale : pendant un mouvement, une nouvelle reception BLE peut
    // reutiliser le tampon global sans abimer la sequence deja en cours.
    char sequence[LONGUEUR_COMMANDE_MAX];
    strncpy(sequence, commande, sizeof(sequence));
    sequence[sizeof(sequence) - 1] = '\0';
    longueurCommande = 0;
    executerSequenceCommandes(sequence);
    return;
  }

  if (longueurCommande < sizeof(commande) - 1) {
    if (c >= 'a' && c <= 'z') c -= 32; // accepte aussi les minuscules
    commande[longueurCommande++] = c;
  } else {
    dansBalise = false; // balise trop longue : abandon propre
    longueurCommande = 0;
  }
}

void afficherAide() {
  afficher(F("--- MECCANOID G15 PRET ---"));
  Serial.print(F("Version programme : ")); Serial.println(VERSION_PROGRAMME);
  afficher(F("CONSOLE USB : 9600 bauds. Ecris une balise avec crochets, puis Envoyer."));
  afficher(F("Calibration : [LIM] libere les bras ; [POSITIONS] affiche les angles."));
  afficher(F("Memoriser une pose : [MEM_REPOS]"));
  afficher(F("[MEM_BONJOUR] [MEM_ETONNEMENT]"));
  afficher(F("Enregistrer : [ENREGISTRE_NOM], puis [FIN_ENREGISTREMENT]"));
  afficher(F("Lire : [NOM] - Voir les noms : [LISTE_MOUVEMENTS]"));
  afficher(F("Supprimer : [SUPPRIME_NOM] - exemples : [SUPPRIME_SALUT] ou [SUPPRIME_1]"));
  afficher(F("Effacement immediat du nom exact, sans confirmation dans la console."));
  afficher(F("Suppression : terminer la lecture ou l'enregistrement avant l'envoi."));
  afficher(F("[STOP] arrete la lecture ; pendant un enregistrement, tente de le sauvegarder."));
  afficher(F("Tests directs : [DIAGNOSTIC] [TEST_GAUCHE] [TEST_DROIT]"));
  afficher(F("Gestes : [REPOS] [BONJOUR] [ETONNEMENT] [STOP]"));
  afficher(F("Yeux : ROUGE ORANGE JAUNE AMBRE VERT TURQUOISE CYAN BLEU VIOLET ROSE BLANC OFF"));
  afficher(F("Utilise par exemple [YEUX_ORANGE] ou [YEUX_VIOLET]."));
  afficher(F("Emotions : [JOIE] [TRISTESSE] [COLERE] [PEUR] [SURPRISE] [CALME]"));
  afficher(F("[TENDRESSE] [CURIOSITE] [CONFUSION] [FIERTE] [DEGOUT] [NEUTRE]"));
  afficher(F("Balise composee : yeux immediats, dernier mouvement prioritaire (max 16)"));
  afficher(F("Exemple : [SALUT,YEUX_BLEU,COLERE]"));
  afficher(F("BLE accepte aussi sans crochets : JOIE,SALUT,TRISTESSE"));
}

void setup() {
  Serial.begin(9600);

  // Serveur BLE UART compatible avec le Web Bluetooth de Bubblai.
  BLEDevice::init(NOM_BLUETOOTH);
  BLEServer *serveurBLE = BLEDevice::createServer();
  serveurBLE->setCallbacks(new RappelsServeurBLE());

  BLEService *serviceBLE = serveurBLE->createService(SERVICE_UUID);

  caracteristiqueTx = serviceBLE->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  caracteristiqueTx->addDescriptor(new BLE2902());

  BLECharacteristic *caracteristiqueRx = serviceBLE->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  caracteristiqueRx->setCallbacks(new RappelsReceptionBLE());

  serviceBLE->start();
  serveurBLE->getAdvertising()->start();
  Serial.println(F("BLE actif : nom Mecanoid, service UART compatible Bubblai."));

  // Sur ESP32, EEPROM.begin() est obligatoire avant get/put.
  EEPROM.begin(sizeof(MemoirePoses));
  memoireMouvementsPrete = LittleFS.begin(true);
  if (!memoireMouvementsPrete)
    afficher(F("ERREUR : memoire flash des mouvements indisponible."));

  // On regroupe les ordres d'une meme chaine avant de l'actualiser.
  gauche0.setAutoUpdate(false);
  gauche1.setAutoUpdate(false);
  droite0.setAutoUpdate(false);
  droite1.setAutoUpdate(false);
  yeux.setAutoUpdate(false);

  chargerMemoire();
  afficher(F("Recherche des modules Meccanoid..."));
  actualiserChaines(64);
  eteindreLedsServomoteurs();

  if (!brasConnectes()) {
    afficher(F("ATTENTION : moins de 4 servos de bras detectes."));
  }

  entrerEnLim(); // securite : aucun mouvement automatique au demarrage
  afficherAide();
  listerMouvements();

  if ((memoire.posesValides & TOUTES_POSES_OK) == TOUTES_POSES_OK)
    afficher(F("Les 3 poses sont deja memorisees."));
  else
    afficher(F("Calibration necessaire : commence par placer la pose REPOS."));
}

void loop() {
  ecouterEntrees();
  executerMouvementPrioritaireSiPret();
  actualiserChaines();
  traiterNouveauxServomoteurs();
  enregistrerEchantillonSiNecessaire();

  // Apres une deconnexion, rendre de nouveau Mecanoid visible dans Bubblai.
  if (!appareilConnecte && ancienEtatConnexion) {
    delay(250);
    BLEDevice::getServer()->startAdvertising();
    ancienEtatConnexion = appareilConnecte;
    Serial.println(F("BLE : publicite redemarree."));
  }
  if (appareilConnecte && !ancienEtatConnexion) {
    ancienEtatConnexion = appareilConnecte;
  }
}
