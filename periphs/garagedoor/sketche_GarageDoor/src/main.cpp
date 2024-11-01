//--------------------------------------------------
//! \file     sketche_GarageDoor.ino
//! \brief    Sketch pour le JeeNode : gestion d'une porte de garage motorisee
//! \brief 	Les messages reçus ont ce format:  $GARL,x
//! \brief 	x = 0 -> consigne pour FERMER la porte de garage
//! \brief 	x = 1 -> consigne pour OUVRIR la porte de garage
//! \date     2024-10
//! \author   minbiocabanon
//--------------------------------------------------

#include <Arduino.h>
#include <JeeLib.h>
#include <MsTimer2.h>
#include <math.h>

#define version "JeeNode GarageDoor L"
char stNum[] = "GARL";

#define NBMINUTEVEILLE 300 * 10  // x10 car basé sur Timer100ms
#define TIME_0_TO_100 2 * 10     // Nb de seconde (x10 car basé sur Timer100ms)  que mets le registre à parcourir la totalité de sa course
#define TIMEOUTMOTEUR_SEC 5 * 10 // Nb de seconde (x10 car basé sur Timer100ms) pour le TimeOut à partir duquel on stoppe automatiquement les moteurs
#define NB_BLIP_LED 500          // nb de cycle à attendre entr 2 clignotements de la led status

// Mode radio
// Values 2 and 3 can cause the millisecond time to lose a few interrupts.
// Value 3 can only be used if the ATmega fuses have been set for fast startup, i.e. 258 CK - the default Arduino fuse settings are not suitable for full power down.
#define MODE_RADIO_NORMAL 0
#define MODE_RADIO_IDLE 1
#define MODE_RADIO_STANDBY 2
#define MODE_RADIO_PWR_DOWN 3
#define RADIO_SLEEP 0
#define RADIO_WAKEUP -1

// Declation des variables pour les GPIO
#define CMD_EXT 4   // Commande Bypass vers Ext sur PD5 - Pin11
#define CMD_PC 5    // Commande Bypass vers Ext sur PD4 - Pin6
#define CMD_VMCDF 7 // Commande Bypass vers Ext sur PD7 - Pin13
#define INFO_PC 17  // Commande Bypass vers Ext sur PC4 - Pin27
#define INFO_EXT 26 // FAUX			// Commande Bypass vers Ext sur PC3 - Pin26
#define CLOSE 0
#define OPEN 1


#define RELAY 4 // Commande Relais qui ouvrer/ferme la porte de garage sur PD4 - Pin6

enum state_machine_t
{
  SM_NIL,
  SM_INIT,
  SM_WAIT,
  SM_ACTION,
  SM_WAIT_OPEN,
  SM_WAIT_CLOSE,
  SM_ERROR,
  SM_SEND_RADIO
} SM_state;

// prototype fonctions
void tache_gestion_radio(void);
void setup(void);
int Trt_msg_GAR(char *message);
void SetVMC(int consigne_VMC);
void Timer100ms(void);
void CaptureSensor(void);
void SendRadioData(void);
int TrtReceptRadio(void);
void SetBypass(int consigne_bypass);
void clignote_led(void);

// variable pour timer
unsigned long TicksMsgRad = 0;
unsigned long TicksMoteur = 0;
unsigned long TimeOutMoteur = 0;
boolean bFlagMoteurPosOK = false;
int nblipled;

// flag
boolean bTimeToSendRadioMsg = false;
boolean bflag_consigne_en_attente = false;

// chaine pour l'émission RF12
char buffer_recept_rf12[66] = "";

//Variables specifiques porte de garage
const int LED = 6; // PD6 - pin12 - DIO Port 3
int nconsigne_porte = NULL;
bool bretinfoporte = OPEN; // on met OPEN par défaut, ainsi, si le serveur voit que la porte est ouverte par défaut, il renverra une consigne CLOSE pour forcer la fermeture

//----------------------------------------------------------------------
//!\brief           fait clignoter la led une fois
//!\return        -
//----------------------------------------------------------------------
void clignote_led(void)
{
  int i = 0;
  for (i = 0; i < 5; i++)
  {
    // allume la led
    digitalWrite(LED, LOW);
    delay(50);
    // eteint la led
    digitalWrite(LED, HIGH);

    delay(100);
  }
}

//----------------------------------------------------------------------
//!\brief		Timer 100 milliseconde
//!\return		-
//----------------------------------------------------------------------
void Timer100ms(void)
{

  // si le timer moteur est lancé
  if (TicksMoteur != 0)
  {
    // on décrémente le compteur
    TicksMoteur--;
    // debug
    Serial.println(TicksMoteur);
    // Si le timer a expiré, on a du atteindre la position
    if (TicksMoteur == 0)
      bFlagMoteurPosOK = true;
  }

  // on incrémente le compteur de ticks
  TicksMsgRad++;
  // Si le timer a expiré, on set le flag pour envoyer un message radio
  if (TicksMsgRad >= NBMINUTEVEILLE)
  {
    // on set le flag pour envoyer le message radio
    bTimeToSendRadioMsg = true;
    // on reset le compteur
    TicksMsgRad = 0;
  }

  // si le timer antiblocage moteur est lancé
  if (TimeOutMoteur != 0)
  {
    // on décrémente le compteur
    TimeOutMoteur--;
    // Si le timer a expiré, on a du atteindre la position
    if (TimeOutMoteur == 0)
    {
      // on stoppe les moteurs
      digitalWrite(CMD_EXT, LOW);
      digitalWrite(CMD_PC, LOW);
    }
  }
}

//----------------------------------------------------------------------
//!\brief		Transmet les données du buffer radio
//!\param	 	Buffer à envoyer
//!\return		?
//----------------------------------------------------------------------
void SendRadioData(void)
{
  // **************************************************
  // *	Compilation des données dans un message
  // **************************************************

  // conversion du string en buffer pour la radio
  char payload[29] = "";

  // conversion du float en INT x100

  sprintf(payload, "$%s,%i\r\n", stNum, bretinfoporte);

  // **************************************************
  // *	Emission des infos  sur la radio
  // **************************************************

  // DEBUG
  Serial.println("\nBuffer radio a emettre: ");
  for (byte i = 0; i <= strlen(payload); i++)
    Serial.print(payload[i]);
  Serial.println();

  // A executer régulièrement
  rf12_recvDone();

  // tant que porteuse pas libre
  while (!rf12_canSend())
    // on interroge le module
    rf12_recvDone();
  // porteuse libre, on envoie le message
  rf12_sendStart(0, payload, strlen(payload));
  Serial.print("\n\rMessage envoye OK\n\r");

  // attente de la fin d'émission radio
  rf12_sendWait(MODE_RADIO_IDLE);
}

//----------------------------------------------------------------------
//!\brief           Gestion de la réception des messages depuis la radio
//!\return        1 sir message reçu = VRL sinon 0
//----------------------------------------------------------------------
int TrtReceptRadio(void)
{

  int RetVal = 0;
  // si on a reçu un message valide
  if (rf12_recvDone() && (rf12_crc == 0))
  {

    // on copie le buffer reçu dans un buffer de travail
    memcpy(buffer_recept_rf12, (void *)rf12_data, rf12_len);
    // terminateur de chaine
    // à tester ???
    buffer_recept_rf12[rf12_len] = 0;

    // DEBUG
    Serial.println("\n Message recu");
    Serial.print(buffer_recept_rf12);
    // on verifie que l'entet du message est la bonne, en théorie c'est toujours le cas si on utilise les ID radio
    if (buffer_recept_rf12[1] == 'G' && buffer_recept_rf12[2] == 'A' && buffer_recept_rf12[3] == 'R' && buffer_recept_rf12[4] == 'L')
    {
      // on affiche le message reçu
      // Serial.println(buffer_recept_rf12);
      // si l'émetteur du message veut un ACK, on lui envoie!
      if (RF12_WANTS_ACK)
      {
        rf12_sendStart(RF12_ACK_REPLY, 0, 0);
        Serial.println("\n ACK sent");
      }
      // on retourne 1
      RetVal = 1;
    }
    else
    {
      Serial.println("Msg recu non reconnu");
      // on retourne 0
    }
  }
  return (RetVal);
}


//----------------------------------------------------------------------
//!\brief          Genere une impulsion sur la sortie relais qui pilote la carte moteur de la porte de garage
//!\param[in]    	duree : duree de l'impulsion en millisecondes
//----------------------------------------------------------------------
void impulsion_relais(int duree)
{
  // on active le relay qui pilote la porte de garage
  digitalWrite(RELAY, HIGH);
  delay(duree);
  // on desactive le relay qui pilote la porte de garage
  digitalWrite(RELAY, LOW);

}

//----------------------------------------------------------------------
//!\brief          Traite le message des consignes VMC et bypass
//!\param[in]    	message reçu
//----------------------------------------------------------------------
int Trt_msg_GAR(char *message)
{
  // Déclaration des variables
  char *entete, *i;
  // on dépouille le message reçu
  entete = strtok_r(message, ",", &i);
  nconsigne_porte = atoi(strtok_r(NULL, ",", &i));

  return (1);
}

//----------------------------------------------------------------------
//!\brief           Tache de fond qui gère la radio
//----------------------------------------------------------------------
void tache_gestion_radio(void)
{

  // si le message radio est correcte -> message radio ($GAR L/)   ET que l'utilisateur n'est pas en train de trifouiller les boutons . on ne traite pas la consigne reçu par radio si l'utilisateur s'apprete à changer la consigne (passage en mode manuel)
  if (TrtReceptRadio() == 1)
  {
    // on execute la consigne du message recu
    Trt_msg_GAR(buffer_recept_rf12);
    // on indique qu'une consigne est à traiter
    bflag_consigne_en_attente = true;
    // on fait clignoter la led pour le debug
    clignote_led();
  }
}

//----------------------------------------------------------------------
//!\brief           Tache de fond qui gère la machine d'etat
//----------------------------------------------------------------------
void tache_gestion_SM(void)
{

  switch (SM_state)
  {
  case SM_NIL:
    // We should not go there !
    // go to initial state of SM
    SM_state = SM_INIT;
    break;

  case SM_INIT:
    // rien a initialiser ???
      SM_state = SM_WAIT;
    break;

  case SM_WAIT:
    if (TrtReceptRadio() == 1)
    {
      // on execute la consigne du message recu
      Trt_msg_GAR(buffer_recept_rf12);
      // on passe a l'etat suivant
      SM_state = SM_ACTION;
      // on fait clignoter la led pour le debug
      clignote_led();
    }

    break;

  case SM_ACTION:

    if( nconsigne_porte == OPEN)
    {
      // si consigne ouverte
      // regarder si porte n'est pas deja ouverte
      // on genere un impulsion sur le relais qui pilote la porte de garage
      impulsion_relais(1000);
      // attendre quelques secondes que l'aimant s'eloigne du capteur
      delay(2000);
      // attendre quelques secondes que l'aimant s'eloigne du capteur
      // armer timeout
      // on passe a l'etat suivant
      SM_state = SM_WAIT_OPEN;
    }

    if( nconsigne_porte == CLOSE)
    {
      // si consigne fermeture
      // regarder si porte n'est pas deja ouverte
      // agir sur GPIO
      // on genere un impulsion sur le relais qui pilote la porte de garage
      impulsion_relais(1000);
      // attendre quelques secondes que l'aimant s'eloigne du capteur
      delay(2000);
      // armer timeout
      // on passe a l'etat suivant
      SM_state = SM_WAIT_CLOSE;
    }
    break;

  case SM_WAIT_CLOSE:
    // capteur fin de course CLOSE = true ?
    // si oui
    //  preparer buffer avec etat CLOSE
    bretinfoporte = CLOSE;
    SM_state = SM_SEND_RADIO;

    break;

  case SM_WAIT_OPEN:
    // capteur fin de course OPEN = true ?
    // si oui
    //  preparer buffer avec etat OPEN
    bretinfoporte = OPEN;
    SM_state = SM_SEND_RADIO;
    break;

  case SM_ERROR:
    // Erreur de timeout a l'ouverture ou fermeture
    // preparer buffer avec erreur
    SM_state = SM_SEND_RADIO;
    break;

  case SM_SEND_RADIO:
    SendRadioData();
    SM_state = SM_WAIT;
    break;

  default:
    // We should not go there !
    // return to initial state of SM
    SM_state = SM_INIT;
    break;
  }
}

//----------------------------------------------------------------------
//!\brief           fait clignoter une led en fond de tache pour donner un signe de vie
//!\return        -
//----------------------------------------------------------------------
void status(void)
{

  nblipled++;
  if (nblipled > NB_BLIP_LED)
  {
    // allume la led
    digitalWrite(LED, HIGH);
    delay(50);
    // eteint la led
    digitalWrite(LED, LOW);
    nblipled = 0;
  }
}

//----------------------------------------------------------------------
//!\brief           scheduler()
//----------------------------------------------------------------------
void tache_scheduler()
{

// 	if( (millis() - taskGetGPS) > PERIOD_GET_GPS){
// 		taskGetGPS = millis();
// 		MyFlag.taskGetGPS = true;	
// 	}
	
// 	if( (millis() - taskTestGeof) > PERIOD_TEST_GEOFENCING){
// 		taskTestGeof = millis();
// 		MyFlag.taskTestGeof = true;
// 	}
	
// 	if( (millis() - taskGetLiPo) > PERIOD_LIPO_INFO){
// 		taskGetLiPo = millis();
// 		MyFlag.taskGetLiPo = true;
// 	}	
	
// 	if( (millis() - taskCheckSMS) > PERIOD_CHECK_SMS){
// 		taskCheckSMS = millis();
// 		MyFlag.taskCheckSMS = true;
// 	}
	
// 	if( (millis() - taskAutoTestSMS) > PERIOD_AUTOTEST_SMS){
// 		taskAutoTestSMS = millis();
// 		SM_autotestsm = SMAT_START;
// 	}	
	
// 	if( (millis() - taskCheckFlood) > PERIOD_CHECK_FLOOD){
// 		taskCheckFlood = millis();
// 		MyFlag.taskCheckFlood = true;
// 	}	
	
// 	if( (millis() - taskStatusSMS) > PERIODIC_STATUS_SMS){
// 		taskStatusSMS = millis();
// 		MyFlag.taskStatusSMS = true;
// 	}

// 	if( (millis() - taskGetAnalog) > PERIOD_READ_ANALOG){
// 		taskGetAnalog = millis();
// 		MyFlag.taskGetAnalog = true;
// 	}	

// 	if( (millis() - taskCheckInputVoltage) > PERIOD_CHECK_ANALOG_LEVEL){
// 		taskCheckInputVoltage = millis();
// 		MyFlag.taskCheckInputVoltage = true;
// 	}

// 	if( ((millis() - TimeOutAutotestSMS) > TIMEOUT_AUTOTEST_SMS) && SM_autotestsm != SMAT_NOPE ){
// 		Serial.println("--- Autotest SMS : Timeout , NO SMS received !! ---");
// 		SM_autotestsm = SMAT_ERROR;
// 	}
	
// 	if( ((millis() - IntervalAutotestSMS) > INTERVAL_AUTOTEST_SMS) && SM_autotestsm == SMAT_RETRYSMS ){
// 		Serial.println("--- Autotest SMS : Interval autotest SMS expired ---");
// 		MyFlag.flagIntervalAutotestSMS = true;
// 	}
	
// 	if( ((millis() - TimeOutSMSMenu) > TIMEOUT_SMS_MENU) && MySMS.menupos != SM_LOGIN && MySMS.menupos != SM_AUTOTEST_SMS ){
// 		MySMS.menupos = SM_LOGIN;
// 		Serial.println("--- SMS Menu manager : Timeout ---");
// 	}
}

//----------------------------------------------------------------------
//!\brief           INITIALISATION
//----------------------------------------------------------------------
void setup()
{
  // init port COM du debug
  Serial.begin(115200);
  // affichage de la version
  Serial.println(version);
  Serial.println();
  Serial.println("Num. de Jeenode : ");
  Serial.println(stNum);

  // config des GPIO
  //  pinMode(CMD_EXT, OUTPUT);
  //  pinMode(CMD_PC, OUTPUT);
  //  pinMode(CMD_VMCDF, OUTPUT);
  //  pinMode(INFO_PC, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(LED, OUTPUT);

  // TEST
  // allume la led RELAY
  // digitalWrite(RELAY, HIGH);

  // initialisation du module radio RF12
  rf12_initialize(1, RF12_868MHZ, 33);
  // Reconfig du baudrate : 1200 bauds
  rf12_control(0xC6A3);
  Serial.print("\nInit Radio : \n 1200 bauds\n");

  // Configuration d'un timer d'une seconde qui sert de scheduler
  MsTimer2::set(100, Timer100ms); // ms periode -> 1 seconde
  MsTimer2::start();

  /* initialize state machine*/
  SM_state = SM_INIT;

  Serial.println("\nFin SETUP");
  clignote_led();
}

//----------------------------------------------------------------------
//!\brief           BOUCLE PRINCIPAL
//----------------------------------------------------------------------
void loop()
{

  tache_scheduler();

  tache_gestion_SM();

  status();

  delay(10);
}
