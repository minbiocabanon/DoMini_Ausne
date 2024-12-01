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
#define RELAY 4           // Commande Relais qui ouvrer/ferme la porte de garage sur PD4 - Pin6
#define SPARE_14 14       // Commande du 2eme Relais PC0 - Pin23

#define IN_DOOR_OPEN 5    // Entrée capteur magnétique pour porte ouverte sur PD5 - Pin11
#define IN_DOOR_CLOSE 15  // Entrée capteur magnétique pour porte fermée sur PC1 - Pin24

#define CLOSE 0
#define OPEN 1

// Declaration des durées des timers / timeout
#define TIMEOUT_DOOR_MOVE			30000		// 30sec in milliseconds. A expiration, la machine d'état vérifie l'état de la porte



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

enum door_state
{
  DOOR_CLOSED,
  DOOR_OPENED,
  DOOR_CLOSING,
  DOOR_OPENING,
  DOOR_ERROR,
  DOOR_UNKNOWN_STATE
} Door_State;


// prototype fonctions
void tache_gestion_radio(void);
void setup(void);
int Trt_msg_GAR(char *message);
void Timer100ms(void);
void SendRadioData(void);
int TrtReceptRadio(void);
void clignote_led(void);
void tache_scheduler();
void tache_gestion_SM(void);
void status(void);
door_state get_door_state(void);
void manage_door(void);

// variable pour timer
unsigned long TimeOutDoorMove = 0;
int nblipled;

// flag
boolean bTimeToSendRadioMsg = false;
boolean bflag_consigne_en_attente = false;
boolean bflag_Timeout_door_expired = true;

// chaine pour l'émission RF12
char buffer_recept_rf12[66] = "";

//Variables specifiques porte de garage
const int LED = 6; // PD6 - pin12 - DIO Port 3
int nconsigne_porte = -1;
boolean bFlagMoteurPosOK = false;
door_state nretinfoporte = DOOR_OPENED; // on met DOOR_OPENED par défaut, ainsi, si le serveur voit que la porte est ouverte par défaut, il renverra une consigne CLOSE pour forcer la fermeture
int ncpt_impulsion = 0;  // compteur du nombre d'impulsions envoyées pour mettre la porte de garage dans l'état demandé par la consigne

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

  sprintf(payload, "$%s,%d\r\n", stNum, nretinfoporte);

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
  Serial.println(">> Generating impulsion << ");
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
  Serial.println("--- Trt_msg_GAR ---");
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

  // si le message radio est correcte -> message radio ($GAR L/)
  if (TrtReceptRadio() == 1)
  {
    Serial.println("Envoi msg radio...");
    // on execute la consigne du message recu
    Trt_msg_GAR(buffer_recept_rf12);
    // on indique qu'une consigne est à traiter
    bflag_consigne_en_attente = true;
    // on fait clignoter la led pour le debug
    clignote_led();
  }
}

//----------------------------------------------------------------------
//!\brief           Fonction qui gère les consignes et impulsion 
//----------------------------------------------------------------------
void manage_door()
{
  // Si Timeout ouverture/fermeture porte est expiré
  if (bflag_Timeout_door_expired == true)
  {
    Serial.println("\t bflag_Timeout_door_expired");

    door_state doorstate = get_door_state();

    // Si la porte est en position fermée et que cet état correspond à consigne demandée
    if( (doorstate == DOOR_CLOSED) && (nconsigne_porte == DOOR_CLOSED))
    {
      // on passe la machine d'état dans l'état correspondant
      SM_state = SM_WAIT_CLOSE;
      // on resette le nb d'impulsions
      ncpt_impulsion = 0;
    }
    // Si la porte est en position ouverte et que c'est la consigne demandée
    else if ((doorstate == DOOR_OPENED) && (nconsigne_porte == DOOR_OPENED))
    {
      // on passe la machine d'état dans l'état correspondant
      SM_state = SM_WAIT_OPEN;
      // on resette le nb d'impulsions
      ncpt_impulsion = 0;
    }
    // Sinon, c'est que la porte est dans un état intermédiaire ou bien en train de bouger.
    else
    {
      // si on a déjà fait plus de 2 impulsions auparavent et que la porte n'est pas dans une position ouverte/fermée au bout du timetout, c'est que la porte est en erreur
      // rappel sur le fonctionnement du controleur de la porte de garage :
      // Une impulsion permet de  :
      //    - Ouvrir la porte si elle est fermée.
      //    - Fermer la porte si elle est ouverte.
      //    - Stoppe la porte si elle en train de s'ouvrir ou de se fermer.
      //    - Change de sens si elle est arrêtée entre deux états.
      //
      //  Par conséquent, avec 3 impulsions max on doit retrouver un état connu.
      if(ncpt_impulsion > 2)
      {
        Serial.println("\t ncpt_impulsion > 2");
        SM_state = SM_ERROR;
        ncpt_impulsion = 0;
      }
      else
      {
        Serial.println("Impulsion_relais(500)");
        // on genere une impulsion sur le relais qui pilote la porte de garage
        impulsion_relais(500);
        // armer le timer avec la durée d'ouverture/fermeture de la porte
        TimeOutDoorMove = millis();
        Serial.println(" TimeOutDoorMove = millis();");
        // reset du flag pour ne pas revenir ici systematiquement
        bflag_Timeout_door_expired = false;
        // on incremente le compteur d'impulsion
        ncpt_impulsion++;
        // attendre quelques secondes que l'aimant s'eloigne du capteur
        delay(2000);
      }
    }
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
    Serial.println("--- SM_INIT ---");
    // rien a initialiser ???
    SM_state = SM_WAIT;
    break;

  case SM_WAIT:
    // Si on a recu un msg radio avec le bon entete
    if (TrtReceptRadio() == 1)
    {
      Serial.println("--- SM_WAIT ---");
      // on execute la consigne du message recu
      Trt_msg_GAR(buffer_recept_rf12);
      // on passe a l'etat suivant
      SM_state = SM_ACTION;
      // on fait clignoter la led pour le debug
      clignote_led();
    }

    break;

  case SM_ACTION:
    // Serial.println("--- SM_ACTION ---");
    // on appelle la fonction qui gère l'application de la consigne
    manage_door();
    break;

  case SM_WAIT_CLOSE:
    Serial.println("--- SM_WAIT_CLOSE ---");
    // capteur fin de course CLOSE = true ?
    // si oui
    //  preparer buffer avec etat CLOSE
    nretinfoporte = DOOR_CLOSED;
    SM_state = SM_SEND_RADIO;

    break;

  case SM_WAIT_OPEN:
    Serial.println("--- SM_WAIT_OPEN ---");
    // capteur fin de course OPEN = true ?
    // si oui
    //  preparer buffer avec etat OPEN
    nretinfoporte = DOOR_OPENED;
    SM_state = SM_SEND_RADIO;
    break;

  case SM_ERROR:
    Serial.println("--- SM_ERROR ---");
    // Erreur de timeout a l'ouverture ou fermeture
    // preparer buffer avec erreur
    SM_state = SM_SEND_RADIO;
    break;

  case SM_SEND_RADIO:
    Serial.println("--- SM_SEND_RADIO ---");
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
//!\brief         Retourne l'état de la porte
//!\return        type door_state
//----------------------------------------------------------------------
door_state get_door_state(void)
{

  Serial.println("lecture infos capteurs");
  // lecture des deux capteurs
  int ncpt_close = digitalRead(IN_DOOR_CLOSE);
  int ncpt_open = digitalRead(IN_DOOR_OPEN);

  Serial.print("\t npt_close : ");
  Serial.println(ncpt_close);
  Serial.print("\t ncpt_open : ");
  Serial.println(ncpt_open);

  // gestion des 4 combinaisons possibles
  // attention logique inverse : GPIO avec PU interne , le capteur fait une mise à la masse.
  if (ncpt_close == 0 && ncpt_open == 1)
    return DOOR_CLOSED;
  else if (ncpt_close == 1 && ncpt_open == 0 )
    return DOOR_OPENED;
  else if (ncpt_close == 0 && ncpt_open == 0)
    return DOOR_ERROR;
  else if (ncpt_close == 1 && ncpt_open == 1)
    return  DOOR_UNKNOWN_STATE;
  else
    return DOOR_ERROR;
}


//----------------------------------------------------------------------
//!\brief           scheduler()
//----------------------------------------------------------------------
void tache_scheduler()
{
	if( ((millis() - TimeOutDoorMove) > TIMEOUT_DOOR_MOVE) && SM_state == SM_ACTION){
		Serial.println("--- Timeout Door move ---");
		bflag_Timeout_door_expired = true;
	}
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
  //configure pin 2 as an input and enable the internal pull-up resistor

  pinMode(IN_DOOR_CLOSE, INPUT_PULLUP);
  pinMode(IN_DOOR_OPEN, INPUT_PULLUP);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);
  pinMode(SPARE_14, INPUT);
  digitalWrite(SPARE_14, LOW);
  pinMode(LED, OUTPUT);

  // TEST
  // allume la led RELAY
  // digitalWrite(RELAY, HIGH);

  // initialisation du module radio RF12
  rf12_initialize(1, RF12_868MHZ, 33);
  // Reconfig du baudrate : 1200 bauds
  rf12_control(0xC6A3);
  Serial.print("\nInit Radio : \n 1200 bauds\n");

  // Initialisaiton des timers
   TimeOutDoorMove = millis();

/* initialize door state*/
  Door_State = DOOR_UNKNOWN_STATE;

  Door_State = get_door_state();

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

