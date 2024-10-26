Periph
=====

Ce repertoire contient l'ensemble des sous-projets réalisés sur des modules Jeenodes (Arduino)

Lien utile : http://jeelabs.org/

##Compteur EDF
Module Jeenode relié à la sortie téléinfo du compteur EDF.
Transmet en continu les données de téléinfo (filtrées) au PC serveur.

##Puits canadien
Voir 'sonde temperature' pour le code et les schémas. Ce répertoire ne contient que les feuilles de calculs et formules m'ayant servis à estimer la puissance récupérée par le puits canadien.
Module Jeenode équipé d'un capteur de température et d'humidité SHT11.
Installé dans le conduit du puits canadien, il envoie régulièrement les infos température, humidité, point de rosé et tension batterie.

##Pyranomètre (pour test)
Module Jeenode m'ayant servi à évaluer différentes solutions du pyranomètre (mesure du flux solaire)
installé dehors ou derrière une fenêtre, il envoie régulièrement les infos de flux solaire (bruts, la conversion est effectuée par le PC serveur)

##sonde température
Module Jeenode équipé d'un capteur de température et d'humidité SHT11.
Il envoie régulièrement les infos température, humidité, point de rosé et tension batterie.

##Station météo
Module quasi identique à 'Sonde temperature'.
Le Jeenode est installé dans un boitier extérieur (type ruche).
Il mesure également le flux solaire (pyranomètre).
Les piles sont rechargées par un capteur solaire, le module est autonome en énergie.