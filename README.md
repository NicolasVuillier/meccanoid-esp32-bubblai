# Meccanoid G15 + ESP32 + Bubblai

Par Nicolas Vuillier. Un robot auquel on apprend des gestes à la main, puis que l'on anime depuis une conversation avec une IA.

Ce projet contient le programme Arduino pour ESP32 qui pilote quatre servomoteurs de bras Meccanoid G15 V1 et les yeux RGB. Il reçoit des commandes par USB ou Bluetooth Low Energy, enregistre des mouvements nommés dans sa mémoire et les rejoue. Bubblai fournit la conversation et la voix ; l'ESP32 exécute les commandes et les trajectoires.

- [Site de présentation Bubblai](https://www.bubblai.fr/)
- [Ouvrir l'application Bubblai](https://app.bubblai.fr/)

## Fichiers

Ouvrir `MeccanoidG15BrasLeds/MeccanoidG15BrasLeds.ino` dans Arduino IDE. Garder `MeccanoidESP32.h` et `MeccanoidESP32.cpp` dans le même dossier : c'est l'adaptation locale nécessaire au programme. Il n'est pas nécessaire d'installer une autre bibliothèque Meccanoid.

Version embarquée : **14 suppression des mouvements**. Les trois fichiers source sont repris sans modification du programme retrouvé dans le travail de programmation. L'auteur a signalé le bon fonctionnement de sa dernière version ; cette archive n'a pas été recompilée ni testée sur un robot pendant sa préparation.

## Matériel et branchements

- ESP32 DevKit / WROOM-32 classique avec BLE, câble USB de données.
- Quatre servomoteurs intelligents Meccanoid, deux par bras.
- Module yeux RGB Meccanoid.
- Alimentation adaptée aux modules et câblage avec masse commune.

Le tableau décrit les signaux du montage final, et non la position physique des contacts sur un connecteur.

| Liaison | Destination, dans l'ordre physique |
| --- | --- |
| GPIO25 | Signal bras gauche : premier servo ID0, deuxième servo ID1, puis yeux ID2 |
| GPIO27 | Signal bras droit : premier servo ID0, puis deuxième servo ID1 |
| GND ESP32 | Masse commune avec l'alimentation des modules |
| USB ESP32 | Alimentation de la carte et console série |
| Alimentation des modules | Alimentation des servos et des yeux, séparée de l'USB |

Les yeux sont **en dernier dans la chaîne gauche**. GPIO26 n'est pas utilisé dans cette version. Dans le logiciel, G0/D0 correspondent au deuxième servo physique (ID1) et G1/D1 au premier (ID0) : cette inversion est volontaire et correspond au montage mécanique de l'auteur.

Couper les alimentations pour modifier le câblage. Identifier signal, positif et masse avant de brancher ; ne pas se fier uniquement à la couleur des fils. Ne pas alimenter les moteurs depuis le 3,3 V de l'ESP32. Le montage de travail utilise une alimentation moteurs séparée de 6 V et une carte alimentée en USB : ne jamais reporter ces 6 V sur un GPIO ou sur le 3,3 V. Vérifier la tension autorisée par les modules et la carte réellement utilisés.

Le bus Meccanoid est bidirectionnel : les GPIO de l'ESP32 doivent rester à des niveaux compatibles 3,3 V, y compris lors des réponses des modules. Vérifier les niveaux et les résistances de rappel de son montage ; une adaptation de niveau peut être nécessaire. Ce tableau n'est pas un schéma électrique de validation de cette interface. La [bibliothèque d'origine](https://github.com/alexfrederiksen/MeccanoidForArduino) renvoie au protocole Smart Module pour les spécifications du bus.

## Installer dans Arduino IDE

1. Installer Arduino IDE et le paquet **esp32 by Espressif Systems** depuis le gestionnaire de cartes, en suivant la [documentation Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
2. Ouvrir le fichier `.ino` avec les deux fichiers associés dans son dossier.
3. Choisir la carte correspondant à son ESP32 ; pour un DevKit WROOM classique, le profil générique est généralement **ESP32 Dev Module**. Sélectionner le port USB de la carte.
4. Conserver un partitionnement comportant de l'espace pour le système de fichiers. Le code utilise LittleFS pour les mouvements et EEPROM émulée pour les poses. Éviter l'effacement complet de la flash si l'on souhaite conserver les apprentissages.
5. Compiler, puis téléverser. Les bibliothèques BLE, EEPROM, Preferences et LittleFS sont fournies par le paquet ESP32. Le code contient une adaptation liée à ESP32 3.3.11 ; la compatibilité avec toutes les versions du paquet n'a pas été vérifiée.
6. Ouvrir le moniteur série à **9600 bauds** et réinitialiser la carte pour lire l'aide. Envoyer les commandes **entre crochets**.

Au démarrage, les bras sont libres en mode LIM, avec les LED des moteurs éteintes. Ils ne doivent pas effectuer de geste automatique. Soutenir les bras si leur propre poids les fait tomber.

## Premier essai par USB

1. Envoyer `[DIAGNOSTIC]` et vérifier les quatre servos.
2. Envoyer `[YEUX_ROUGE]`, puis `[YEUX_BLEU]`.
3. Envoyer `[LIM]`, placer doucement les bras dans une position de repos et envoyer `[MEM_REPOS]`.
4. Dégager l'espace autour du robot avant `[TEST_GAUCHE]`, puis `[TEST_DROIT]`.

`[STOP]` interrompt une lecture, mais ne coupe pas l'alimentation et peut maintenir la position. `[LIM]`, une fois le mouvement arrêté, libère les bras. Garder une coupure d'alimentation accessible pendant les essais.

## Apprendre et rejouer un mouvement

Envoyer `[ENREGISTRE_SALUT]`, déplacer les bras doucement à la main, puis envoyer `[FIN_ENREGISTREMENT]`. Envoyer ensuite `[SALUT]` pour le rejouer. Les LED moteurs sont rouges pendant l'enregistrement et vertes pendant la lecture ; les yeux restent indépendants.

Le nom contient 1 à 24 caractères parmi A–Z, 0–9 et `_`, sans espace ni accent. Éviter les noms de commandes et les préfixes `ENREGISTRE_`, `SUPPRIME_` et `MEM_`. Un nom simple comme `SALUT` ou `1` convient. Enregistrer sous un nom existant remplace sa trajectoire.

Les mouvements sont conservés après extinction dans LittleFS. La capture vise un intervalle de 50 ms, avec les limites du bus ; la durée maximale est 60 secondes et le tampon contient au maximum 1 200 échantillons. Un effacement de flash, un changement de partitions ou le formatage automatique après un échec de montage LittleFS peut supprimer ces données.

| Commande | Action |
| --- | --- |
| `[LISTE_MOUVEMENTS]` | Afficher les noms sauvegardés dans la console |
| `[SUPPRIME_SALUT]` | Effacer uniquement le mouvement SALUT |
| `[SUPPRIME_1]` | Effacer uniquement le mouvement 1 |
| `[STOP]` | Arrêter une lecture ; pendant une capture, tenter de la sauvegarder |
| `[POSITIONS]` | Afficher les quatre angles |
| `[MEM_REPOS]`, `[MEM_BONJOUR]`, `[MEM_ETONNEMENT]` | Mémoriser les poses placées à la main |
| `[REPOS]`, `[BONJOUR]`, `[ETONNEMENT]` | Utiliser les poses apprises ; BONJOUR et ETONNEMENT privilégient une trajectoire du même nom si elle existe |

La suppression est immédiate dans la console, sans confirmation, et refusée pendant une action ou lorsqu'un mouvement prioritaire attend. Terminer l'action avant de recommencer. Pour une suppression demandée oralement, l'agent doit demander confirmation avant d'envoyer la balise. Les autres mouvements et les poses sont conservés.

## Yeux et émotions

Couleurs disponibles : `[YEUX_ROUGE]`, `[YEUX_ORANGE]`, `[YEUX_JAUNE]`, `[YEUX_AMBRE]`, `[YEUX_VERT]`, `[YEUX_TURQUOISE]`, `[YEUX_CYAN]`, `[YEUX_BLEU]`, `[YEUX_VIOLET]`, `[YEUX_ROSE]`, `[YEUX_BLANC]`, `[YEUX_OFF]`.

| Émotion | Couleur des yeux |
| --- | --- |
| `[JOIE]` | Jaune |
| `[TRISTESSE]` | Bleu |
| `[COLERE]` | Rouge |
| `[PEUR]` | Violet |
| `[SURPRISE]` | Blanc |
| `[CALME]` | Cyan |
| `[TENDRESSE]` | Rose |
| `[CURIOSITE]` | Turquoise |
| `[CONFUSION]` | Orange |
| `[FIERTE]` | Ambre |
| `[DEGOUT]` | Vert |
| `[NEUTRE]` | Blanc atténué |

Une émotion change les yeux et joue aussi le mouvement du même nom s'il a été enregistré. Par exemple, apprendre `[ENREGISTRE_JOIE]` permet ensuite à `[JOIE]` de combiner lumière et geste. Sans mouvement associé, seuls les yeux changent.

Pendant une lecture, les yeux peuvent changer et un nouveau mouvement peut remplacer le précédent. Les commandes composées sont acceptées, par exemple `[SALUT,YEUX_BLEU,COLERE]`, avec un maximum de 16 éléments ; elles ne constituent pas une file garantissant la lecture complète de tous les gestes. Pour débuter, envoyer une balise à la fois. Pendant un enregistrement, seules la fin d'enregistrement et STOP sont traitées en plus de la branche de suppression, qui refuse l'effacement.

## Connecter Bubblai

Ouvrir [Bubblai](https://app.bubblai.fr/) dans Chrome, activer le Bluetooth de l'appareil, puis utiliser la bulle **Communication** en Bluetooth BLE. Déclencher la connexion et sélectionner **Mecanoid** dans la fenêtre du navigateur. Le nom est volontairement écrit sans deuxième « c » dans le firmware.

La connexion utilise le BLE/GATT intégré à l'ESP32 et un service UART. Aucun module HC-05 ou HC-06 n'est nécessaire. Si les réglages de la bulle demandent les UUID, utiliser :

| Paramètre | Valeur |
| --- | --- |
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX : écriture vers le robot | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX : notifications | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

Envoyer d'abord `[YEUX_BLEU]` depuis la communication pour vérifier le chemin complet. Le firmware accepte aussi du texte de commandes sans crochets en BLE, pour les intégrations qui retirent les délimiteurs. Utiliser les crochets dans les instructions destinées à l'agent et dans la console USB.

Web Bluetooth requiert un contexte sécurisé HTTPS et une action de l'utilisateur pour choisir l'appareil. Sous Linux, la documentation Chrome indique d'activer **Experimental Web Platform features** dans `chrome://flags/#experimental-web-platform-features`, puis de relancer le navigateur. La disponibilité varie selon le système : voir la [documentation officielle Web Bluetooth](https://developer.chrome.com/docs/capabilities/bluetooth).

Pour la démonstration vocale, activer aussi les capacités Voix ou Realtime de l'agent et l'autorisation du microphone. Les instructions de l'agent doivent lui donner les commandes ci-dessus et les mouvements réellement appris. Ne pas lui annoncer une bibliothèque de gestes inexistante. La parole et son intégration aux balises sont gérées par Bubblai ; ce programme n'embarque ni LLM ni synthèse vocale, et ne garantit pas une synchronisation mot à mot entre voix et moteurs.

Les libellés détaillés des réglages Realtime n'ont pas été vérifiés dans l'interface pendant cette préparation. Le texte exact du SKILL.md de la démonstration précédente n'a pas été retrouvé et n'est pas remplacé ici par une reconstitution.

## Dépannage

- `MeccanoidESP32.h` introuvable : remettre les deux fichiers associés à côté du `.ino`.
- `BLEDevice.h` introuvable : vérifier le paquet et la sélection d'une carte ESP32 compatible BLE.
- Rien dans la console : 9600 bauds, bon port USB, câble de données, puis bouton Reset.
- Yeux muets : vérifier leur position ID2 après les deux servos de gauche, leur alimentation et envoyer une couleur explicite.
- Servo absent : vérifier alimentation, masse commune, connecteurs et ordre des modules avec `[DIAGNOSTIC]`.
- Connexion BLE impossible : fermer l'autre application éventuellement connectée, vérifier Chrome/HTTPS/Bluetooth, puis reconnecter.
- Geste absent : lire la console USB et `[LISTE_MOUVEMENTS]` ; les messages détaillés sont principalement imprimés sur USB, la présence d'une caractéristique TX ne garantit pas leur remontée dans Bubblai.

## Licence et crédits

Le programme principal et cette documentation sont proposés sous licence MIT : voir `LICENSE`. Les fichiers `MeccanoidESP32.h` et `.cpp` dérivent de [MeccanoidForArduino par Alex Frederiksen](https://github.com/alexfrederiksen/MeccanoidForArduino) ; leur attribution et la permission d'utilisation figurant dans leur en-tête sont conservées. Le dépôt amont consulté ne présente pas de fichier LICENSE distinct : la licence MIT de ce projet ne réattribue pas les droits sur le code d'origine de cette bibliothèque.

Ce dépôt porte sur le firmware et son mode d'emploi. L'application Bubblai reste un projet distinct, accessible par les liens ci-dessus.
