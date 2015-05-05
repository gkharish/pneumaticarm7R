#ifndef MAIN_CONTROLEUR_H
#define MAIN_CONTROLEUR_H

/***** DEFINITION DE L'ADRESSAGE DES cardS *****/
/*         BASE_REG_CIODAC16 : CIO-DAC16-I 	*/
/*         BASE_REG_CIODAS64 : CIO-DAS6402/16   */
/************************************************/
#define BASE_REG_CIODAC16	0x200
#define BASE_REG_CIODAS64	0x250

/***** DEFINITION DES INFOS CONCERNANT LA GACHETTE *****/
#define VOIE_PINCE_1        14   // sorties de la card de commande
#define VOIE_PINCE_2        15   // reliees a la pince

#define PRESSION_MAX_NUM 4095  // pression maximale numerique
#define PRESSION_MIN_NUM    0  // pression minimale numerique


//infos d'initialisations des pressions correspondant a la position initiale du bras
#define DELTA_INIT_AXE_1 -1.4
#define DELTA_INIT_AXE_2 -2.5
#define DELTA_INIT_AXE_3  0.0
#define DELTA_INIT_AXE_4  1.8
#define DELTA_INIT_AXE_5  0.2
#define DELTA_INIT_AXE_6 -0.6
#define DELTA_INIT_AXE_7 -0.2

/* Output ports of actuators associated to the ouptput command card (CIODAC16)*/
#define VOIE_1_1  2
#define VOIE_1_2  3
#define VOIE_2_1  0
#define VOIE_2_2  1
#define VOIE_3_1  5
#define VOIE_3_2  4
#define VOIE_4_1  7
#define VOIE_4_2  6
#define VOIE_5_1 12
#define VOIE_5_2 13
#define VOIE_6_1  9
#define VOIE_6_2  8
#define VOIE_7_1 10
#define VOIE_7_2 11

//Ports d'entree des joysticks et du palonnier sur la card d'acquisition (CIODAS64)
#define VOIE_X_1          1
#define VOIE_Y_1	  0
#define VOIE_Z_1          3
#define VOIE_VITESSE_1    2
#define VOIE_X_2	  7
#define VOIE_Y_2	  6
#define VOIE_Z_2	  4
#define VOIE_BOUTTON_A_2  5
#define VOIE_PALONNIER   23
#define VOIE_INUTILISEE  -1

//seuil de prise en compte des mouvements du joystick et du palonnier
#define SEUIL_JOY  0.65
#define SEUIL_PAL  0.25

//Informations concernant la vitesse du mouvement
#define COEF_LENT        0.5    //coefficients de rapidite du
#define COEF_MOYEN       1 	// mouvement
#define COEF_RAPIDE      2

#define VITESSE_LENTE    0
#define VITESSE_MOY      1
#define VITESSE_RAPIDE   2

#define PRESSION_BASE     2.5
#define DEMI_TRAJET     180.0
#define P_ECHANT         10.0  //periode d echantillonnage en ms
#define P_ECHANT_S	 1/P_ECHANT
#define DUREE_GONFLEMENT  4.0     //duree pour effectuer 1/2 trajet (180 deg) en secondes
#define VITESSE_PRESSION (P_ECHANT*PRESSION_BASE)/(DUREE_GONFLEMENT*1000)
#define VITESSE_ANGLE   ( P_ECHANT * DEMI_TRAJET) / (DUREE_GONFLEMENT*1000)

//Infos concernant les Message Queues
#define NB_MAX_MSG      1
#define LONG_MAX_MSG    100

//angles theoriques au repos des axes
#define ANGLE_REPOS_1    0.0
#define ANGLE_REPOS_2    0.0
#define ANGLE_REPOS_3    0.0
#define ANGLE_REPOS_4    0.0
#define ANGLE_REPOS_5    0.0
#define ANGLE_REPOS_6    0.0
#define ANGLE_REPOS_7    0.0

// Rapports mecaniques entre rotation des axes des s et rotation reelle des articulations
// Determines par une mesure de la tension avec l articulation a 90 degres K=90/(Vmes*360/5)
#define RAP_MECA_CAP_1    1.5
#define RAP_MECA_CAP_2    1
#define RAP_MECA_CAP_3    1
#define RAP_MECA_CAP_4    1
#define RAP_MECA_CAP_5    2
#define RAP_MECA_CAP_6    1
#define RAP_MECA_CAP_7    1


//sens de rotation des s par rapport au sens theorique
#define SENS__1   1
#define SENS__2   0
#define SENS__3   0
#define SENS__4   0
#define SENS__5   0
#define SENS__6   1
#define SENS__7   0

#define SENS_PRESSION_1   1
#define SENS_PRESSION_2   1
#define SENS_PRESSION_3   1
#define SENS_PRESSION_4  -1
#define SENS_PRESSION_5  -1
#define SENS_PRESSION_6  -1
#define SENS_PRESSION_7   1

//nombre de points maximal pour la sauvegarde des rseultats
#define NB_POINTS  20

#define BOUCLE_OUVERTE  0
#define BOUCLE_FERMEE  1

//Infos PID
#define P_AXE_1 (0.0139) *1.5 // ancien : 1.0
#define P_AXE_2 (0.0139) *1.75
#define P_AXE_3 (0.0139) *1.0 // ancien : 1.25
#define P_AXE_4 (0.0139) *1.25 //ancien: 1.25
#define P_AXE_5 (0.0139) *1.5
#define P_AXE_6 (0.0139) *1.75
#define P_AXE_7 (0.0139)

//CONSTANTE POUR LA BOUCLE OUVERTE
#define K_BOUCLE_OUVERTE (2.5/180)

#define D_AXE_1 0 //1.5
#define D_AXE_2 0  // 2
#define D_AXE_3 0 //0.75
#define D_AXE_4 0 //1.5
#define D_AXE_5 0 //2
#define D_AXE_6 0 //1.5
#define D_AXE_7 0 //1



/****** INCLUDES ******/
//#include "taskLib.h"
//#include "wdLib.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
//#include "kernelLib.h"
//#include "msgQLib.h"

#include "card.hh"
#include "CIODAC16.hh"
#include "CIODAS64.hh"
#include "actuator.hh"
#include "I_teleop.h"
#include "joystick.h"
#include "capture.hh"
#include "position_sensor.hh"
#include "controller_axe.hh"
#include "controller_tool.hh"
#include "fichier.h"
#include "modele.h"

#endif
