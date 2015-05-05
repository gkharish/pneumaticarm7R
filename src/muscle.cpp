/************************************************************
 *	       PROGRAMME DE TEST DE LA REALISATION	    *
 *		     DU CONTROLEUR DE ROBOT	            *
 ************************************************************
 * Cree par : Erwan GUIOCHET/ Mehdi Seffar / JEREMIE GUIOCHET*
 * Date     : 02/07/2002 				    *
 * Derniere modification de M.Seffar: 20/09/2002
 * REPRIS PAR J. GUIOCHET le 15/12/2002			    *
 * VERSION
 ************************************************************/

/*MODIFICATIONS
  17/12/2002: AJOUT DES RAPPORTS MECANIQUES POUR LES sensorS DE POSITION
  20/12/2002: AJOUT Du choix boucle ouverte boucle fermee
*/
#define DBG_INFO  std::endl << __FILE__ << "\n" << __LINE__
#ifndef NDEBUG
#define ODEBUG(x) std::cerr << x << std::endl
#else
#define ODEBUG(x)
#endif


#include <muscle.hh>
void Muscle::
init_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse)
{
  controleur_i -> initialisation_muscles(*delta,*vitesse);
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void Muscle::
reset_muscle_i (controller_axis *controleur_i,  double * vitesse)
{
  controleur_i -> degonfle(*vitesse);
  //signale a la tache principale la fin du degonflement des muscles
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void Muscle::
trait_muscle_i (controller_axis *controleur_i,
                double * delta,
                double * vitesse)
{
  //const char * buf = std::string("ok").c_str();
  //char * buffer = new char[2 * sizeof(double) + 2];
  //double pos_joy,coef;
  if (!end_)
    {
      //double del = *delta;
      controleur_i -> initialisation_muscles(*delta,*vitesse);
      /*if (!tele_op)
	{
	double del = *delta;
	controleur_i -> initialisation_muscles(del,vit);
	//msgQSend(msgq2,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}*/

    }
  else
    {
      controleur_i -> degonfle(*vitesse);
    }
}
