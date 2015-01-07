/****************************************
 * Fichier fichier.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#include "fichier.h"

//---------------------------------------------------------------------------



fichier::fichier(char * chemin) {
	acces = (char *) malloc (200);
	strcpy(acces,chemin);
	}


/***************************************/
/* Méthodes de test pour l'ecriture dans un fichier txt*/
/***************************************/

//---------------------------------------------------------------------------
/***************************************/
/* Methodes pour l'ecriture au format Matlab*/
/* A partir des sources de Augustin Sanchez*/
/***************************************/

 // ==============================================
 // ===== SAUVEGARDE DES DONNEES AU FORMAT MATLAB
 // ==============================================

 typedef struct
 {
  long type;         // == type
  long mrows;        // == row dimension
  long ncols;        // == column dimension
  long imagf;        // == flag indicating imag part
  long namlen;       // == name length (including NULL)
 } Fmatrix;


 void fichier::Sauve_Format_MATLAB(FILE *fp, long type, char *pname, long mrows,long ncols, long imagf, double *preal, float *pimag)
  {
  Fmatrix x;
  long mn;

  x.type     = type;
  x.mrows    = mrows;
  x.ncols    = ncols;
  x.imagf    = imagf;
  x.namlen   = strlen(pname) + 1;
  mn         = (long)(x.mrows * x.ncols);

  fwrite(&x, sizeof(Fmatrix), 1, fp);
  fwrite(pname, sizeof(char), (long)(x.namlen), fp);
  fwrite(preal, sizeof(double), mn, fp);
  if (imagf)
  { fwrite(pimag, sizeof(float), mn, fp); }
  
 }
 // == FIN PROCEDURE SAUVE_FORMAT_MATLAB()

 // ======================================================
 // ===== SAUVEGARDE DES VALEURS ACQUISES DANS UN FICHIER
 // ======================================================

  
  void fichier :: sauvegarde_matlab(char * nom_fichier,
                double *temps,double ** pressions,
                double *pression_pince,double *etat_pince,
                double ** angles,double ** angles_des, double ** angle_filtre,
                int nb_points) {
  
  FILE *FICH;
  strcat(acces,nom_fichier);
  strcat(acces,".mat");
  printf("\n La sauvegarde des resultats est dans : \n  %s\n",acces);
  printf("\n\n Enregistrement des mesures... \n\n\n");
  if ((FICH = fopen(acces,"w+b")) != NULL) 
  {
   this->Sauve_Format_MATLAB(FICH,0,"pression_1",1,nb_points,0,pressions[0],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"pression_2",1,nb_points,0,pressions[1],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"pression_3",1,nb_points,0,pressions[2],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"pression_4",1,nb_points,0,pressions[3],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"pression_5",1,nb_points,0,pressions[4],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"pression_6",1,nb_points,0,pressions[5],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"pression_7",1,nb_points,0,pressions[6],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"consigne_pince",1,nb_points,0,pression_pince,(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"etat_pince",1,nb_points,0,etat_pince,(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_1",1,nb_points,0,angle_filtre[0],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_2",1,nb_points,0,angle_filtre[1],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_3",1,nb_points,0,angle_filtre[2],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_4",1,nb_points,0,angle_filtre[3],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_5",1,nb_points,0,angle_filtre[4],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_6",1,nb_points,0,angle_filtre[5],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"ang_fil_7",1,nb_points,0,angle_filtre[6],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_1",1,nb_points,0,angles[0],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_2",1,nb_points,0,angles[1],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_3",1,nb_points,0,angles[2],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_4",1,nb_points,0,angles[3],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_5",1,nb_points,0,angles[4],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_6",1,nb_points,0,angles[5],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_7",1,nb_points,0,angles[6],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_1",1,nb_points,0,angles_des[0],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_2",1,nb_points,0,angles_des[1],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_3",1,nb_points,0,angles_des[2],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_4",1,nb_points,0,angles_des[3],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_5",1,nb_points,0,angles_des[4],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_6",1,nb_points,0,angles_des[5],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"angle_des_7",1,nb_points,0,angles_des[6],(float*)0);
   this->Sauve_Format_MATLAB(FICH,0,"temps",1,nb_points,0,temps,(float*)0);
  }
  else
  { perror("ERREUR LORS DE L'OUVERTURE DE NOM_FICH"); }
  fclose(FICH);
 }
 // == FIN PROCEDURE SAUVEGARDE()



 // ======================================================
 // ===== renommer l'accès au fichier MATLAB
 // ======================================================

void fichier::renommer(char* chemin)
{
    acces = chemin; // récupération du chemin d'accès au fichier MATLAB
}


