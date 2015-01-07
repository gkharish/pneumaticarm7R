/****************************************
 * Fichier fichier.h            	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef FICHIER
#define FICHIER 
 
 ///---------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
//#include <dos.h>
#include <string.h>
#include <time.h>

/********************************************************************
 *                          Classe fichier                          *
 ********************************************************************
 *                                                                  *
 *    gere la sauvegarde des donnees sous format matlab             *
 *                						    *
 *		  		        			    *
 ********************************************************************/ 

class fichier {

    public:  
    	    //Constructeurs
            fichier (){}
            fichier(char*);
            
            //Sauvegarde sous format matlab
            void sauvegarde_matlab(char *,double*,double**,double *,double *,	
            			   double**,double **,double **,
            			   int);	   
            void renommer (char *);			   

    private:
            void Sauve_Format_MATLAB(FILE *, long,char*,long,long,long,double*,float*);
            char * acces;     //chemin d'acces au fichier

};

#endif