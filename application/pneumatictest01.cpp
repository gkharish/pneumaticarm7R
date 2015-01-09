// Copyright (c) 2014 CNRS
// Authors: Ganesh Kumar


/* ******** It is a test program for pneumatic&R software of Jeremie ********/

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
//#include <native/task.h>
//#include <native/timer.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include "main_controleur.cpp"

void	finProgramme()

{
	printf("\n\n\n*** TOUTE CHOSE A UNE FIN ***\n\n");

}

int main()

{
    char debut;
    printf("\n inside main");
	//sysClkRateSet(1000);
	printf("\n scanning user input");
	//afficherPageAccueil();
	
	
	init();
	
	printf("\n\n\n\n\n\n\n");
	printf("	*****************************************************************\n");
	printf("	*								*\n");
	printf("	*		        PROGRAMME DE				*\n");
	printf("	*               TELEOPERATION DU BRAS 7 AXES			*\n");
	printf("	*								*\n");
	printf("	*****************************************************************\n");
	printf("\n\n\n");
	printf("Tapez une touche puis validez pour initialiser les muscles, ou q pour quitter\n");
    
	
	scanf("%c",&debut);
	if(debut!='0')
	{
		main1 = taskSpawn("t_main",110,0,22000,(FUNCPTR)principale,0,0,0,0,0,0,0,0,0,0); 
		//principale();
		//taskDelete(main1);
	}
	finProgramme();
}	//FIN main

