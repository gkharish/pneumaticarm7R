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
	sysClkRateSet(1000);
	printf("\n scanning user input");
	//afficherPageAccueil();
	scanf("%c",&debut);
	
	if(debut!='0')
	{
		principale();
	}
	finProgramme();
}	//FIN main

