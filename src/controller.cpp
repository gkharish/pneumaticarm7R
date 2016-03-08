#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/time.h>
// #include <random>

//#include <tgmath.h>
#include <native/task.h>
#include <native/timer.h>

#include <debug.hh>
#include <controller.hh>
#include <shared_memory.hh>
#include <pneumaticarm_model.hh>

using namespace std;
PneumaticarmModel *Joint1modelp = new PneumaticarmModel();
PneumaticarmModel *Joint2modelp = new PneumaticarmModel();
PneumaticarmModel *Joint3modelp = new PneumaticarmModel();


PressureModel *pmodel = new PressureModel();
PressureModel *sim_pmodel = new PressureModel();
////////// COntrol Open Loop Data ///////////


 ///// uhat 1 ///////////////////
 double  uhat1 [] = {
3,3,3,3,3,3,3,3,3,3,3,3,3,3,2.99979328379191,2.99966009436854,2.99948901669674,2.99928919549250,2.99930073280787,2.99925720245182,2.99916329531828,2.99919607939007,2.99779390204989,3,3,3,2.63569140717944,2.77064209026021,2.70272283238329,2.46027269101067,2.39866900974803,2.52991705137644,2.75583862751389,2.28196501520257,2.99654503778593,2.66616958015418,2.56758176596979,2.57302572285290,2.39025838175369,2.74303943257797,2.70137157849647,2.65882179627571,2.61542747540641,2.45159260217569,2.46134889012260,2.35299986809626,2.20712179508561,2.05224065894081,2.00690403143241,1.77710901006944,1.68316605496610,1.82048186502159,1.51844372888261,1.42327917303104,1.32661173499675,0.920474062736631,0.817809243222528,0.512390151818466,0.503685909798358,0.517070746982757,0.484714026599077,0.407067828407987,0.326379741401807,0.140010269954360,0.125430373778824,0.110508574948706,0.0958481327224211,0.200711736190627,0.181072551315774,0.211149596288700,0.0372659203130839,0.0222341566808924,0.0679226680266941,0.00112824889165367,0,0.00658941200019261,0.00124933807684382,0,0,0,0.0168250979762201,0,0,0,0,0,0,0,0,0,0,8.83074026941164e-06,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2.12043922634481e-05,2.31199325592668e-05,0,0,0,0,6.51696703785068e-05,2.05934192657047e-05,1.86631856540656e-05,1.93379667890384e-05,1.99566333729981e-05,2.05069225740581e-05,2.09767232826507e-05,2.13540137812980e-05,2.16277541972720e-05,2.17872750706484e-05,2.18230295145833e-05,2.17267496469413e-05,2.14916806297733e-05,2.11125571671404e-05,2.05860490123385e-05,1.99107745816107e-05,1.90873180408707e-05,1.81185029223784e-05,1.70091534418484e-05,1.57661201892958e-05,1.43980527595074e-05,1.29150665155625e-05,2.69429750911938e-05,2.15374782143453e-05,1.06977010177487e-05,8.97273163798631e-06,7.19489180687411e-06,8.84102065407338e-06,1.10639431269959e-05,1.89321582995057e-06,1.34971689478103e-05,1.76830561437592e-06,1.54248911914073e-06,1.41259321672239e-06,1.68373764637991e-05,0,3,3,3,3,3,3,3,3,3,3,2.99972609360351,3,3,3,3,3,3,0,0,0,0.387076781378699,0.496739974309719,0.522102143376974,0.537046914816215,0.483827144139012,0.510734393524477,0.542935104436305,0.562084673869301,0.571733082550356,0.581271193241945,0.581280916553125,0.693131762530710,0.700967276018668,0.709448844200418,0.690176963798368,0.673067762245336,0.639287314720012,0.604241107685129,0.558396057262336,0.508617301065650,0.465822232124165,0.397057314001679,0.337521915096196,0.273056012214726,0.209298503175575,0.149258579252709,0.0976571996796192,0.0504853894225646,0.0165585957350479,0,0};
// uhat1: delP_ballthrowing_07_03_2016 

/*
 // Delp_optimallift_4sec uhat 1 */
 
 
// DelP_maxend_Vel uhat1
/////////////////////////////
//
//// uhat2 /////////
double uhat2 [] = {
 1.49553696643912,0,0,0,0,0,0,0,0,0,2.36072718647511,1.61740972711664,0.000327276744062506,0,0,0,0,0.201883856705539,1.86167871184431,0.745468443467986,0.132889025600363,0.0597681507147453,0.169723664367839,1.14379919491045,0.468108857174754,0.694797990016562,0.524871793194797,0.265198066565385,0.170207065587983,0.649444484382320,0.509536312421039,0.0813015766461032,0.703155397184666,0.000766365977711816,0.650493899144162,0.638169136935242,0.958971978330451,1.10287616249946,1.13355131397134,1.28070835797857,1.23151596792108,1.21263673035550,0.597943829322277,0.639851873755138,0.681490480099731,1.20434443779087,2.20577045343595,2.15328218500176,2.01418927860444,2.37796767488957,2.39192068238054,2.91868887369256,2.41148526491898,3.00139533627502,2.88514042569835,3.16468702860391,2.94582280712820,2.92016896368924,3.03769534242504,3.38323588215787,3.42671774488768,3.45985626381439,3.27844847500394,3.30685899169687,2.54911287500749,2.53790463307604,2.59780210191140,2.50235340895555,2.52517262144560,3.61706974216949,3.58837321872263,2.95015345214974,3.02018245640691,2.89822788139350,3.96146709234898,3.44195754769915,3.99994994078328,3.73632500958209,3.79487404677228,3.79384602610932,3.84975777416170,3.84814620179286,3.84619417628192,3.84383426192583,3.84105466715256,3.88825095971953,3.88701442007931,3.88560214544426,3.88401298488214,3.88224751904253,3.88030893677556,3.71294973125667,3.87566780439624,3.87640932080749,3.87839104305990,3.87649715302281,3.87392623240643,3.87077009234824,3.86718586260301,3.86327422777307,3.85908673843622,3.85465335103055,3.84999006106356,3.81076937314745,3.80520349894631,3.79947498420170,3.79359627001838,2.58950856194595,2.56809958473255,2.24678751397363,2.21140115572380,1.88488122926472,1.91923862779002,3.71301265904048,3.70843546513694,3.71155981638985,3.71071191734312,3.70086492551731,3.69913612601166,3.72804365668547,2.45364396605552,2.74012303391848,2.80861199496076,2.71517747029416,2.79546508629202,3.44247190962701,3.62835402192333,3.68497459015032,3.72536195351822,3.35904122573830,3.35726346841039,3.42408076983465,3.47139115811936,3.17486604433511,1.92520068478429,1.33895848563589,1.62687192934952,1.73557342049709,1.54154885730159,1.47729983477071,2.66628639917588,2.42826048082293,1.70597193329360,0.0550942778743518,0,0,0,0.356154242775533,2.35122380821368,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.59291670491836e-05,6.67494608775992e-06,0,0,8.11547750145537e-06,0,0,0,0,0,2.72296785444431e-05,0,0,0,8.93069430552155e-06,0,2.39466650905409e-05,0,0,0,0,0,0}; //uhat2:Delp_ballthrowing_07_03_2016

/*};
 // Delp_optimallift_4sec uhat 2*/

//Delp_maxend_Vel uhat2
///////////////////////////
//
//// uhat3 ////////////////
double uhat3 [] = {
0.48573,0.66904,0.89484,1.1896,1.3468,1.5689,1.8493,2.0606,2.2642,2.499,2.8393,2.9943,3.2379,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.4552,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4329,3.4302,3.4125,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.4077,3.1447,1.8655,0.56155,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.02544};
//////////////////////////
//
//// uhat4 ///////////////
double uhat4 [] = {
0.028464,0.032386,0.036725,0.041519,0.046811,0.052649,0.059081,0.066164,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.067088,0.071959,0.078462,0.085426,0.092846,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.092277,0.36824,0.86706,1.3769,1.9217,2.5285,2.9647,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.0278,2.3202,1.7845,1.1489,0.66593,0.21829,0,0};
///////////////////////////



///////////////////////////////////////////
// Joint 1 parameters
// Joint 2 parameters
double j2lo = 0.23;
double j2alphao = 20*PI/180;
double j2k = 1.1;
double j2ro = 0.012;
double j2R = 0.009;
double j2m = 5.20;
double j2link_l = 0.67;
double j2fv = 3.0;
double j2Pmax = 3.0e5;
// Joint 3 parameters
double j3lo = 0.185;
double j3alphao = 20*PI/180;
double j3k = 1.25;
double j3ro = 0.0085;
double j3R = 0.015;
double j3m = 2.6;
double j3link_l = 0.32;
double j3fv = 0.25;

void principale_controller_function(void *arg)
{
  Controller * aController = static_cast<Controller *> (arg);
  if (aController!=0)
  {
    cout << " Controller object is created" << endl;
    aController->ApplyControlLaw();
  }
}


Controller::Controller()
{
  positions_.resize(7);
  velocity_.resize(7);
  acceleration_.resize(7);
  position_store_.resize(4);
  xstate_.resize(3);
  state_mpc_.resize(4);
  simulated_positions_.resize(7);
  controls_.resize(16);
  simulated_controls_.resize(16);
  apply_controls_.resize(16);
  user_controls_.resize(16);
  initconfig_controls_.resize(16);
  simulated_initconfig_controls_.resize(16);
  JOINT_NUM_.resize(7);
  mean_pressure_.resize(7);
  delta.resize(7);
  error_now_.resize(7);
  error_derivative_.resize(7);
  error_prev_.resize(7);
  simulated_delta_.resize(7);
  simulated_error_now_.resize(7);
  simulated_error_derivative_.resize(7);
  simulated_error_prev_.resize(7);

  loop_reference_traj_.resize(7);
  ref_init_.resize(7);
  ref_final_.resize(7);
  ref_traj_.resize(7);
  ref_vel_.resize(7);
  ref_acl_.resize(7);
  ref_jerk_.resize(7);
  ref_jerkdot_.resize(7);
  reference_.resize(5);
  reference_mpc_.resize(4);
  ref_slope_.resize(7);
  ref_type_.resize(7);
  desired_position_.resize(7);
  reset_control_ = false;



  /** \ Pid controller parameter intialization  */
  Pid_factor_.resize(7);
  P_.resize(7);
  D_.resize(7);

  //Here parameters are inititalized to test jont num 4 the elbow joint 
  // Elbow
  P_[3] = 0.0005;
  D_[3] = 0.000;
  Pid_factor_[3] = 1;
  ref_init_[3] = 0;
  desired_position_[3] = 45;
  ref_slope_[3] = 15;
  ref_type_[3] = 0;
  ref_traj_[3] = 0;
 // Joint 2
  P_[1] = 0.0005;
  D_[1] = 0.000;
  Pid_factor_[1] = 1;
  ref_init_[1] = 0;
  desired_position_[1] = 40;
  ref_slope_[1] = 10;
  ref_type_[1] = 0;
  ref_traj_[1] = 0;
// Arm rotation
  P_[2] = 0.0005;
  D_[2] = 0;
  Pid_factor_[2] = 1;
  ref_init_[2] = 0;
  desired_position_[2] = 50;
  ref_slope_[2] = 4;
  ref_type_[2] = 0;
  // Elbow rotation
  P_[4] = 0.001;
  D_[4] = 0;
  Pid_factor_[4] = 1;
  ref_init_[4] = 0;
  desired_position_[4] = 30;
  ref_slope_[4] = 1;
  ref_type_[4] = 1;

  // Wrist 
  P_[5] = 0.0005;
  D_[5] = 0;
  Pid_factor_[5] = 1;

  ref_init_[5] = 0;
  desired_position_[5] = 30;
  ref_slope_[5] = 1;
  ref_type_[5] = 2;

  for(unsigned int i=0;i<16;i++)
    {
      apply_controls_[i] = false;
      user_controls_[i] = 0.0;
      controls_[i]=0.0;
      simulated_controls_[i] =0.0;
    }

  InitSharedMemory();
}


Controller::~Controller()
{
}

void Controller::SetJointNum(unsigned int i)
{
  JOINT_NUM_[i] = true;
  ODEBUG("Setting Joint NUm :" << i);
}

void Controller::SetControllerType(int i)
{
  CONTROLLER_TYPE_ = i;
}

void Controller::ApplyControlLaw()
{
  RTIME   now, TASK_PERIOD = 5e6;//1000000; ..present,
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
  unsigned int loop = 0;
  unsigned int filter_loop =0;
  double velocity1 = 0, velocity2 = 0, acceleration = 0;
  double u_pres[2] = {0, 0};
  /*Plant Model object created*/
 
  if(Joint2modelp!=0)
      ODEBUGL("Pneumatic model object is created",4);
  //Joint2modelp -> setProblemDimension(2);
  /*Joint2modelp -> setParameters(j3lo, 
                                j3alphao, 
                                j3k, 
                                j3ro, 
                                j3R, 
                                j3m,
                                j3link_l, 
                                j3fv) ;*/
  Joint2modelp -> setParameters(j2lo, 
                                j2alphao, 
                                j2k, 
                                j2ro, 
                                j2R, 
                                j2m,
                                j2link_l, 
                                j2fv,
                                j2Pmax) ;

 //PressureModel *pmodel = new PressureModel();
  if(pmodel!=0)
      ODEBUGL("Pressure model object is created",4);
  //pmodel -> setProblemDimension(2);
  pmodel -> setParameters();
  sim_pmodel -> setParameters();

  //model -> server_start();
  double integrator_timestep = 0.005;
  double time_step = TASK_PERIOD/1e9;
  vector<double> previous_state, newstate,  u;
             
        
  //int control_len = 0, control_size, state_len = 0, states_size;
  u.resize(2);
  previous_state.resize(2);
  newstate.resize(2);
  //states_size = previous_state.size();
 /* Variables used in Timer*/
  double t, present_time, previous_time ;
 /* time_t now , previous;
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  now  = spec.tv_sec;
  present_time = (spec.tv_nsec / 1.0e9);*/
  now = rt_timer_read();
  present_time = (double)now/1.0e9;
  previous_time = present_time;
    
  for(unsigned int i=0; i <7; i++)
    {
      unsigned index1 = 16;
      positions_[i] = shmaddr_[index1++];
      ref_init_[i] = 0;   //positions_[i];
       
     
     loop_reference_traj_[i] = 0;
    }
    Joint2modelp -> Set_StateVector(0*positions_[3]*3.14/180, 0);
    Joint2modelp -> Set_StateVector(3.0e5, 4);
    Joint2modelp -> Set_StateVector(0e5, 2);
    cout << "init pos: " <<Joint2modelp ->Get_StateVector(0) << endl;
    pmodel -> Set_StateVector(0.67,0); 
    pmodel -> Set_StateVector(0,1); 
    pmodel -> Set_StateVector(4.0,2); 
    pmodel -> Set_StateVector(0,3);
    sim_pmodel -> Set_StateVector(0.0,0); 
    sim_pmodel -> Set_StateVector(0,1); 
    sim_pmodel -> Set_StateVector(4.0,2); 
    sim_pmodel -> Set_StateVector(0,3);

    simulated_positions_[3] = (Joint2modelp -> Get_StateVector(0))*180/3.14;
    previous_state[0] = simulated_positions_[3];
    previous_state[1] = 0;
    newstate[0] = previous_state[0];
    newstate[1] = 0;
    u[0] = 0.0;///;  //shmaddr_[6];  //Joint2modelp -> Get_ControlVector(0);  //shmaddr_[6];
    u[1] = 3.0e5;  //shmaddr_[7];  //Joint2modelp -> Get_ControlVector(1); //shmaddr_[7];
    simulated_initconfig_controls_[2] = 0.0; 
    simulated_initconfig_controls_[3] = 0.0; 
    for (unsigned int i =0; i<2; i++)
          Joint2modelp -> Set_ControlVector(u[i], i);
    /*for (unsigned int i =0; i<2; i++)
          Joint2modelp -> Set_StateVector(0, i);*/
  
    //for (unsigned int i =2; i<4; i++)
    //      Joint2modelp -> Set_StateVector(2.5, i);;
  while(1)
    {
      // Waiting the next iteration
      rt_task_wait_period(NULL);      

      shm_sem_.Acquire();
      unsigned int index =16;
      for(unsigned int i=0;i<7;i++)
      {
        positions_[i] = shmaddr_[index++];
        
        ref_final_[i] = GetDesiredPosition(i);
      }
      shm_sem_.Release();
      ODEBUGL("DEbug Before referencegen" << position_store_[0], 0);
      double js2 = Joint2modelp -> Get_StateVector(0);
      //cout << "POSITION[1] = " <<js2  << endl;
      sim_pmodel -> Set_PositionFeedback(positions_[3]*3.14/180);
      pmodel -> Set_PositionFeedback(positions_[3]*3.14/180);

      //pmodel -> Set_PositionFeedback(Joint2modelp -> Get_StateVector(0));
      //sim_pmodel -> Set_PositionFeedback(Joint2modelp -> Get_StateVector(0));
      
      if(filter_loop <=2)
      {
          position_store_[2] = position_store_[1];
          position_store_[1] = position_store_[0];

          filter_loop++;
      }
      if(filter_loop ==3)
          filter_loop = 0;
      position_store_[0] = positions_[3]*3.14/180;
      

      velocity1 = (position_store_[1] - position_store_[2])/time_step;
      velocity2 = (position_store_[0] - position_store_[1])/time_step;

      acceleration = (velocity2 - velocity1)/time_step;
      xstate_[0] = position_store_[0];
      xstate_[1] = velocity2;
      //xstate_[
      velocity_[3] = velocity2;
      acceleration_[3] = acceleration;
      xstate_[2] = acceleration;
      
      //ReferenceGenerator(loop*TASK_PERIOD/1.0e9);
      // ODEBUGL("After Refgen", 1);
      ComputeControlLaw(TASK_PERIOD);

     /* u[0] = simulated_controls_[6] ;
      u[1] = simulated_controls_[7];
      //cout << "sim_Pmodel output:" <<u[0];
      for (unsigned int i =0; i<2; i++)
          Joint2modelp -> Set_ControlVector(u[i], i);
      ODEBUGL("DEbug after set_Controlvector", 4);*/
      /*u_pres[0] = initconfig_controls_[6] + mpc_u;
      u_pres[1] = initconfig_controls_[7] - mpc_u;
      for (unsigned int i =0; i<2; i++)
          pmodel -> Set_ControlVector(u_pres[i], i);*/
       
     /* clock_gettime(CLOCK_REALTIME, &spec);
      now  = spec.tv_sec;
      present_time = (spec.tv_nsec / 1.0e9); */
      now = rt_timer_read();
      present_time = (double)now/1.0e9;
      t = present_time - previous_time;
      /* Ste*/

      //Joint2modelp -> integrateRK4(t, integrator_timestep);
      simulated_positions_[3] = (Joint2modelp -> Get_StateVector(0));
      ODEBUGL("DEbug after integrator" << simulated_positions_[3] , 1);
      previous_time = present_time;
     /* pmodel -> integrateRK4(t, integrator_timestep);
      controls_[6] = pmodel -> Get_StateVector(0);
      controls_[7] = pmodel -> Get_StateVector(2);
      if(controls_[7] >=4.0)
          controls_[7] = 4.0;
      else if (controls_[7] <= 0.0)
          controls_[7] = 0.0;
      if(controls_[6] >=3.75)
          controls_[6] = 3.75;
      else if (controls_[6] <= 0.0)
          controls_[6] = 0.0;a*/

      
      
      for (unsigned int i=0; i<2; i++)
          previous_state[i] = newstate[i];

      // ODEBUGL("After Control Law" , 1);
      shm_sem_.Acquire();
      for(unsigned int i=0;i<16;i++)
	shmaddr_[i] = controls_[i];
      shmaddr_[24] =  Joint2modelp -> Get_PmeanRef() ;//state_mpc_[2]*1e-5;
      //shmaddr_[24] = ref_traj_[1]*PI/180; //velocity_[3];
      //shmaddr_[21] = acceleration_[3];
      //shmaddr_[24] = mpc_controller.GetState()*180/3.14;
      //shmaddr_[23] = (int)( Joint2modelp -> Get_StateVector(0)) *180/3.14;  //newstate[0]*180/3.14;
      shm_sem_.Release();
      loop++;
    }
}

void Controller::ComputeControlLaw(long double timestep)
{
   double u_pres[2] = {0,0};  
    if (CONTROLLER_TYPE_== 3)
    {
      for (unsigned int i=0;i<16;i++)
	{
	  if (reset_control_ ==false)
          {
              controls_[i] = initconfig_controls_[i];
              simulated_controls_[i] = simulated_initconfig_controls_[i];
          }
          else 
          {
              controls_[i] = 0.0;
              simulated_controls_[i] = 0.0;
          }
	}
    }
  if(CONTROLLER_TYPE_== 1)
    {
      for (unsigned int i=0;i<16;i++)
	{
	  if (apply_controls_[i] ==true && reset_control_ ==false)
          {
              controls_[i] = user_controls_[i];
              simulated_controls_[i] = user_controls_[i];
          }
	  else 
          {
              controls_[i] = 0.0;//initconfig_controls_[i];
              simulated_controls_[i] = 0.0;
          }
	}
        controls_[6] = initconfig_controls_[6]+2;   // Step input open loop
        controls_[7] = initconfig_controls_[7]-2;
    }
  if (CONTROLLER_TYPE_ == 2)
    {
      //ODEBUGL("Inside Pid control: " << JOINT_NUM_[3], 1);
      // JOINT_NUM_[3] = tru= true && end_of_loop_ == false)
   double wn = 0.2;
      double delc = 1*( (sin((double)(loop_reference_traj_[3]*timestep*2*PI*wn/1.0e9))) );/*+ 
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*0.5)/1.0e9)))+ 
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*0.25)/1.0e9)))+
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*1.5)/1.0e9)))+
                          (sin((double)(loop_reference_traj_[3]*timestep*2*PI*(wn*1.75)/1.0e9))) );*/
      double tim = loop_reference_traj_[3]*timestep/1.0e9;
      double delc1;
      double step_time = 1.5;
      double step_amp = 0.1;
      bool exit = false;
      int lp  = (int)(tim/step_time);
      delc1 = 3*tim;//step_amp*lp;
      //delc1 = 0.02*(rand()%100); 
      //delc1 = d(gen);
     if(delc1 > 3)
              delc1 = 3;
    double init_pres1 = initconfig_controls_[2] ;
    double init_pres2 = initconfig_controls_[3] ;
    //double Pdes_feedforward;
    double sim_u_pres[2];
    criterror_ = 1e-8;
      for (unsigned int i =0; i<7; i++)
	{
	  if (JOINT_NUM_[i] == true && reset_control_==false)  
	    {
	      /*ReferenceGenerator(loop_reference_traj_[i]*timestep/1.0e9, i,  ref_type_[i]);
	      //ODEBUG("Inside Joint num:" << i );
             //ref_traj_ = ref_final_;
	      error_now_[i] = ref_traj_[i] - positions_[i];
              simulated_error_now_[i] = ref_traj_[i] - simulated_positions_[i];
	      error_derivative_[i] = error_now_[i] - error_prev_[i];
              simulated_error_derivative_[i] = simulated_error_now_[i] - simulated_error_prev_[i];
	      error_prev_[i] = error_now_[i];
              simulated_error_prev_[i]  = simulated_error_now_[i];
              if(abs(error_now_[i]) >= criterror_)
                integrated_error_ = integrated_error_ + 0.005*error_now_[i]*PI/180;

	      ODEBUGL("error_now: " << error_now_[i],3);
	      ODEBUGL("error_prev:" << error_prev_[i],3);
	      // mpc_u = PidController(error_now_[i], error_derivative_[i],i);
              //SimulatedPidController(simulated_error_now_[i], simulated_error_derivative_[i],i);
              
              reference_[0] = ref_traj_[i]*3.14/180;
              reference_[1] = ref_vel_[i]*3.14/180;
              reference_[2] = ref_acl_[i]*3.14/180;
              reference_[3] = ref_jerk_[i]*3.14/180;
              reference_[4] = ref_jerkdot_[i]*3.14/180;
              vector<double> Pdes_feedforward = Joint2modelp ->  InverseModel(reference_);
              //Pdes_feedforward = Pdes_feedforward + 0.6*error_now_[i] + 0.1*integrated_error_;

              //double Pcur =  OpenInverseModel(xstate_);
              //cout << "Pcur: " << Pdes_feedforward;
              reference_mpc_[0] = reference_[0];
              reference_mpc_[1] = reference_[1];
              reference_mpc_[2] = Pdes_feedforward[0]*1e5;
              reference_mpc_[3] = 4.0*1e5 - reference_mpc_[2];

              // Calling MPC controller
              state_mpc_[0] = xstate_[0];
              state_mpc_[1] = xstate_[1];
              //state_mpc_[2] =  (Pdes_feedforward)*1e5; //pmodel->Get_StateVector(0)*1e5; //Pcur*1e5; 
              //state_mpc_[3] = 4e5 - state_mpc_[2]; // pmodel->Get_StateVector(2)*1e5; //4.0*1e5 - Pcur*1e5;
              //cout << "Pes: " << state_mpc_[2]*1e-5 << endl;
              //mpc_u = mpc_controller.GetControl(state_mpc_, reference_mpc_);
              //cout << "mpc_u :" << mpc_u << endl;
              double torquedes = Joint2modelp -> Get_TorqueDes();
              double torque = Joint2modelp -> Get_Torque();
              double thetades = ref_traj_[1]*PI/180;//Joint2modelp -> Get_StateVector(0);
              double poscur = positions_[1]*PI/180;
              double Terror = torquedes - torque;
              double poserror = thetades - poscur;
              if(abs(Terror) >= criterror_)
                integrated_Terror_ = integrated_Terror_ + 0.005*Terror;
              //cout << "Terror: " << poserror << " Int error" << integrated_error_ <<endl;
              //Pdes_feedforward[0] = Pdes_feedforward[0] + 0.8*poserror + 1.5*integrated_error_;

              //Pdes_feedforward = Pdes_feedforward + 3.5*(Terror) + 3.0*integrated_Terror_;
              cout <<" Pdes" << Pdes_feedforward[0] << endl;
              
              u_pres[0] = initconfig_controls_[6] + mpc_u*1e-5; //Pdes_feedforward;
              u_pres[1] = initconfig_controls_[7] - mpc_u*1e-5; //Pdes_feedforward;
              for (unsigned int i =0; i<2; i++)
                  pmodel -> Set_ControlVector(u_pres[i], i);
              //pmodel -> integrateRK4(loop_reference_traj_[i]*timestep/1.0e9, 0.005);
              
              sim_u_pres[0] = 0 + Pdes_feedforward[0]; //*(loop_reference_traj_[i]*timestep/1.0e9); //Pdes_feedforward;
              sim_u_pres[1] =  3 - Pdes_feedforward[0]; //*(loop_reference_traj_[i]*timestep/1.0e9); ; //Pdes_feedforward;

              for (unsigned int i =0; i<2; i++)
                  sim_pmodel -> Set_ControlVector(sim_u_pres[i], i);
              sim_pmodel -> integrateRK4(loop_reference_traj_[i]*timestep/1.0e9, 0.005);
              simulated_controls_[2*i] = sim_pmodel -> Get_StateVector(0);
              simulated_controls_[2*i+1] = sim_pmodel -> Get_StateVector(2);
              //cout << "sim_cont" << simulated_controls_[2*i] << endl;
              //if(Pdes_feedforward <=  initconfig_controls_[6] )
              //{
              */
              if(loop_reference_traj_[i] <= 200)
              {
                  if(uhat2[loop_reference_traj_[3]] > initconfig_controls_[6])
                  {
                    controls_[6] = uhat2[loop_reference_traj_[3]];// + initconfig_controls_[2*i ];  
                    controls_[7] = 4 - uhat2[loop_reference_traj_[3]]; 
                  }
                  else
                  {
                      controls_[6] = uhat2[loop_reference_traj_[3]]+ initconfig_controls_[6];  
                      controls_[7] =  4 - controls_[6]; //uhat2[loop_reference_traj_[3]]; 
                  }
                 if(uhat1[loop_reference_traj_[3]] > initconfig_controls_[3])
                  {
                    controls_[2] = uhat1[loop_reference_traj_[3]];// + initconfig_controls_[2*i ];  
                    controls_[3] = 3 - uhat1[loop_reference_traj_[3]]; 
                  }
                  else
                  {
                      controls_[2] = uhat1[loop_reference_traj_[3]]+ initconfig_controls_[2];  
                      controls_[3] =  3 - controls_[2]; //uhat2[loop_reference_traj_[3]]; 
                  }
 
                  
              
             }
            
              if(loop_reference_traj_[i] == 180)
              {
                  controls_[14] = 5;
                 // controls_[7] = uhat4[loop_reference_traj_[3]]; 
                 controls_[15] =  0;
              
             }

              loop_reference_traj_[i]++;
              cout << "loop: "  << uhat2[loop_reference_traj_[3]] << endl;        /*
             Joint2modelp -> Set_ControlVector(sim_u_pres[0]*1e5, 0);
             Joint2modelp -> Set_ControlVector(sim_u_pres[1]*1e5, 1);
            
             
             Joint2modelp -> integrateRK4(loop_reference_traj_[i]*timestep/1.0e9, 0.005);
            */
             if(controls_[2*i +1] >=4.5)
                  controls_[2*i +1] = 4.5;
             else if (controls_[2*i+1] <= 0.0)
                  controls_[2*i+1] = 0.0;
              
             if(controls_[2*i] >=4.5)
                  controls_[2*i] = 4.5;
             else if (controls_[2*i] <= 0.0)
                  controls_[2*i] = 0.0;
             
             
	      ODEBUGL(" loop_traj" << loop_reference_traj_[i] << "\n",0);
	    }
	  else
	    {
	      controls_[2*i] = initconfig_controls_[2*i];
	      controls_[2*i+1] = initconfig_controls_[2*i+1];
              simulated_controls_[2*i] = simulated_initconfig_controls_[2*i];
	      simulated_controls_[2*i+1] = simulated_initconfig_controls_[2*i+1];

	    }
	}
         
    }


}
/*double Integrator(double t, double error, double h)
{
    double st1, st2, st3, st4, integrated_Error;
    error_temp = error;
    st1 = error;
    integrated_Error = error_temp + 0.5*h*st1;
    st2 = error;
    integrated_Error = error_temp + 0.5*h*st2;
    st3 = error;
    integrated_Error = error_temp + h*st3;
    st4 = error;
    integrated_Error = error_temp + ( (1/6.0) * h * (st1 + 2.0*st2 + 2.0*st3 + st4) );

}*/
void Controller::ResetControl(bool idx)
{
  reset_control_ = idx;
}

double Controller::PidController(double error, double error_derivative, int joint_num)
{
  double update_delta;
  //  double error_acceptable_ = 1;
 // if (error >= error_acceptable_ || error <= -error_acceptable_)
 if(true)
    {
      update_delta =  Pid_factor_[joint_num]*(P_[joint_num]*error + D_[joint_num]*error_derivative);
      delta[joint_num] = delta[joint_num]+update_delta;
    }
   
  /*double control_limit_agonistic =  initconfig_controls_[2*joint_num]+ delta[joint_num];
  double control_limit_antagonistic = initconfig_controls_[2*joint_num+1] - delta[joint_num];


  if (control_limit_agonistic  <=4.5 && control_limit_agonistic >=0)
    { controls_[2*joint_num]= control_limit_agonistic;}
  else
    {
      if (control_limit_agonistic < 0.0)  
        controls_[2*joint_num] = 0.0;
      else 
	if (control_limit_agonistic > 4.5) 
	  controls_[2*joint_num] = 4.5;
    }

  if (control_limit_antagonistic <= 4.5 && control_limit_antagonistic >=  0.0)
    controls_[2*joint_num + 1] = control_limit_antagonistic;
  else
    {
      if (control_limit_antagonistic <0.0) controls_[2*joint_num +1] = 0.0;
      else if (control_limit_antagonistic >4.5) controls_[2*joint_num+1] = 4.5;
    }
     

  // controls_[2*joint_num] = MeanPressure(joint_num)+ delta[joint_num];
  // controls_[2*joint_num+1] = MeanPressure(joint_num) - (delta[joint_num]);
   
  ODEBUGL("Update delta:     " <<update_delta, 4);
  ODEBUGL("Pid command : " << delta[joint_num],4);*/
  return(delta[joint_num]);

}

void Controller::SimulatedPidController(double error, double error_derivative, int joint_num)
{
  double simulated_update_delta;
  //  double error_acceptable_ = 1;
 // if (error >= error_acceptable_ || error <= -error_acceptable_)
 if(true)
    {
      simulated_update_delta =  Pid_factor_[joint_num]*(P_[joint_num]*error + D_[joint_num]*error_derivative);
      simulated_delta_[joint_num] = simulated_delta_[joint_num] + simulated_update_delta;
    }
   
  double simulated_control_limit_agonistic =  simulated_initconfig_controls_[2*joint_num] + 1.5; //simulated_delta_[joint_num];
  double simulated_control_limit_antagonistic = simulated_initconfig_controls_[2*joint_num +1] - 1.5;//simulated_delta_[joint_num];


  /*if (control_limit_agonistic  <=4.5 && control_limit_agonistic >=0)
    { controls_[2*joint_num]= control_limit_agonistic;}
  else
    {
      if (control_limit_agonistic < 0.0)  
        controls_[2*joint_num] = 0.0;
      else 
	if (control_limit_agonistic > 4.5) 
	  controls_[2*joint_num] = 4.5;
    }

  if (control_limit_antagonistic <= 4.5 && control_limit_antagonistic >=  0.0)
    controls_[2*joint_num + 1] = control_limit_antagonistic;
  else
    {
      if (control_limit_antagonistic <0.0) controls_[2*joint_num +1] = 0.0;
      else if (control_limit_antagonistic >4.5) controls_[2*joint_num+1] = 4.5;
    }*/
     

  simulated_controls_[2*joint_num] = 3.5 ;//controls_[6] - initconfig_controls_[6]; //simulated_control_limit_agonistic;
  simulated_controls_[2*joint_num+1] = 3.5 ;//controls_[7] -initconfig_controls_[7]; //simulated_control_limit_antagonistic;
   
  ODEBUGL("Simulated Update delta:     " <<simulated_update_delta, 4);
  ODEBUGL("Simulated Pid command : " << delta[joint_num],4);
}



double Controller::MeanPressure(int i)
{
  //mean_pressure_[i] = initconfig_controls_[i];
  return(mean_pressure_[i]);
}

void Controller::ReferenceGenerator(long double timestep, unsigned int joint_num, unsigned int type)
{
  if (type == 0)
  {

     if (timestep <= 4.0)
     {
        ref_traj_[joint_num] = ref_init_[joint_num] + ref_slope_[joint_num]*(double)timestep;
        ref_vel_[joint_num] = ref_slope_[joint_num];
                
        ref_final_[joint_num] = ref_init_[joint_num] + ref_slope_[joint_num]*4.0;


     }
    
     if (timestep <= 20.0 && timestep > 4.0)
     {
        ref_traj_[joint_num] = ref_final_[joint_num];
        ref_vel_[joint_num] = 0;
        ref_acl_[joint_num] = 0;

     }

     if (timestep <= 24.0 && timestep > 20.0) 
     {
        ref_traj_[joint_num] = ref_final_[joint_num] - ref_slope_[joint_num]*((double)timestep - 20.0);
        ref_vel_[joint_num] = -ref_slope_[joint_num];
        ref_acl_[joint_num] = 0;

     }

     if (timestep > 24.0)
     {
        ref_traj_[joint_num] = 0;
        ref_vel_[joint_num] = 0;
        ref_acl_[joint_num] = 0;

     }
     ref_acl_[joint_num] = 0;
     ref_jerk_[joint_num] = 0.0;
     ref_jerkdot_[joint_num] = 0.0;

   /* if(ref_init_[joint_num] <= ref_final_[joint_num])
    {
      if(ref_traj_[joint_num] >= ref_final_[joint_num])
      {
	ref_traj_[joint_num] = ref_final_[joint_num];
        ref_vel_[joint_num] = 0;
        ref_acl_[joint_num] = 0;
      }
      else
      { ref_traj_[joint_num] = ref_init_[joint_num] + ref_slope_[joint_num]*(double)timestep;
        ref_vel_[joint_num] = ref_slope_[joint_num];
        ref_acl_[joint_num] = 0;
      }
    }
  if(ref_init_[joint_num] >  ref_final_[joint_num])
    {
      if(ref_traj_[joint_num] <= ref_final_[joint_num])
      {
	ref_traj_[joint_num] = ref_final_[joint_num];
        ref_vel_[joint_num] = 0;
        ref_acl_[joint_num] = 0;
      }
      else
      {
        ref_traj_[joint_num] = ref_init_[joint_num] - ref_slope_[joint_num]*(double)timestep;
        ref_vel_[joint_num] = -ref_slope_[joint_num]; 
        ref_acl_[joint_num] = 0;
      }
    }*/
  }
  
  if (type == 1)
  {
      //SetStepResponse();
      ref_traj_[joint_num] = desired_position_[joint_num];
  }

  if (type == 2)
  {
      if (joint_num == 1)
      {
        double f = 2*PI*0.1;
        ref_traj_[joint_num] = 20+20*(sin((double)timestep*f ));
        ref_vel_[joint_num] = f*20*(cos((double)timestep*f));
        ref_acl_[joint_num] = -f*f *20*(sin((double)timestep*f ));
        ref_jerk_[joint_num] = -f*f*f *20*(cos((double)timestep*f ));
        ref_jerkdot_[joint_num] = f*f*f*f *20*(sin((double)timestep*f ));


      }
      else 
        ref_traj_[joint_num] = 20* sin((double)timestep*2*PI/10);
  }


 ODEBUGL("Ref_traj_ : "<< ref_traj_[joint_num], 1); 
}

double Controller::GetDesiredPosition(unsigned int idx)
{
  return(desired_position_[idx]);
}

void Controller::SetUserControl(unsigned int idx, double control)
{
  user_controls_[idx] = control;
  initconfig_controls_[idx] = control;
}

double Controller::GetUserControl(unsigned int idx)
{
  return user_controls_[idx];
}

void Controller::SetApplyControl(unsigned int idx, bool apply_control)
{
  apply_controls_[idx] = apply_control;
}

bool Controller::GetApplyControl(unsigned int idx)
{
  return apply_controls_[idx];
}

void Controller::StartingRealTimeThread()
{
  unsigned n=0;
  n = rt_task_create(&principal_task_, "Principale Controller Function", 0, 99, 0);
  if (n!=0)
    {
      std::cerr << "Failed @ RT Create" << n << std::endl;
    }
  else 
    { ODEBUGL("END of RT Create",3); }

  n = rt_task_start(&principal_task_, principale_controller_function, this);
  if (n!=0)
    {
      std::cerr<< "Failed of RT STart" <<n<< std::endl;
    }
  else 
    { ODEBUGL("END of RT Start",3); }

}

void Controller::InitSharedMemory()
{
  // Update and/or create the file.
  std::ofstream aof;
  aof.open(SHM_LOG_FILENAME,
           std::ofstream::out | std::ofstream::app);
  struct timeval current_time;
  gettimeofday(&current_time,0);
    
  aof << "Starting control " 
      << current_time.tv_sec << "." 
      << current_time.tv_usec << std::endl;
  aof.close();
  
  // Attached the shared memory to a memory segment.
  shmaddr_ = CreateSharedMemoryForPneumaticArm(false);

}

void Controller::CloseSharedMemory()
{
  shmdt(shmaddr_);

  // Update the file.
  std::ofstream aof;
  aof.open(SHM_LOG_FILENAME,
           std::ofstream::out | std::ofstream::app);
  struct timeval current_time;
  gettimeofday(&current_time,0);
    
  aof << " Stop control at " 
      << current_time.tv_sec << "." 
      << current_time.tv_usec << std::endl;
  aof.close();

}


double Controller::GetErrorNow(unsigned int idx)
{
  return error_now_[idx];
}

double Controller::GetErrorDerivative(unsigned int idx)
{
  return error_derivative_[idx];
}

double Controller::GetUpdateDelta(unsigned int idx)
{
    return(P_[idx]*error_now_[idx]);
}

bool Controller::GetJointNum(unsigned int idx)
{
    return(JOINT_NUM_[idx]);
}

