///////////////////////////////////////////////////////////////////////////
//////// MDP MODEL FOR HRI IN 3 AGRICULTURAL SCENARIOS //////////////////// 
///////////////////////////////////////////////////////////////////////////
//The guards are tagged with operation codes according to the corresponding agricultural operation
//These codes are: "U1" for UV-C treatment,"P1" for Picking,"L1" and "L2" for Logistics and "G1" for general operations
///////////////////////////////////////////////////////////////////////////
//The guards defined in the HRI module are executed according to the following sequence:
//1st The x_uvc - variables that contains all the operations of the specific scenario simulated
//    The x_logistics
//    The x_picking
//2nd The x_human (to know if the human is going to interact or not, and if the human was trained or not)
//3rd The x_human_aware (to know if the human is aware of the robot intentions or danger)
//4th The x_human_motion (to know if the human is moving or stationary)
//5th The x_human_distance (to know the distance between the robot and the human)
//6th The x_human_gesture (to know if the human is performing hand gestures)
//7th The x_hds (human detection system)
//8th The x_htmis (human tracking and motion inference system) - used for logistics and picking inside the polytunnels & logistics, picking  outside the polytunnels
//9th The x_hars (human action recognition system)- used for logistics and picking inside the polytunnels & logistics, picking outside the polytunnels
//10th The x_robot  
//11th The x_visual & x_voice (feedback alerts) - communication messages
//12th The x_visual & x_voice (feedback alerts) - periodic messages
//13th The x_scs (safety contact system)
//14th The x_trays (used in logistics and picking) - variables to determine when each operation has finished
//     The x_runs (used in logistics and picking)
//     The x_rows (used in UV-C treatment)
//     The x_seg (used in all scenarios)
//(here the variables used for synchronism are reset to repeat the sequence again)
///////////////////////////////////////////////////////////////////////////
// IMPORTANT INFORMATION ABOUT THE HRI MODEL //////////////////////////////
///////////////////////////////////////////////////////////////////////////
// The pickers along the rows who called the robot begins stationary, but they can decide to stay stationary or approch to the robot when they are 1.2-3.6m
// If the picker didn't call the robot, he/she starts stationary, but they can decide to approach or move away when the robot is 1.2-3.6m
// The untrained people inside polytunnels and workers at the end of the rows are always moving to the robot position till they become aware of the danger, then they move away
// The workers and untrained people at footpaths are moving to robot direction till they become aware of the danger, then they can decide to stop or move away
// If the worker or untrained people stopped before being 3.6m from the robot, then the robot starts evading and the human can decide to keep stationary or walk next to the robot
// Only workers can perform hand gestures, and it is allowed only when the robot is whitin 3.6m and the human is aware of the robot presence
// If a human (trained or untrained) get an injury or if the robot performed a safety stop, then he/she gets aware about the danger automatically and decides to move away from the robot
// Workers who are supposed to place trays on the robot (inside and outside polytunnel) are getting aware of the danger since the beggining  
// There are unplanned interactions with trained people only at footpaths for UVC, i.e. inside polytunnels is assumed non planned interactions only with non trained people  
// The N_trays, N_segments and N_rows counters increase when the robot is performing segment or row transitions
// The counters/variables used for task monitoring (14th step) are updated only if none human is interacting with the robot
///////////////////////////////////////////////////////////////////////////

mdp // To tell PRISM that the model must be interpreted as an MDP model

/////// CONSTANTS /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
const int N_rows=5; // Number of rows that the robot can cover before it has to comeback to the robot shed to charge the battery 
const int N_segments_shed=2; // Number of footpath segments required to be traversed when moving from robot shed to the polytunnel
const int N_segments_collect=1; // Number of footpath segments required to be traversed when moving from collection point to the polytunnel
const int N_runs=N_rows; // Number of times a robot can perform two way trips from polytunnel to collection point before it has to comeback to robot shed to charge the battery
const int N_trays=2; // Number of trays that the robot can transport, OR number of times the robot is summoned by a picker to exchange trays before going back to the collection point
const int scenario=1; // A Number related with the scenario simulated (1=UVC, 2=Picking, 3=Logistics)
const bool training= false; // To know if the unplanned HRI is with a trained or non-trained person
///////////////////////////////////////////////////////////////////////////

/////// PROBABILITIES FOR THE HUMAN BEHAVIOUR /////////////////////////////
///////////////////////////////////////////////////////////////////////////
//Probability that an untrained person or a distracted worker gets aware about the "potential danger, robot presence, or robot intentions" when robot activates "visual alerts"  
//(By default humans are not aware of robot presence till the audio visual alarms are activated)
const double p_h_aware_1=0.7 ; // "untrained" human case 
const double p_h_aware_2=0.8; // "trained" human case
//Probability that an untrained person or a distracted worker  gets aware about the "potential danger, robot presence, or robot intentions" when robot activates "voice messages" 
const double p_h_aware_3=0.9 ;
//Probability that the human decides to stay stationary or move away (safer decision) instead of make a risky movement during close interactions
const double p_h_decision_1=0.8; // the higher value, the safer the decision
//Probability that a worker performs a hand gesture to "reply" robot messages during close interactions
const double p_h_reply_1=0.8; 
//////// PROBABILITIES FOR THE SAFETY SYSTEM //////////////////////////////
/////////////////////////////////////////////////////////////////////////// 
// Failures of Periodic audiovisual alerts
const double p_periodic_alerts=0.7; // Probability that at this specific moment a periodic audiovisual alert is activated to warn nearby human about danger 
// Failures of CDS (collision detection system)
const double p_contact_fail=0.3;  // Probability that the robot contact sensors fails to detect a collision
// Failures of HDS (human detection system)
const double p_hds_fail_1;  // Probability that HDS fails in detect a human in the same row farther than 7m 
const double p_hds_fail_2=p_hds_fail_1;  // Probability that HDS fails in detect on time a human at 7m in the same row 
const double p_hds_fail_3=p_hds_fail_1;  // Probability that HDS fails in detect on time a human at 3.6m in the same row 
const double p_hds_fail_4=p_hds_fail_1;  // Probability that HDS fails in detect on time a human at 1.2m in the same row 
const double p_hds_fail_5=0.4;  // Probability that HDS fails in detect a human at the end of the rows farther 7m when the robot is going to perform row transitions 
const double p_hds_fail_6=0.3;  // Probability that HDS fails in detect on time a human at the end of the row at 7m when the robot is going to perform row transitions 
const double p_hds_fail_7=0.2;  // Probability that HDS fails in detect on time a human at the end of the row at 3.6m when the robot is going to perform row transitions 
const double p_hds_fail_8=0.1;  // Probability that HDS fails in detect on time a human at the end of the row at 1.2m when the robot is going to perform row transitions 
const double p_hds_fail_9=0.3;  // Probability that HDS fails in detect a human on time in the same footpath above 3.6m  
const double p_hds_fail_10=0.2; // Probability that HDS fails in detect a human on time in the same footpath at 3.6m 
const double p_hds_fail_11=0.1; // Probability that HDS fails in detect a human on time in the same footpath at 1.2m 
// Failures of HTMIS (human tracking and motion inference system)
const double p_htmis_fail_1=0.2;  // Probability that HTMIS fails in tracking accurately a human in the same row 
const double p_htmis_fail_2=0.2;  // Probability that HTMIS fails in tracking accurately a human in the same footpath 
//Failures of HGRS (Human gesture recognition system)
const double p_hars_fail=0.2; // Probability that the HGRS fails to detect the correct hand gesture 
//Probability that an unplanned interaction is going to happen inside the polytunnels with an untrained person
const double p_h_interact_1; 
//Probability that an unplanned interaction is going to happen inside the polytunnels with an trained person
const double p_h_interact_2=p_h_interact_1; 
//Probability that an unplanned interaction is going to happen outside the polytunnels with an untrained person
const double p_h_interact_3=p_h_interact_1; 
//Probability that an unplanned interaction is going to happen outside the polytunnels with an trained person
const double p_h_interact_4=p_h_interact_2;
///////////////////////////////////////////////////////////////////////////

//////// FORMULAS /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// conditions used as guards to execute commands with "no update" only if the commands "with updates" are not executed
// for update flag_operation in the case of uvc scenario
formula condition_1 = (scenario=1) & (((flag_task_finished=false) & (x_seg=0 & x_rows=0 & x_robot=0 & x_uvc=0)) | (x_seg=N_segments_shed & x_rows=0 & x_robot=1 & x_uvc=1) | (x_seg=0 & x_rows=N_rows & x_robot=4 & x_uvc=2) | (x_seg=N_segments_shed & x_robot=1 & x_uvc=3));
// for update flag_operation in the case of logistics scenario
formula condition_3 = (scenario=3) & (((flag_task_finished=false) & (x_seg=0 & x_rows=0 & x_robot=0 & x_logistics=0)) | (x_seg=(N_segments_shed-N_segments_collect) & x_trays=0 & x_robot=1 & x_logistics=1) | (x_seg=0 & x_trays=0 & x_robot=2 & x_logistics=2) | (x_seg=N_segments_collect & x_robot=1 & x_logistics=3) | (x_human=0 & x_robot=5 & x_logistics=4) | (x_trays=N_trays & x_robot=7 & x_logistics=5) | (x_trays<N_trays & x_robot=7 & x_logistics=5) | (x_seg=N_segments_collect & x_robot=2 & x_logistics=6) | (x_seg=0 & x_runs=N_runs & x_robot=2 & x_logistics=7) | (x_seg=0 & x_runs<N_runs & x_robot=2 & x_logistics=7) | (x_seg=(N_segments_shed-N_segments_collect) & x_robot=1 & x_logistics=8));
// for update flag_operation in the case of picking scenario
formula condition_4 = (scenario=2) & (((flag_task_finished=false) & (x_seg=0 & x_rows=0 & x_robot=0 & x_picking=0)) | (x_seg=(N_segments_shed-N_segments_collect) & x_trays=0 & x_robot=1 & x_picking=1) | (x_seg=0 & x_trays=0 & x_robot=2 & x_picking=2) | (x_seg=N_segments_collect & x_robot=1 & x_picking=3) | (x_trays=N_trays & x_robot=6 & x_picking=4) | (x_seg=N_segments_collect & x_robot=2 & x_picking=5) | (x_seg=0 & x_runs=N_runs & x_robot=2 & x_picking=6) |  (x_seg=0 & x_runs<N_runs & x_robot=2 & x_picking=6) | (x_seg=(N_segments_shed-N_segments_collect) & x_robot=1 & x_picking=7));
// for update flag_human in the case of operations at polytunnel  & in general
formula condition_5 = (((x_uvc=2 & x_rows<N_rows) | ((x_logistics=4 | x_picking=4) & x_trays<N_trays & x_runs<N_runs)) & (training=false & x_human=0)) | ((((x_logistics=4 | x_picking=4) & x_trays<N_trays & x_runs<N_runs))  & (training=true & x_human=0)) | ((x_logistics=5) & (x_human=0)) | ((x_uvc>=1 | x_logistics>=1 | x_picking>=1)  & (x_human_distance=0 & (x_robot=9 | x_human_motion=2) & x_human_aware=1 & x_human!=0));    
// for update flag_human in the case of operations at footpaths
formula condition_6 = (x_human=0) & ((((x_uvc=1 | x_uvc=3) & x_seg<N_segments_shed) | ((x_logistics=1 | x_logistics=8 | x_picking=1 | x_picking=7) & x_runs<N_runs & x_seg<(N_segments_shed-N_segments_collect)) | ((x_logistics=3 | x_logistics=6 | x_picking=3 | x_picking=5) & x_runs<N_runs & x_seg<N_segments_collect)) | (x_logistics=2 | x_logistics=7 | x_picking=2 | x_picking=6));    
// for update flag_aware 
formula condition_7 = ((x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_robot!=10 & x_human_distance>=1 & x_visual>=1 & x_human=1 & x_human_aware=0)) | ((x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_logistics!=2 & x_logistics!=5 & x_logistics!=7 & x_picking!=2 & x_picking!=6) & (x_robot!=10 & x_human_distance>=1 & x_visual>=1 & x_human=2 & x_human_aware=0)) | ((x_logistics=2 | x_logistics=5 | x_logistics=7 | x_picking=2 | x_picking=6) & (x_human=2 & x_human_aware=0)) | ((x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_human=0 & x_human_aware=1));
// for update flag_motion in the case of  robot is navigating inside polytunnels
formula condition_8 = ((x_robot>=4 & x_robot<=7) & ((x_human=1 & x_human_distance!=5 & x_human_aware=0 & x_human_motion=0) | (x_human=1 & x_human_aware=1 & x_human_motion=1))) | (((x_logistics=4 | x_picking=4) & x_robot=7) & ((x_human=2 & x_human_distance!=5 & x_human_aware=0 & x_human_motion=0) | (x_human=2 & x_human_aware=1 & x_human_motion=1)));
// for update flag_motion in the case of  robot is navigating outside polytunnels
formula condition_9 = (x_robot=1 | x_robot=2 | x_robot=0) & ((x_human!=0 & x_human_distance!=5 & x_human_aware=0 & x_human_motion=0) | (x_human!=0 & x_human_aware=1 & x_human_motion=1));
// for update flag_motion in the case of  robot is evading people
formula condition_10 = (x_robot=3) & ((x_human!=0 & x_human_distance=3 & x_human_motion=0) | (x_human!=0 & x_human_motion=4));
// for update flag_motion in the case picker approaching to place trays on the robot (planned or unplanned)
formula condition_11 = ((x_robot=6 | x_robot=5) & (x_human=2 & x_human_distance=3 & x_human_motion=0)) | ((x_robot=8) & (x_human=2 & x_human_distance=3 & x_human_motion=0)) | ((x_robot=8 | x_robot=6 | x_robot=5) & (x_human_distance!=4 & x_human=2 & x_human_motion=3));
// for update flag_motion in general cases after aa injury, when human is not longer present or when the robot performed a safety stop
formula condition_12 = (x_human_distance=5 & x_human!=0 & x_human_motion!=2) | (x_human=0 & x_human_motion=2) | (x_human!=0 & x_robot=10 & x_human_motion!=2);
// for update flag_distance
formula condition_13 = (x_human!=0 & x_human_motion=1 & x_human_distance<=3) | (x_human!=0 & x_human_motion=0 & (x_robot=1 | x_robot=2 | (x_robot>=4 & x_robot<=6)) & x_human_distance<=3) | (x_human!=0 & (x_human_motion=2 | x_robot=9) & x_human_distance>=1) | ((x_robot=8 | x_robot=6 | x_robot=5) & (x_human=2 & (x_human_motion=3 | x_htmis!=1 | x_hars=2) & x_human_distance=3)) | ((x_robot=3) & (x_human!=0 & (x_human_motion=4 | x_htmis!=1 | x_hars=2) & x_human_distance=3)) | ((x_robot=8 | x_robot=6 | x_robot=5 | x_robot=3) & (x_human!=0 & x_human_motion=0 & x_htmis=1 & x_hars!=2 & x_human_distance=3)) | (x_robot!=10 & x_human!=0 & x_human_distance=4);
// for update flag_gesture
formula condition_14 = (x_logistics>=1 | x_picking>=1) & (((x_human_distance>=2 & x_human_distance<=3) & x_human_aware=1 & x_human=2 & x_human_gesture=0) | (x_human=0 & x_human_gesture!=0));
// for update flag_hds in case of operations inside polytunnels
formula condition_15 = ((x_uvc=2) & (x_human!=0 & x_human_distance=1 & x_hds=0)) | ((x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & ((x_human!=0 & x_human_distance=2 & x_hds<=1) | (x_human!=0 & x_human_distance=3 & x_hds<=2) | (x_human!=0 & x_human_distance=4 & x_hds<=3) | (x_human!=0 & x_human_distance=3 & x_hds=4) | (x_human!=0 & x_human_distance=2 & x_hds=3) | (x_human!=0 & x_human_distance=1 & x_hds=2) | (x_human_distance=0 & x_human=0 & x_hds=1))); 
// for update flag_hds in case of operations outside polytunnels
formula condition_16 = ((x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0))) & ((x_human!=0 & x_human_distance=1 & x_hds=0) | (x_human!=0 & x_human_distance=2 & x_hds<=1) | (x_human!=0 & x_human_distance=3 & x_hds<=2) | (x_human!=0 & x_human_distance=4 & x_hds<=3) | (x_human!=0 & x_human_distance=3 & x_hds=4) | (x_human!=0 & x_human_distance=2 & x_hds=3) | (x_human!=0 & x_human_distance=1 & x_hds=2) | (x_human=0 & x_human_distance=0 & x_hds=1));
// for update flag_htmis in case of operations inside polytunnels
formula condition_17 = (x_logistics=4 | x_logistics=5 | x_picking=4) & ((x_hds>=3 & x_htmis=0) | (x_hds=0 & x_htmis!=0));
// for update flag_htmis in case of operations outside polytunnels
formula condition_18 =  (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & ((x_hds>=3 & x_htmis=0) | (x_hds=0 & x_htmis!=0));
// for update flag_hars 
formula condition_19 = (x_logistics>=1 | x_picking>=1) & ((x_human_distance>=3 & x_human_gesture!=0 & x_hars=0) | (x_human=0 & x_hars!=0)); 
// for update flag_alerts 
formula condition_20 = (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & ((x_hds>=1 & x_visual!=1) | (x_hds=0 & x_visual!=0));
// for update flag_alerts_p
formula condition_21 = (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & ((x_robot=7 | x_robot=1 | (x_robot>=4 & x_robot<=6)) & x_visual=0);
// for update flag_robot in case of  moving at footpaths without human presence
formula condition_22 = ((x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & ((x_human=0 & x_robot=1) | (x_human=0 & x_robot=2)));
// for update flag_robot in case of  moving inside polytunnels without human presence
formula condition_23 = ((x_uvc=2) & (x_human=0 & x_robot=7)) | ((x_logistics=4) & (x_human=0 & x_robot=7)) | ((x_picking=4) & (x_human=0 & x_robot=7)) | ((x_uvc=2 | x_logistics=4 | x_picking=4) & (x_human=0 & (x_robot=4 | x_robot=5 | x_robot=6)));
// for update flag_robot in case of robot planned interactions inside polytunnel
formula condition_24 = (x_logistics=5) & ((x_human=2 & x_human_distance=3 & x_human_motion=0 & x_robot=5) | (x_human=2 & x_human_distance=2 & x_robot=8) | (x_human=0 & x_robot=9));
// for update flag_robot in case of robot planned interactions outside polytunnel
formula condition_25 = (x_logistics=2 | x_logistics=7 | x_picking=2 | x_picking=6) & ((x_human=2 & x_human_distance=3 & x_human_motion=0 & x_robot=1) | (x_human=2 & x_human_distance=2 & x_robot=8) | (x_human=0 & x_robot=9));
// for update flag_robot in case of robot performing evasive maneuvers
formula condition_26 = (x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & ((x_human!=0 & x_human_distance=3 & x_human_motion=0 & (x_robot=1 | x_robot=2)) | (x_human!=0 & x_human_distance=2 & x_robot=3) | (x_human=0 & x_robot=9));
// for update flag_robot in case of safety stops triggered by LiDAR, gesture recognition directive or after collision
formula condition_27 = ((x_uvc=2) & (x_human!=0 & x_hds=2 & x_robot!=10)) | ((x_uvc>=1 | x_logistics>=1 | x_picking>=1 ) & (x_human!=0 & ((x_hds=4) | (x_hars=1 & x_human_distance=4)) & x_robot!=10)) | (x_human!=0 & x_human_distance=5 & x_robot!=10);       
// for update flag_robot in case of Robot restart motion after safety stops  
formula condition_28 = ((x_uvc=1 | x_uvc=3 | (x_logistics!=0 & x_logistics!=4 & x_logistics!=5) | (x_picking!=0 & x_picking!=4)) & (x_human=0 & x_robot=10)) | ((x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human=0 & x_robot=10));      
// for update flag_scs   
formula condition_29 = (x_human_distance=5 & x_scs=0) | (x_scs!=0);
// for update flag_task in case of uvc scenarios 
formula condition_30 = (x_uvc=2 & (x_human=0 & x_rows<N_rows & x_robot=7)) | ((x_uvc=1 | x_uvc=3) & (x_human=0 & x_seg<N_segments_shed & x_robot=2));        
// for update flag_task in case of logistics and picking scenarios 
formula condition_31 = ((x_logistics=5 | x_picking=4) & (x_human=0 & x_trays<N_trays & x_robot=7)) | ((x_logistics=3 | x_logistics=6 | x_picking=3 | x_picking=5) & (x_human=0 & x_seg<N_segments_collect & x_robot=2)) | ((x_logistics=1 | x_logistics=8 | x_picking=1 | x_picking=7) & (x_human=0 & x_seg<(N_segments_shed-N_segments_collect) & x_robot=2)) | ((x_logistics=7 | x_picking=6) & (x_human=0 & x_runs<N_runs & x_robot=2 & x_seg=0));        
///////////////////////////////////////////////////////////////////////////

module HRI
    //// VARIABLES DEFINITION /////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    x_uvc : [0..3] init 0;
        // 0 - robot at the shed
        // 1 - robot moving from robot shed to polytunnel (O-G1)
        // 2 - robot performing the UV-C treatment (O-U1)
        // 3 - robot moving from polytunnel to robot shed (O-G1)
    x_logistics : [0..8] init 0;
        // 0 - robot at the shed
        // 1 - robot moving from robot shed to collection point to place empty trays on it (O-G1)
        // 2 - robot interacting with the worker who load empty trays on it (O-L1)
        // 3 - robot moving from collection point to polytunnel after a human summoned it (O-G1)
        // 4 - robot moving inside the polytunnel (O-L2)
        // 5 - robot interacting with the human who summoned it (O-L1)
        // 6 - robot moving from polytunnel to collection point to unload full trays (O-G1)
        // 7 - robot interacting with the worker who unload full trays (O-L1)
        // 8 - robot moving from collection point to robot shed (O-G1)
    x_picking : [0..7] init 0;
        // 0 - robot at the shed
        // 1 - robot moving from robot shed to collection point to place empty trays on it (O-G1)
        // 2 - robot interacting with the worker who load empty trays on it (O-L1)
        // 3 - robot moving from collection point to polytunnel (O-G1)
        // 4 - robot performing picking task (O-P1)
        // 5 - robot moving from polytunnel to collection point to unload full trays (O-G1)
        // 6 - robot interacting with the worker who unload full trays (O-L1)
        // 7 - robot moving from collection point to robot shed (O-G1)
    ///////////////////////////////////////////////////////////////////////
    x_human: [0..2] init 0; // human presence, trained or untrained
        // 0 - no human presence
        // 1 - untrained human is interacting with the robot
        // 2 - trained human is interacting with the robot
    x_human_aware: [0..1] init 0; // human knowlege about robot intentions or potential danger 
        // 0 - human is not aware of robot presence or potential danger of approaching to the robot
        // 1 - human is aware of robot intentions or danger of approaching to the robot
    x_human_motion: [0..4] init 0; // human movements
        //(the human is stationary by defalt)
        // 0 - human is stationary (picking) or waiting for robot action - (for any people)
        // 1 - human is moving to the robot position - (for untrained people)
        // 2 - human is moving away from the robot position - (for untrained people)
        // 3 - human approaching to the robot to place trays on it - (for trained people only when robot is within 0-3.6m)
        // 4 - human walking next to the robot along the footpath - (for any people only when robot is within 0-3.6m)
    x_human_distance: [0..4] init 0; // distance between human and robot
        // General
        // 0 - human is too far from the robot
        // 1 - human farther than 7 m from the robot
        // 2 - human within 3.6-7m from the robot
        // 3 - human within 1.2-3.6 m from the robot
        // 4 - human within 0-1.2 m from the robot
        // 5 - human collides with the robot
     x_human_gesture: [0..1] init 0; // human hand gestures
        // 0 - human is not performing any hand gesture
        // 1 - human (trained) performing a hand gesture to make the robot knows about his/her intentions, i.e. if he/she will be static or if will move to the robot position
    ///////////////////////////////////////////////////////////////////////
    x_hds : [0..4] init 0; // human detection system
        // 0 - no human detected
        // 1 - human detected above 7m
        // 2 - human detected within 3.6-7 on time
        // 3 - human detected within 1.2-3.6 on time 
        // 4 - human detected within 0-1.2m on time
    x_htmis : [0..2] init 0; // human tracking and prediction system
        // 0 - no human tracked
        // 1 - accurate human motion prediction
        // 2 - not reliable human motion prediction due to tracking errors or unpredictable human behavior
    x_hars : [0..2] init 0; // human gesture recognition system
        // 0 - no human gesture detected
        // 1 - correct human gesture recognized
        // 2 - wrong human gesture recognized 
    x_scs : [0..2] init 0; // collision detection
        // 0 - no contact detected
        // 1 - collision detected
        // 2 - collision is not detected on time 
    ///////////////////////////////////////////////////////////////////////
    x_robot : [0..11] init 0; // robot operation states
        // General
        // 0 - Robot operation is paused (start point)
        // Footpaths G1 ///////////////////////////////////
        // 1 - Robot moving along footpaths
        // 2 - Robot performing a transition between footpath segments
        // 3 - Robot evading a human at footpaths
        // Polytunnels U1,S1,L2 ////////////////////////////////////
        // 4 - Robot moving along the row performing UV-C treatment
        // 5 - Robot moving along a row transporting trays
        // 6 - Robot moving along a row while picking fruits
        // 7 - Robot performing a transition between rows
        // Planned interactions L1 //////////////////////////////
        // 8 - Robot approaching to the human worker position (reducing the speed)
        // Planned and unplanned interactions  ////////////////////////////////////
        // 9 - Robot moving away from the human position 
        // Safety stops
        // 10- Robot stops because of safety purposes
    ///////////////////////////////////////////////////////////////////////
    x_visual : [0..2] init 0; // visual feedback alerts
        // 0 - Alerts are not activated
        // 1 - Robot visual indicators are actived when a human is detected
        // 2 - Periodic alerts activated (even when there is not human detection)
    x_voice : [0..2] init 0; // audio feedback alerts
        // 0 - Alerts are not activated
        // 1 - Robot voice messages are actived when a human is detected
        // 2 - Periodic alerts activated (even when there is not human detection)
    
    ///////////////////////////////////////////////////////////////////////
    x_rows: [0..N_rows] init 0 ; //counter for the number of rows covered by the robot during UVC treatment
    x_seg: [0..N_segments_shed] init 0 ; //counter for the number of footpath segments traversed by the robot during any operation outside the polytunnel
    x_runs: [0..N_runs] init 0 ; //counter for the number of times the robot is unloading trays at the collection point
    x_trays: [0..N_trays] init 0 ; //counter for the number of full trays that the robot is transporting during logistics    
    ///////////////////////////////////////////////////////////////////////
    flag_operation: bool init false; // flag to know if a command to update any variable in this module was executed
    flag_human: bool init false; // flag to know if a command to update x_human_training was executed
    flag_aware: bool init false; // flag to know if a command to update x_human_aware was executed
    flag_motion: bool init false; // flag to know if a command to update x_human_motion was executed
    flag_distance: bool init false; // flag to know if a command to update x_human was executed
    flag_gesture: bool init false; // flag to know if a command to update x_human_gesture was executed
    flag_hds: bool init false; // flag to know if a command to update x_hds was executed
    flag_htmis: bool init false; // flag to know if a command to update x_htmis was executed
    flag_hars: bool init false; // flag to know if a command to update x_hars was executed
    flag_alerts: bool init false; // flag to know if a command to update x_visual & x_voice were executed
    flag_alerts_p: bool init false; // flag to know if a command to update x_visual & x_voice were executed in case of periodic alerts
    flag_robot: bool init false; // flag to know if a command to update x_robot was executed
    flag_scs: bool init false; // flag to know if a command to update x_scs was executed
    flag_task: bool init false; // flag to know if a command to update any variable in this module was executed
    flag_task_finished: bool init false; //to know if the simulation has ended
    
    ///////////////////////////////////////////////////////////////////////
    //AGRICULTURAL TASKS///////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    
    //UV-C scenario////////////////////////////////////////////////////////
    // No transitions
    [all] ((condition_1=false & condition_3=false & condition_4=false) | x_human!=0) & ( flag_operation=false) -> (flag_operation'=true); //to stay in the same operation till the human is no longer interacting with the robot
    // 0 to 1
    [G1] (flag_task_finished=false) & (scenario=1) & (x_seg=0 & x_rows=0 & x_robot=0 & x_uvc=0) & ( flag_operation=false) -> (x_uvc'=1)&(x_robot'=1)&(flag_operation'=true); //x_robot is also updated to fix bugs
    // 1 to 2
    [U1] (scenario=1) & (x_seg=N_segments_shed & x_rows=0 & x_robot=1 & x_uvc=1) & ( flag_operation=false) -> (x_uvc'=2)&(x_robot'=4)&(x_seg'=0)&(flag_operation'=true); //x_robot, x_segment are also updated to fix bugs
    // 2 to 3
    [G1] (scenario=1) & (x_seg=0 & x_rows=N_rows & x_robot=4 & x_uvc=2) & ( flag_operation=false) -> (x_uvc'=3)&(x_robot'=1)&(x_rows'=0)&(flag_operation'=true); //x_robot, x_rows are also updated to fix bugs
    // 3 to 0
    [G1] (scenario=1) & (x_seg=N_segments_shed & x_robot=1 & x_uvc=3) & ( flag_operation=false) -> (x_uvc'=0)&(x_robot'=0)&(x_seg'=0)&(flag_task_finished'=true)&(flag_operation'=true); //x_robot, x_segment are also updated to fix bugs
    
    //Logistics scenario //////////////////////////////////////////////////
    // 0 to 1
    [G1] (flag_task_finished=false) & (scenario=3) & (x_seg=0 & x_rows=0 & x_robot=0 & x_logistics=0) & ( flag_operation=false ) -> (x_logistics'=1)&(x_robot'=1)&(flag_operation'=true); //x_robot is also updated to fix bugs
    // 1 to 2
    [L1] (scenario=3) & (x_seg=(N_segments_shed-N_segments_collect) & x_trays=0 & x_robot=1 & x_logistics=1) & ( flag_operation=false) -> (x_logistics'=2)&(x_seg'=0)&(flag_operation'=true); //x_seg is updated to fix bugs
    // 2 to 3
    [G1] (scenario=3) & (x_seg=0 & x_trays=0 & x_robot=2 & x_logistics=2) & ( flag_operation=false) -> (x_logistics'=3)&(flag_operation'=true);
    // 3 to 4
    [L2] (scenario=3) & (x_seg=N_segments_collect & x_robot=1 & x_logistics=3) & ( flag_operation=false) -> (x_logistics'=4)&(x_robot'=5)&(x_seg'=0)&(flag_operation'=true); //x_robot, x_seg are also updated to fix bugs
    // 4 to 5
    [L1] (scenario=3) & (x_human=0 & x_robot=5 & x_logistics=4) & ( flag_operation=false) -> (x_logistics'=5)&(flag_operation'=true);
    // 5 to 6
    [G1] (scenario=3) & (x_trays=N_trays & x_robot=7 & x_logistics=5) & ( flag_operation=false) -> (x_logistics'=6)&(x_trays'=0)&(x_robot'=1)&(flag_operation'=true); //x_trays, x_robot are also updated to fix bugs
    // 5 to 4
    [G1] (scenario=3) & (x_trays<N_trays & x_robot=7 & x_logistics=5) & ( flag_operation=false) -> (x_logistics'=4)&(flag_operation'=true);
    // 6 to 7
    [L1] (scenario=3) & (x_seg=N_segments_collect & x_robot=2 & x_logistics=6) & ( flag_operation=false) -> (x_logistics'=7)&(x_robot'=1)&(x_seg'=0)&(flag_operation'=true); //x_robot, x_seg are also updated to fix bugs
    // 7 to 8
    [G1] (scenario=3) & (x_seg=0 & x_runs=N_runs & x_robot=2 & x_logistics=7) & ( flag_operation=false) -> (x_logistics'=8)&(x_runs'=0)&(flag_operation'=true); //x_runs is also updated to fix bugs
    // 7 to 3
    [G1] (scenario=3) & (x_seg=0 & x_runs<N_runs & x_robot=2 & x_logistics=7) & ( flag_operation=false) -> (x_logistics'=3)&(flag_operation'=true);    
    // 8 to 0
    [G1] (scenario=3) & (x_seg=(N_segments_shed-N_segments_collect) & x_robot=1 & x_logistics=8) & ( flag_operation=false) -> (x_logistics'=0)&(x_robot'=0)&(x_seg'=0)&(flag_task_finished'=true)&(flag_operation'=true); //x_robot, x_seg are also updated to fix bugs
    
    //Picking scenario/////////////////////////////////////////////////////
    // 0 to 1
    [G1] (flag_task_finished=false) & (scenario=2) & (x_seg=0 & x_rows=0 & x_robot=0 & x_picking=0) & ( flag_operation=false) -> (x_picking'=1)&(x_robot'=1)&(flag_operation'=true); //x_robot is also updated to fix bugs
    // 1 to 2
    [L1] (scenario=2) & (x_seg=(N_segments_shed-N_segments_collect) & x_trays=0 & x_robot=1 & x_picking=1) & ( flag_operation=false) -> (x_picking'=2)&(x_seg'=0)&(flag_operation'=true); //x_seg is also updated to fix bugs
    // 2 to 3
    [G1] (scenario=2) & (x_seg=0 & x_trays=0 & x_robot=2 & x_picking=2) & ( flag_operation=false) -> (x_picking'=3)&(flag_operation'=true);
    // 3 to 4
    [P1] (scenario=2) & (x_seg=N_segments_collect & x_robot=1 & x_picking=3) & ( flag_operation=false) -> (x_picking'=4)&(x_robot'=6)&(x_seg'=0)&(flag_operation'=true); //x_robot, x_seg is also updated to fix bugs
    // 4 to 5
    [G1] (scenario=2) & (x_trays=N_trays & x_robot=6 & x_picking=4) & ( flag_operation=false) -> (x_picking'=5)&(x_trays'=0)&(x_robot'=1)&(flag_operation'=true); // x_trays, x_robot are also updated to fix bugs
    // 5 to 6
    [L1] (scenario=2) & (x_seg=N_segments_collect & x_robot=2 & x_picking=5) & ( flag_operation=false) -> (x_picking'=6)&(x_robot'=1)&(x_seg'=0)&(flag_operation'=true); //x_robot, x_seg are also updated to fix bugs
    // 6 to 7
    [G1] (scenario=2) & (x_seg=0 & x_runs=N_runs & x_robot=2 & x_picking=6) & ( flag_operation=false) -> (x_picking'=7)&(x_runs'=0)&(flag_operation'=true); // x_runs is also updated to fix bugs
    // 6 to 3
    [G1] (scenario=2) & (x_seg=0 & x_runs<N_runs & x_robot=2 & x_picking=6) & ( flag_operation=false) -> (x_picking'=3)&(flag_operation'=true);    
    // 7 to 0
    [G1] (scenario=2) & (x_seg=(N_segments_shed-N_segments_collect) & x_robot=1 & x_picking=7) & ( flag_operation=false) -> (x_picking'=0)&(x_robot'=0)&(x_seg'=0)&(flag_task_finished'=true)&(flag_operation'=true); //x_robot, x_seg are also updated to fix bugs
   
    //Task monitoring//////////////////////////////////////////////////////
    // No transitions
    [all] (condition_30=false & condition_31=false) & (flag_scs=true & flag_task=false) -> (flag_task'=true); //to stay in the same operation till the human is no longer interacting with the robot (for UV-C treatment)
    //UV-C treatment inside polytunnels
    [U1aS1] (x_uvc=2) & (x_human=0 & x_rows<N_rows & x_robot=7) & (flag_scs=true & flag_task=false)-> (x_rows'=x_rows+1)&(flag_task'=true); //to increse x_rows
    //Logistics & picking inside polytunnels
    [L2aP1] (x_logistics=5 | x_picking=4) & (x_human=0 & x_trays<N_trays & x_robot=7) & (flag_scs=true & flag_task=false)-> (x_trays'=x_trays+1)&(flag_task'=true); //to increse x_trays
    //UV-C treatment outside the polytunnels
    [G1] (x_uvc=1 | x_uvc=3) & (x_human=0 & x_seg<N_segments_shed & x_robot=2) & (flag_scs=true & flag_task=false)-> (x_seg'=x_seg+1)&(flag_task'=true); //to increse x_seg
    //Logistics and Picking outside the polytunnels
    [G1] (x_logistics=3 | x_logistics=6 | x_picking=3 | x_picking=5) & (x_human=0 & x_seg<N_segments_collect & x_robot=2) & (flag_scs=true & flag_task=false)-> (x_seg'=x_seg+1)&(flag_task'=true); //to increse x_seg
    [G1] (x_logistics=1 | x_logistics=8 | x_picking=1 | x_picking=7) & (x_human=0 & x_seg<(N_segments_shed-N_segments_collect) & x_robot=2) & (flag_scs=true & flag_task=false)-> (x_seg'=x_seg+1)&(flag_task'=true); //to increse x_seg
    [L1] (x_logistics=7 | x_picking=6) & (x_human=0 & x_runs<N_runs & x_robot=2 & x_seg=0) & (flag_scs=true & flag_task=false)-> (x_runs'=x_runs+1)&(flag_task'=true); // to consider a change in x_run only after the empty trays were placed on the robot
    //To reset the flags
    [all] flag_task=true -> (flag_operation'=false)&(flag_human'=false)&(flag_aware'=false)&(flag_motion'=false)&(flag_distance'=false)&(flag_gesture'=false)&(flag_hds'=false)&(flag_htmis'=false)&(flag_hars'=false)&(flag_alerts'=false)&(flag_alerts_p'=false)&(flag_scs'=false)&(flag_robot'=false)&(flag_task'=false);   

    ///////////////////////////////////////////////////////////////////////
    //HUMAN BEHAVIOUR//////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
   
    // x_human ////////////////////////////////////////////////////////////
    // No transition
    [all_G1] (condition_5=false & condition_6=false) & (flag_operation=true & flag_human=false) -> (flag_human'=true);
    //Polytunnels ////
    // 0 to 1 untrained people
    [all_G1] ((x_uvc=2 & x_rows<N_rows) | ((x_logistics=4 | x_picking=4) & x_trays<N_trays & x_runs<N_runs)) & (training=false & x_human=0) & (flag_operation=true & flag_human=false) -> p_h_interact_1:(x_human'=1)&(flag_human'=true)+1-p_h_interact_1:(flag_human'=true);
    // 0 to 2 unplanned interactions with trained people
    [all_G1] (((x_logistics=4 | x_picking=4) & x_trays<N_trays & x_runs<N_runs))  & (training=true & x_human=0) & (flag_operation=true & flag_human=false) -> p_h_interact_2:(x_human'=2)&(flag_human'=true)+1-p_h_interact_2:(flag_human'=true);
    // 0 to 2 planned interactions
    [L1] (x_logistics=5) & (x_human=0) & (flag_operation=true & flag_human=false) -> (x_human'=2)&(flag_human'=true);    
    //Footpaths /////
    // 0 to 1 untrained people
    [G1aL1] (((x_uvc=1 | x_uvc=3) & x_seg<N_segments_shed) | ((x_logistics=1 | x_logistics=8 | x_picking=1 | x_picking=7) & x_runs<N_runs & x_seg<(N_segments_shed-N_segments_collect)) | ((x_logistics=3 | x_logistics=6 | x_picking=3 | x_picking=5) & x_runs<N_runs & x_seg<N_segments_collect)) & (training=false & x_human=0) & (flag_operation=true & flag_human=false) -> p_h_interact_3:(x_human'=1)&(flag_human'=true)+1-p_h_interact_3:(flag_human'=true);
    // 0 to 2 unplanned interactions with trained people
    [G1aL1] (((x_uvc=1 | x_uvc=3) & x_seg<N_segments_shed) | ((x_logistics=1 | x_logistics=8 | x_picking=1 | x_picking=7) & x_runs<N_runs & x_seg<(N_segments_shed-N_segments_collect)) | ((x_logistics=3 | x_logistics=6 | x_picking=3 | x_picking=5) & x_runs<N_runs & x_seg<N_segments_collect)) & (training=true & x_human=0) & (flag_operation=true & flag_human=false) -> p_h_interact_4:(x_human'=2)&(flag_human'=true)+1-p_h_interact_4:(flag_human'=true);    
    // 0 to 2 planned interactions
    [L1] (x_logistics=2 | x_logistics=7 | x_picking=2 | x_picking=6) & (x_human=0) & (flag_operation=true & flag_human=false) -> (x_human'=2)&(flag_human'=true);       
    //General ///
    // 1,2 to 0 
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1)  & (x_human_distance=0 & (x_robot=9 | x_human_motion=2) & x_human_aware=1 & x_human!=0) & (flag_operation=true & flag_human=false) -> (x_human'=0)&(flag_human'=true);
    
    // x_human_aware //////////////////////////////////////////////////////
    // No transition
    [all] (condition_7=false) & (flag_human=true & flag_aware=false) -> (flag_aware'=true);
    // 0 to 1 using only "visual" alerts for "untrained" human
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_robot!=10 & x_human_distance=1 & x_visual>=1 & x_human=1 & x_human_aware=0) & (flag_human=true & flag_aware=false) -> p_h_aware_1:(x_human_aware'=1)&(flag_aware'=true)+1-p_h_aware_1:(flag_aware'=true);
    // 0 to 1 using only "visual" alerts for "trained" human
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_logistics!=2 & x_logistics!=5 & x_logistics!=7 & x_picking!=2 & x_picking!=6) & (x_robot!=10 & x_human_distance=1 & x_visual>=1 & x_human=2 & x_human_aware=0) & (flag_human=true & flag_aware=false) -> p_h_aware_2:(x_human_aware'=1)&(flag_aware'=true)+1-p_h_aware_2:(flag_aware'=true);
    // 0 to 1 using "voice" and "visual" alerts for "untrained" human
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_robot!=10 & x_human_distance>=2 & x_voice>=1 & x_human=1 & x_human_aware=0) & (flag_human=true & flag_aware=false) -> max(p_h_aware_3,p_h_aware_1):(x_human_aware'=1)&(flag_aware'=true)+1-max(p_h_aware_3,p_h_aware_1):(flag_aware'=true); 
     // 0 to 1 using "voice" and "visual" alerts for "trained" human
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_logistics!=2 & x_logistics!=5 & x_logistics!=7 & x_picking!=2 & x_picking!=6) & (x_robot!=10 & x_human_distance>=2 & x_voice>=1 & x_human=2 & x_human_aware=0) & (flag_human=true & flag_aware=false) -> max(p_h_aware_3,p_h_aware_2):(x_human_aware'=1)&(flag_aware'=true)+1-max(p_h_aware_3,p_h_aware_2):(flag_aware'=true); 
    // 0 to 1 if the human is a worker who is planned to load/unload trays on the robot (inside and outside polytunnels)
    [G1_L1] (x_logistics=2 | x_logistics=5 | x_logistics=7 | x_picking=2 | x_picking=6) & (x_human=2 & x_human_aware=0) & (flag_human=true & flag_aware=false) -> (x_human_aware'=1)&(flag_aware'=true);
    // 1 to 0
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_human=0 & x_human_aware=1) & (flag_human=true & flag_aware=false) -> (x_human_aware'=0)&(flag_aware'=true);
       
    // x_human_motion /////////////////////////////////////////////////////
    // No transition
    [all] (condition_8=false & condition_9=false & condition_10=false & condition_11=false & condition_12=false) & (flag_aware=true & flag_motion=false) -> (flag_motion'=true);
    //While robot is navigating inside polytunnels/////////
    // 0 to 1 for "untrained" people
    [all_G1aL1] (x_robot>=4 & x_robot<=7) & (x_human=1 & x_human_distance!=5 & x_human_aware=0 & x_human_motion=0) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=1)&(flag_motion'=true);
    // 1 to 2 for "untrained" people
    [all_G1aL1] (x_robot>=4 & x_robot<=7) & (x_human=1 & x_human_aware=1 & x_human_motion=1) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=2)&(flag_motion'=true);
    // 0 to 1 for "trained" people at the end of the rows
    [all_G1aL1] ((x_logistics=4 | x_picking=4) & x_robot=7) & (x_human=2 & x_human_distance!=5 & x_human_aware=0 & x_human_motion=0) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=1)&(flag_motion'=true);
    // 1 to 2 for "trained" people at the end of the rows
    [all_G1aL1] ((x_logistics=4 | x_picking=4) & x_robot=7) & (x_human=2 & x_human_aware=1 & x_human_motion=1) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=2)&(flag_motion'=true);
    //While robot is navigating outside polytunnels/////////
    //(this one also includes when the robot is in pause and approaching, to fix bugs)
    // 0 to 1 for "untrained" and "trained" people
    [G1] (x_robot=1 | x_robot=2 | x_robot=0) & (x_human!=0 & x_human_distance!=5 & x_human_aware=0 & x_human_motion=0) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=1)&(flag_motion'=true); 
    // 1 to 0,2 for "untrained" and "trained" people
    [G1] (x_robot=1 | x_robot=2  | x_robot=0) & (x_human!=0 & x_human_aware=1 & x_human_motion=1) & (flag_aware=true & flag_motion=false) -> 1-p_h_decision_1:(x_human_motion'=0)&(flag_motion'=true)+p_h_decision_1:(x_human_motion'=2)&(flag_motion'=true);      
    //While robot is evading a human/////////
    // 0 to 4,0 for "untrained" and "trained" people
    [G1] (x_robot=3) & (x_human!=0 & x_human_distance=3 & x_human_motion=0) & (flag_aware=true & flag_motion=false) -> 1-p_h_decision_1:(x_human_motion'=4)&(flag_motion'=true)+p_h_decision_1:(x_human_motion'=0)&(flag_motion'=true);
    // 4 to 2 for "untrained" and "trained" people
    [G1] (x_robot=3) & (x_human!=0 & x_human_motion=4) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=2)&(flag_motion'=true);
    //When a picker is approaching to place trays on the robot/////////
    // 0 to 3,2 for unplanned interactions
    [L2aP1] (x_robot=6 | x_robot=5) & (x_human=2 & x_human_distance=3 & x_human_motion=0) & (flag_aware=true & flag_motion=false) -> 1-p_h_decision_1:(x_human_motion'=3)&(flag_motion'=true)+p_h_decision_1:(x_human_motion'=2)&(flag_motion'=true);
    // 0 to 3,0 for planned interactions
    [L1] (x_robot=8) & (x_human=2 & x_human_distance=3 & x_human_motion=0) & (flag_aware=true & flag_motion=false) -> 1-p_h_decision_1:(x_human_motion'=3)&(flag_motion'=true)+p_h_decision_1:(x_human_motion'=0)&(flag_motion'=true);    
    // 3 to 2 in general for planned or unplanned interactions
    [L2aL1aP1] (x_robot=8 | x_robot=6 | x_robot=5) & (x_human_distance!=4 & x_human=2 & x_human_motion=3) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=2)&(flag_motion'=true);
    // In general after a injury
    // from any human motion to 2
    [all] (x_human_distance=5 & x_human!=0 & x_human_motion!=2) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=2)&(flag_motion'=true);
    //In general when the human is not longer present
    // 2 to 0 for "trained" and "untrained" people
    [all] (x_human=0 & x_human_motion=2) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=0)&(flag_motion'=true);
    // In general if the robot performed a safety stop    
    // from any to 2
    [all] (x_human!=0 & x_robot=10 & x_human_motion!=2) & (flag_aware=true & flag_motion=false) -> (x_human_motion'=2)&(flag_motion'=true);
    
    // x_human_distance ///////////////////////////////////////////////////  
    // No transition
    [all] (condition_13=false) & (flag_motion=true & flag_distance=false) -> (flag_distance'=true);
    // n to n+1 when human is moving to robot position (0-4)
    [all] (x_human!=0 & x_human_motion=1 & x_human_distance<=3) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=x_human_distance+1)&(flag_distance'=true);
    // n to n+1 when robot is moving to human position (0-4)
    [all] (x_human!=0 & x_human_motion=0 & (x_robot=1 | x_robot=2 | (x_robot>=4 & x_robot<=6)) & x_human_distance<=3) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=x_human_distance+1)&(flag_distance'=true);
    // n to n-1 in general (5-0)
    [all] (x_human!=0 & (x_human_motion=2 | x_robot=9) & x_human_distance>=1) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=x_human_distance-1)&(flag_distance'=true);
    //When human decides to approach to the robot to place trays on it (planned or unplanned) OR when the HTMIS or hgrs are not working well 
    // 3 to 4 
    [L1] (x_robot=8 | x_robot=6 | x_robot=5) & (x_human=2 & (x_human_motion=3 | x_htmis!=1 | x_hars=2) & x_human_distance=3) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=4)&(flag_distance'=true);
    //When human decides to walk next to the robot during evasive maneuvers OR when the HTMIS or hgrs are not working well
    // 3 to 4 
    [G1] (x_robot=3) & (x_human!=0 & (x_human_motion=4 | x_htmis!=1 | x_hars=2) & x_human_distance=3) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=4)&(flag_distance'=true);
    // 3 to 2 in General if the safety system worked well after approaching (planned and unplanned) or evasive maneuvers
    [G1] (x_robot=8 | x_robot=6 | x_robot=5 | x_robot=3) & (x_human!=0 & x_human_motion=0 & x_htmis=1 & x_hars!=2 & x_human_distance=3) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=2)&(flag_distance'=true);
    // 4 to 5 in General if the the robot didn't activate the safety stop when human was <=1.2m
    [G1] (x_robot!=10 & x_human!=0 & x_human_distance=4) & (flag_motion=true & flag_distance=false) -> (x_human_distance'=5)&(x_human_aware'=1)&(flag_distance'=true);    
    
    // x_human_gesture ////////////////////////////////////////////////////
    // No transition
    [all_U1aS1] (condition_14=false) & (flag_distance=true & flag_gesture=false) -> (flag_gesture'=true);
    // 0 to 1 Planned and unplanned interactions
    [all_U1aS1] (x_logistics>=1 | x_picking>=1) & ((x_human_distance>=2 & x_human_distance<=3)  & x_human_aware=1 & x_human=2 & x_human_gesture=0) & (flag_distance=true & flag_gesture=false) ->  p_h_reply_1:(x_human_gesture'=1)&(flag_gesture'=true)+1- p_h_reply_1:(flag_gesture'=true);
    // 1 to 0
    [all_U1aS1] (x_logistics>=1 | x_picking>=1) & (x_human=0 & x_human_gesture!=0) & (flag_distance=true & flag_gesture=false) -> (x_human_gesture'=0)&(flag_gesture'=true);
    
    ///////////////////////////////////////////////////////////////////////  
    //SAFETY SYSTEM ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    
    //Human Detection System (HDS) ////////////////////////////////////////
    //No transition
    [all] (condition_15=false & condition_16=false) & (flag_gesture=true & flag_hds=false) -> (flag_hds'=true);
    //HDS inside polytunnels///////////////////////////////////////////////
    //0 to 1
    [U1] (x_uvc=2) & (x_human!=0 & x_robot!=7 & x_human_distance=1 & x_hds=0) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_1:(x_hds'=1)&(flag_hds'=true)+p_hds_fail_1:(flag_hds'=true); // to detect a human farther than 7m along the same row
    [U1] (x_uvc=2) & (x_human!=0 & x_robot=7 & x_human_distance=1 & x_hds=0) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_5:(x_hds'=1)&(flag_hds'=true)+p_hds_fail_5:(flag_hds'=true); // to detect a human farther than 7m at the end of the rows
    //0,1 to 2
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_robot!=7 & x_human_distance=2 & x_hds<=1) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_2:(x_hds'=2)&(flag_hds'=true)+p_hds_fail_2:(flag_hds'=true); // to detect a human entering the zone 3.6-7m along the same row
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_robot=7 & x_human_distance=2 & x_hds<=1) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_6:(x_hds'=2)&(flag_hds'=true)+p_hds_fail_6:(flag_hds'=true); // to detect a human entering the zone 3.6-7m at the end of the rows
    //0,1,2 to 3
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_robot!=7 & x_human_distance=3 & x_hds<=2) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_3:(x_hds'=3)&(flag_hds'=true)+p_hds_fail_3:(flag_hds'=true); // to detect a human entering the zone 1.2-3.6m along the same row
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_robot=7 & x_human_distance=3 & x_hds<=2) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_7:(x_hds'=3)&(flag_hds'=true)+p_hds_fail_7:(flag_hds'=true); // to detect a human entering the zone 1.2-3.6m at the end of the rows
    //0,1,2,3 to 4
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_robot!=7 & x_human_distance=4 & x_hds<=3) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_4:(x_hds'=4)&(flag_hds'=true)+p_hds_fail_4:(flag_hds'=true); // to detect a human entering the zone 0-1.2m along the same row
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_robot=7 & x_human_distance=4 & x_hds<=3) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_8:(x_hds'=4)&(flag_hds'=true)+p_hds_fail_8:(flag_hds'=true); // to detect a human entering the zone 0-1.2m at the end of the rows
    //4 to 3
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_human_distance=3 & x_hds=4) & (flag_gesture=true & flag_hds=false) -> (x_hds'=3)&(flag_hds'=true); // to detect humans leaving the zone 1.2-3.6m
    //3 to 2
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_human_distance=2 & x_hds=3) & (flag_gesture=true & flag_hds=false) -> (x_hds'=2)&(flag_hds'=true); // to detect humans leaving the zone 3.6-7m
    //2 to 1
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human!=0 & x_human_distance=1 & x_hds=2) & (flag_gesture=true & flag_hds=false) -> (x_hds'=1)&(flag_hds'=true); // to detect humans leaving the zone within 7m
    //1 to 0
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human_distance=0 & x_human=0 & x_hds=1) & (flag_gesture=true & flag_hds=false) -> (x_hds'=0)&(flag_hds'=true); // to stop detecting humans along the same row
    
    // HDS outside polytunnels/////////////////////////////////////////////
    //0 to 1
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=1 & x_hds=0) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_9:(x_hds'=1)&(flag_hds'=true)+p_hds_fail_9:(flag_hds'=true); // to detect a human farther than 7m along the same footpath
    //0,1 to 2
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=2 & x_hds<=1) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_9:(x_hds'=2)&(flag_hds'=true)+p_hds_fail_9:(flag_hds'=true); // to detect a human entering the zone 3.6-7m along the same footpath
    //0,1,2 to 3
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=3 & x_hds<=2) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_10:(x_hds'=3)&(flag_hds'=true)+p_hds_fail_10:(flag_hds'=true); // to detect a human entering the zone 1.2-3.6m along the same footpath  
    //0,1,2,3 to 4
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=4 & x_hds<=3) & (flag_gesture=true & flag_hds=false) -> 1-p_hds_fail_11:(x_hds'=4)&(flag_hds'=true)+p_hds_fail_11:(flag_hds'=true); // to detect a human entering the zone 0-1.2m along the same footpath
    //4 to 3
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=3 & x_hds=4) & (flag_gesture=true & flag_hds=false) -> (x_hds'=3)&(flag_hds'=true); // to detect humans leaving the zone 1.2-3.6m
    //3 to 2
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=2 & x_hds=3) & (flag_gesture=true & flag_hds=false) -> (x_hds'=2)&(flag_hds'=true); // to detect humans leaving the zone 3.6-7m
    //2 to 1
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human!=0 & x_human_distance=1 & x_hds=2) & (flag_gesture=true & flag_hds=false) -> (x_hds'=1)&(flag_hds'=true); // to detect humans leacing the zone within 7m
    //1 to 0
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_human=0 & x_human_distance=0 & x_hds=1) & (flag_gesture=true & flag_hds=false) -> (x_hds'=0)&(flag_hds'=true); // to stop detecting humans along the same footpath
    
    // Human Tracking and motion inference system (HTMIS)//////////////////
    //No transition
    [all_S1aU1] (condition_17=false & condition_18=false) & (flag_hds=true & flag_htmis=false) -> (flag_htmis'=true);
    // HTMIS inside polytunnels////////////////////////////////////////////
    //0 to 1,2
    [L2aL1aP1] (x_logistics=4 | x_logistics=5 | x_picking=4) & (x_hds>=3 & x_htmis=0) & (flag_hds=true & flag_htmis=false) -> 1-p_htmis_fail_1:(x_htmis'=1)&(flag_htmis'=true)+p_htmis_fail_1:(x_htmis'=2)&(flag_htmis'=true); //to fail in predicting future human movements due to algorithm limitations or human factor
    //1,2 to 0
    [L2aL1aP1] (x_logistics=4 | x_logistics=5 | x_picking=4) & (x_hds=0 & x_htmis!=0) & (flag_hds=true & flag_htmis=false) -> (x_htmis'=0)&(flag_htmis'=true); //to unactivate motion prediction when no human is detected   
    // HTMIS outside polytunnels///////////////////////////////////////////
    //0 to 1,2
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_hds>=3 & x_htmis=0) & (flag_hds=true & flag_htmis=false) -> 1-p_htmis_fail_2:(x_htmis'=1)&(flag_htmis'=true)+p_htmis_fail_2:(x_htmis'=2)&(flag_htmis'=true); //to fail in predicting future human movements due to algorithm limitations or human factor
    //1,2 to 0
    [G1aL1] (x_uvc=1 | x_uvc=3 | (x_logistics!=4 & x_logistics!=5 & x_logistics!=0) | (x_picking!=4 & x_picking!=0)) & (x_hds=0 & x_htmis!=0) & (flag_hds=true & flag_htmis=false) -> (x_htmis'=0)&(flag_htmis'=true); //to unactivate motion prediction when no human is detected
    // Human action recognition (HARS) ////////////////////////////////////
    //No transition
    [all_U1aS1] (condition_19=false) & (flag_htmis=true & flag_hars=false) -> (flag_hars'=true);
    //0 to 1,2 (all UV-C operations are not included since they are performed at night)
    [all_U1aS1] (x_logistics>=1 | x_picking>=1 ) & (x_human_distance>=3 & x_human_gesture!=0 & x_hars=0) & (flag_htmis=true & flag_hars=false) -> 1-p_hars_fail:(x_hars'=1)&(flag_hars'=true)+p_hars_fail:(x_hars'=2)&(flag_hars'=true); //to recognize or not hand gestures of a worker in planned interactions
    //1,2 to 0 
    [all_U1aS1] (x_logistics>=1 | x_picking>=1 ) & (x_human=0 & x_hars!=0) & (flag_htmis=true & flag_hars=false) -> (x_hars'=0)&(flag_hars'=true); //to unactivate hand gesture recognition when no human is detected
    
    // Visual & voice alerts///////////////////////////////////////////////
    //No transition
    [all] (condition_20=false) & (flag_robot=true & flag_alerts=false) -> (flag_alerts'=true);
    //0,2 to 1 when a human is detected
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_hds>=1 & x_visual!=1) & (flag_robot=true & flag_alerts=false) -> (x_visual'=1)&(x_voice'=1)&(flag_alerts'=true); //to activate visual and audio alerts after detecting a human
    //1,2 to 0 
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & (x_hds=0 & x_visual!=0) & (flag_robot=true & flag_alerts=false) -> (x_visual'=0)&(x_voice'=0)&(flag_alerts'=true); //to unactivate visual alerts when no human is detected 
    //Periodic alerts     
    //The periodic alerts are activated during robot movements along rows, row transition or footpaths 
    //No transition
    [all] (condition_21=false) & (flag_alerts=true & flag_alerts_p=false) -> (flag_alerts_p'=true);
    //0 to 2 Periodic alerts 
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1) & ((x_robot=7 | x_robot=1 | (x_robot>=4 & x_robot<=6)) & x_visual=0) & (flag_alerts=true & flag_alerts_p=false) -> p_periodic_alerts:(x_visual'=2)&(x_voice'=2)&(flag_alerts_p'=true)+1-p_periodic_alerts:(flag_alerts_p'=true); //to activate periodic visual and voice alerts to warn people about potential danger and robot presence
    
    // Safety contact system (SCS)/////////////////////////////////////////
    // No transition
    [all] (condition_29=false) & (flag_alerts_p=true & flag_scs=false) -> (flag_scs'=true);
    // 0 to 1,2
     [all] (x_human_distance=5 & x_scs=0) & (flag_alerts_p=true & flag_scs=false) -> p_contact_fail:(x_scs'=2)&(flag_scs'=true)+1-p_contact_fail:(x_scs'=1)&(flag_scs'=true);  //x_human_aware is also updated to fix bugs
    // from any hazard to 0
     [all] (x_scs!=0) & (flag_alerts_p=true & flag_scs=false) -> (x_scs'=0)&(flag_scs'=true);
                                                                                                                                                                    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //ROBOT OPERATION//////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    
    // No transition
    [all] (condition_22=false & condition_23=false & condition_24=false & condition_25=false & condition_26=false & condition_27=false & condition_28=false) & (flag_hars=true & flag_robot=false) -> (flag_robot'=true);
    //Robot moving outside footpaths without human presence
    // 1 to 2
    [G1] (x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & (x_human=0 & x_robot=1) & (flag_hars=true & flag_robot=false) -> (x_robot'=2)&(flag_robot'=true);
    // 2 to 1
    [G1] (x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & (x_human=0 & x_robot=2) & (flag_hars=true & flag_robot=false) -> (x_robot'=1)&(flag_robot'=true);    
    //Robot moving inside polytunnels without human presence
    // 7 to 4 uvc
    [U1] (x_uvc=2) & (x_human=0 & x_robot=7) & (flag_hars=true & flag_robot=false) -> (x_robot'=4)&(flag_robot'=true);
    //7 to 5 logistics
    [L2] (x_logistics=4) & (x_human=0 & x_robot=7) & (flag_hars=true & flag_robot=false) -> (x_robot'=5)&(flag_robot'=true);
    //7 to 6 picking
    [P1] (x_picking=4) & (x_human=0 & x_robot=7) & (flag_hars=true & flag_robot=false) -> (x_robot'=6)&(flag_robot'=true);
    //4,5,6 to 7 all scenarios
    [all_G1aL1] (x_uvc=2 | x_logistics=4 | x_picking=4) & (x_human=0 & (x_robot=4 | x_robot=5 | x_robot=6)) & (flag_hars=true & flag_robot=false) -> (x_robot'=7)&(flag_robot'=true);
    //Robot planned interactions inside polytunnels
    //5 to 8 logistics
    [L1] (x_logistics=5) & (x_human=2 & x_human_distance=3 & x_human_motion=0 & x_robot=5) & (flag_hars=true & flag_robot=false) -> (x_robot'=8)&(flag_robot'=true);
    //8 to 9 logistics
    [L1] (x_logistics=5) & (x_human=2 & x_human_distance=2 & x_robot=8) & (flag_hars=true & flag_robot=false) -> (x_robot'=9)&(flag_robot'=true);
    //9 to 7 logistics
    [L1] (x_logistics=5) & (x_human=0 & x_robot=9) & (flag_hars=true & flag_robot=false) -> (x_robot'=7)&(flag_robot'=true);
    //Robot planned interactions outside polytunnels
    //1 to 8 logistics & picking
    [L1] (x_logistics=2 | x_logistics=7 | x_picking=2 | x_picking=6) & (x_human=2 & x_human_distance=3 & x_human_motion=0 & x_robot=1) & (flag_hars=true & flag_robot=false) -> (x_robot'=8)&(flag_robot'=true);
    //8 to 9 logistics & picking
    [L1] (x_logistics=2 | x_logistics=7 | x_picking=2 | x_picking=6) & (x_human=2 & x_human_distance=2 & x_robot=8) & (flag_hars=true & flag_robot=false) -> (x_robot'=9)&(flag_robot'=true);
    //9 to 2 logistics & picking
    [L1] (x_logistics=2 | x_logistics=7 | x_picking=2 | x_picking=6) & (x_human=0 & x_robot=9) & (flag_hars=true & flag_robot=false) -> (x_robot'=2)&(flag_robot'=true);
    //Robot performing evasive maneuver    
    // 1,2 to 3 - only activated if the human is stationary
    [G1] (x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & (x_human!=0 & x_human_distance=3 & x_human_motion=0 & (x_robot=1 | x_robot=2)) & (flag_hars=true & flag_robot=false) -> (x_robot'=3)&(flag_robot'=true);   
    // 3 to 9
    [G1] (x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & (x_human!=0 & x_human_distance=2 & x_robot=3) & (flag_hars=true & flag_robot=false) -> (x_robot'=9)&(flag_robot'=true);   
    // 9 to 2
    [G1] (x_uvc=1 | x_uvc=3 | x_logistics=1 | x_logistics=3 | x_logistics=6 | x_logistics=8 | x_picking=1 | x_picking=3 | x_picking=5 | x_picking=7) & (x_human=0 & x_robot=9) & (flag_hars=true & flag_robot=false) -> (x_robot'=2)&(flag_robot'=true);
    //Robot safety stops triggered by LiDAR or gesture recognition directive
    // 4,7 to 10 only during uvc treatment (when human detected within 7m)
    [U1] (x_uvc=2) & (x_human!=0 & x_hds=2 & x_robot!=10) & (flag_hars=true & flag_robot=false) -> (x_robot'=10)&(x_human_aware'=1)&(flag_robot'=true); //x_human_aware is also updated to fix bugs
    // from any robot operation to 10 (when human detected within 1.2m)
    [all] (x_uvc>=1 | x_logistics>=1 | x_picking>=1 ) & (x_human!=0 & ((x_hds=4) | (x_hars=1 & x_human_distance=4)) & x_robot!=10) & (flag_hars=true & flag_robot=false) -> (x_robot'=10)&(x_human_aware'=1)&(flag_robot'=true); //x_human_aware is also updated to fix bugs
    //Robot safety stop triggered after collision
    // from any robot operation to 10
    [all] (x_human!=0 & x_human_distance=5 & x_robot!=10) & (flag_hars=true & flag_robot=false) -> (x_robot'=10)&(flag_robot'=true);
    //Robot restarts motion after safety stops    
    // 10 to 2 - after a stop outside polytunnels the robot is performing segment transition
    [G1] (x_uvc=1 | x_uvc=3 | (x_logistics!=0 & x_logistics!=4 & x_logistics!=5) | (x_picking!=0 & x_picking!=4)) & (x_human=0 & x_robot=10) & (flag_hars=true & flag_robot=false) -> (x_robot'=2)&(flag_robot'=true);
    // 10 to 7 - after a stop inside polytunnels the robot is performing row transition
    [all_G1] (x_uvc=2 | x_logistics=4 | x_logistics=5 | x_picking=4) & (x_human=0 & x_robot=10) & (flag_hars=true & flag_robot=false) -> (x_robot'=7)&(flag_robot'=true);   
        
endmodule


