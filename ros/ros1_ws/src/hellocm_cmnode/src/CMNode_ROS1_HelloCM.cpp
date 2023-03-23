/*!
 ******************************************************************************
 **  CarMaker - Version 7.1.2
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 *
 *
 * ToDo:
 * - C++!!!
 * - ROS naming/way/namespaces
 * - parameter: CarMaker read, ROS set by service?
 *   -> ROS parameter mechanism seems better solution!
 * - node/topic/... destruction to allow dynamic load/unload
 *   when TestRun starts instead of initialization at CarMaker startup
 * - New Param_Get() function to read parameters from Infofile
 * - ...
 *
 */


/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */
#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"

#include "Vehicle.h"        // lei
// #include "DrivMan.h"
// #include "Vehicle/Sensor_Object.h"
#include "DirectVarAccess.h"        // lei

/* ROS */
#include "cmrosutils/CMRosUtils.h"    /* Node independent templates, ...*/
#include "cmrosutils/CMRosIF_Utils.h" /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemote.h"      /* Basic service for CarMaker remote from ROS */

/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 1
#  include "hellocm/ROS1_HelloCM.h"  /* External ROS Node. Topic name, ... */
#else
#  include <hellocm_msgs/Ext2CM.h>
#  include <hellocm_msgs/CM2Ext.h>
#  include <hellocm_msgs/Init.h>
#endif

/*! String and numerical version of this Node
 *  - String:    e.g. <Major>.<Minor>.<Patch>
 *  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
 */
#define CMNODE_VERSION "0.8.0"
#define CMNODE_NUMVER  800


/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  warning "Debug options are enabled!"
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Not beautiful but consistent with external ROS Node
 * where ROS_INFO is used (implicit newline)*/
# define LOG(frmt, ...)  Log(frmt "\n", ##__VA_ARGS__)


/* General switches for CarMaker ROS Node */
typedef enum tCMNode_Mode {
    CMNode_Mode_Disabled  = 0,  /*!< Node is disabled. e.g. don't publish. */
    CMNode_Mode_Default   = 1,  /*!< Node is enabled, spinOnce is used  */
    CMNode_Mode_Threaded  = 2   /*!< Node is enabled, spin in parallel thread
                                     - Messages are received all the time
                                     - Data is updated at defined position, e.g. *_In()
                                     - Currently not implemented! */
} tCMNode_Mode;



/* Managing synchronization between CarMaker Node and other ROS nodes */
typedef enum tCMNode_SyncMode {
    CMNode_SyncMode_Disabled  = 0, /*!< No synchronization on CarMaker side */
    CMNode_SyncMode_Tpc       = 1  /*!< Buffer messages or Spin until external Topics are received */
} tCMNode_SyncMode;



/* Global struct for this Node */
static struct {
    unsigned long  CycleNoRel;  /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct {
        double  Duration;       /*!< Time spent for synchronization task */
        int     nCycles;        /*!< Number of cycles in synchronization loop */
        int     CyclePrepDone;  /*!< Last cycle when preparation was done */
        int     CycleJobDone;   /*!< Last cycle when job was done */
        double  SynthDelay;     /*!< Synthetic delay in seconds provided to external node to check sync */
    } Sync; /*!< Synchronization related information */

    struct {
        int     CycleNo;        /*!< Cycle number of external ROS Node (only for information) */

        /* For debugging */
        int     CycleLastOut;   /*!< Cycle number when Topic was published */
        int     CycleLastIn;    /*!< Cycle number when Topic from external ROS Node was received */
        int     CycleLastFlush; /*!< Cycle number when data from external ROS Node was provided to model */
    } Model; /*!< Model related information. ROS side! */

    struct {
        struct {
            tRosIF_TpcSub<hellocm_msgs::Ext2CM> Ext2CM; /* For this example also used for Synchronization */

        } Sub; /*!< Topics to be subscribed */

        struct {
            tRosIF_TpcPub<hellocm_msgs::CM2Ext> CM2Ext;

            /*!< CarMaker can be working as ROS Time Server providing simulation time
             *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;

        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
        /*!< Initialization/Preparation of external ROS Node e.g. when simulation starts */
        tRosIF_Srv<hellocm_msgs::Init>    Init;
        tRosIF_Srv<cmrosutils::CMRemote>  CMRemote; // Trial
    } Services; /*!< ROS Services used by this Node (client and server)*/

    struct {
        int               QueuePub;     /*!< Queue size for Publishers */
        int               QueueSub;     /*!< Queue size for Subscribers */
        int               nCyclesClock; /*!< Number of cycles publishing /clock topic.
                                             CycleTime should be multiple of this value */
        tCMNode_Mode      Mode;
        tCMNode_SyncMode  SyncMode;
        double            SyncTimeMax;  /* Maximum Synchronization time */

        tRosIF_Cfg        Ros;
    } Cfg; /*!< General configuration for this Node */

} CMNode;


/* ------------------------ custom global variable -> start ------------------------ */

#define CarParWriteCycle 10

// uint gCM_CycleCounter;
// uint gCM_Seq;

/* ------------------------ custom global variable -> end   ------------------------ */ 

/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
static void
cmnode_HelloCM_CB_TpcIn (const hellocm_msgs::Ext2CM::ConstPtr &msg)
{
    /* Process message only if receive is expected */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	    return;
    
    int rv;
    auto in = &CMNode.Topics.Sub.Ext2CM;

    /* ------------------- Assign msg to global variables -> start -------------------- */

    in->Msg.ControlCmd               = msg->ControlCmd;
    in->Msg.TargetSpd_mps            = msg->TargetSpd_mps;
    
    /* ------------------- Assign msg to global variables -> end   -------------------- */


    /* Update receive buffer
     * - No lock for spinOnce necessary?
     */
    in->Msg.header  = msg->header;
    in->Msg.time    = msg->time;
    in->Msg.cycleno = msg->cycleno;

    /* Stopping simulation is only necessary when synchronization is activated */
    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success) {
	    LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;

    Log("%s (CMSimTime=%.3fs): External Node is in cycle %lu, Time=%.3fs, Stamp=%.3fs, SeqID=%d",
	    ros::this_node::getName().c_str(), SimCore.Time,
	    in->Msg.cycleno, msg->time.toSec(), in->Msg.header.stamp.toSec(), in->Msg.header.seq);

}



/*!
 * Description:
 * - Exemplary Service Callback for CarMaker Remote using ROS
 * - e.g. via rqt Service Caller or terminal "rosservice call ..."
 *
 *
 */
static bool
cmnode_HelloCM_CB_SrvCMRemote(cmrosutils::CMRemote::Request &req,
	cmrosutils::CMRemote::Response &resp)
{

    int rv = -2;
    char sbuf[512];

    LOG("%s: Service '%s' was triggered with type='%s', msg='%s', data='%s'",
	    ros::this_node::getName().c_str(),
	    CMNode.Services.CMRemote.Srv.getService().c_str(),
	    req.type.c_str(), req.msg.c_str(), req.data.c_str());


    /* Commands to CarMaker GUI
     * - Tcl commands!
     * - More information see "ProgrammersGuide"
     */
    if (strcasecmp("guicmd", req.type.c_str()) == 0) {
	    /* Default: Evaluate command sent with message */
        if (strcasecmp("eval", req.msg.c_str()) == 0) {
            /* e.g. data = 'LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim' */
            rv = GuiCmd_Eval(req.data.c_str());
        } else {
            if (strcasecmp("start", req.msg.c_str()) == 0) {
                if (req.data.length() == 0)
                    rv = GuiCmd_Eval("LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim");
                else {
                    sprintf(sbuf, "%s; StartSim", req.data.c_str());
                    rv = GuiCmd_Eval(sbuf);
                }
            }
            if (strcasecmp("stop", req.msg.c_str()) == 0)
            rv = GuiCmd_Eval("StopSim");
        }

	/* Commands directly to CarMaker Executable
	 * - Warning:
	 *   - Information normally provided by CarMaker GUI might be missing
	 */
    } else if (strcasecmp("cmd", req.type.c_str()) == 0) {
        if (strcasecmp("start", req.msg.c_str()) == 0) {
            if (req.data.length() == 0) {
                /* Most strings are normally provided by CarMaker GUI */
                SimStart(NULL, ros::this_node::getName().c_str(),
                    "CMRosIF/AdaptiveCruiseControl", NULL, NULL);
            } else {
                /* Most strings are normally provided by CarMaker GUI */
                SimStart(NULL, ros::this_node::getName().c_str(),
                    req.data.c_str(), NULL, NULL);
            }
        }
        if (strcasecmp("stop", req.msg.c_str()) == 0)
            SimStop2(0);
        rv = 0;
    }

    resp.res = rv;
    return true;
}



/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Get versions from shared library
 * - Set the returned Version to 0 if there is no dependency!
 * - Compatibility check should be done by calling procedure
 *   as early as possible(e.g. before CMRosIF_CMNode_Init()
 *
 * Arguments:
 * - CMRosIFVer = CMRosIF shared library version (User defined)
 *                - Initially filled with version of CMRosIF management library
 * - CMNumVer   = CarMaker version used for shared library at compile time (normally CM_NUMVER)
 *                - Initially filled with version of CMRosIF management library
 * - RosVersion = ROS version used for shared library at compile time (normally ROS_VERSION)
 *                - Initially filled with version requested by CMRosIF management library (0 if no request)
 *
 */
int
CMRosIF_CMNode_GetVersion (unsigned long *CMRosIFCMNodeNumVer,
                           unsigned long *CMNumVer,
			               unsigned long *RosNumVer)
{

    *CMRosIFCMNodeNumVer = CMNODE_NUMVER;
    *CMNumVer            = CM_NUMVER;
    *RosNumVer           = ROS_VERSION;

    return 0;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Basic Initialization
 * - e.g. create ROS Node, subscriptions, ...
 * - Return values
 *   - "rv <  0" = Error at initialization, CarMaker executable will stop
 *   - "rv >= 0" = Everything OK, CarMaker executable will continue
 *
 * Arguments:
 * - Argc/Argv  = Arguments normally provided to ROS executable are not be provided
 *                to CM executable directly, but can be set in Infofile for CMRosIF
 *                with key "Node.Args" in "Data/Config/CMRosIFParameters"
 *
 * - CMNodeName = Default CarMaker Node name
 *                - Can be parameterized in Infofile for CMRosIF
 *                - Final node name might be different (argument remapping, ...)
 *
 * - Inf        = Handle to CarMaker Infofile with parameters for this interface
 *                - Please note that pointer may change, e.g. before TestRun begins
 *
 * ToDo:
 * - Possible to create/initialize node/... before each TestRun start instead of CM startup?
 * - New Param_Get() function to read parameters from Infofile
 */
int
CMRosIF_CMNode_Init (int Argc, char **Argv, char *CMNodeName, struct tInfos *Inf)
{

    int rv;
    bool rvb                = false;
    char sbuf[512]          = "";
    char keybuf[256]        = "";
    ros::NodeHandlePtr node = NULL;
    ros::V_string names;


    LOG("Initialize CarMaker ROS Node");
    LOG("  -> Node Version = %05d", CMNODE_NUMVER);
    LOG("  -> ROS Version  = %05d", ROS_VERSION);
    LOG("  -> CM Version   = %05d", CM_NUMVER);

    /* ROS initialization. Name of Node might be different after remapping! */
    if (ros::isInitialized() == false) {
        /* "Remapping arguments" functionality (launchfiles, ...)? */
        ros::init(Argc, Argv, CMNodeName);
    } else {
	    //node.reset(); ToDo!
    }

    if (ros::master::check() == false) {
        LogErrF(EC_Init, "Can't contact ROS Master!\n Start roscore or run launch file e.g. via Extras->CMRosIF\n");
        ros::shutdown();
        return -1;
    }

    /* Node specific */
    CMNode.Cfg.Ros.Node = ros::NodeHandlePtr(boost::make_shared<ros::NodeHandle>());
    node                = CMNode.Cfg.Ros.Node;

    /* Publish specific */
    CMNode.Cfg.QueuePub  = iGetIntOpt(Inf, "Node.QueuePub", 1000); /* ToDo: Influence of queue length relevant? */

    /* Prepare the node to provide simulation time. CarMaker will be /clock server */
    strcpy(sbuf, "/use_sim_time");

    if ((rv = node->hasParam(sbuf)) == true) {
        node->getParam(sbuf, rvb);
        LOG("  -> Has param '%s' with value '%d'", sbuf, rvb);
    }

    /* Additional switch to provide simulation Time */
    strcpy(keybuf, "Node.UseSimTime");

    if ((rv = iGetIntOpt(Inf, keybuf, 1)) > 0) {
        /* Parameter must be set before other nodes start
        * - set parameter outside to be independent from execution order?
        */
        LOG("  -> Provide simulation time!");
        node->setParam("/use_sim_time", true); /* enable parameter if not already done */

        CMNode.Cfg.nCyclesClock  = iGetIntOpt(Inf, "Node.nCyclesClock", CarParWriteCycle);      // lei: 1000-10

        strcpy(sbuf, "/clock");
        LOG("    -> Publish '%s' every %dms", sbuf, CMNode.Cfg.nCyclesClock);
        CMNode.Topics.Pub.Clock.Pub  = node->advertise<rosgraph_msgs::Clock>(sbuf, CMNode.Cfg.QueuePub);


        /* ToDo: Necessary/Possible to ensure /clock is zeroed? */
        CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
        CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    } else {
        LOG("  -> Don't provide simulation time!");
        CMNode.Cfg.nCyclesClock  = 0;
    }

    strcpy(sbuf, hellocm::tpc_in_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.CM2Ext.Pub         = node->advertise<hellocm_msgs::CM2Ext>(sbuf, CMNode.Cfg.QueuePub);
    CMNode.Topics.Pub.CM2Ext.Job         = CMCRJob_Create("CM2Ext");
    CMNode.Topics.Pub.CM2Ext.CycleTime   = CarParWriteCycle;          // lei: the cycle of control/localition/planning is 20ms/10ms/100ms
    CMNode.Topics.Pub.CM2Ext.CycleOffset = 0;


    /* Subscribe specific */
    CMNode.Cfg.QueueSub  = iGetIntOpt(Inf, "Node.QueueSub", 10); /* ToDo: Effect of queue length for subscriber? */      // lei: 1->10

    strcpy(sbuf, hellocm::tpc_out_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Subscribe '%s'", sbuf);
    CMNode.Topics.Sub.Ext2CM.Sub         = node->subscribe(sbuf, CMNode.Cfg.QueueSub, cmnode_HelloCM_CB_TpcIn);
    CMNode.Topics.Sub.Ext2CM.Job         = CMCRJob_Create("Ext2CM_for_Sync");

    /* In this example cycle time might be updated with value of external ROS Node
     * - See CMRosIF_CMNode_TestRun_Start_atBegin() */
    CMNode.Topics.Sub.Ext2CM.CycleTime   = 15000;

    /* Services */
    strcpy(sbuf, hellocm::srv_init_name.c_str());
    LOG("  -> Service Client '%s'", sbuf);
    CMNode.Services.Init.Clnt = node->serviceClient<hellocm_msgs::Init>(sbuf);


    strcpy(sbuf, "CMRemote");
    LOG("  -> Create Service '%s'", sbuf);
    CMNode.Services.CMRemote.Srv = node->advertiseService(
	    sbuf, cmnode_HelloCM_CB_SrvCMRemote);


    /* Print general information after everything is done */
    LOG("Initialization of ROS Node finished!");
    LOG("  -> Node Name = '%s'", ros::this_node::getName().c_str());
    LOG("  -> Namespace = '%s'", ros::this_node::getNamespace().c_str());


    /* Advertised Topics */
    ros::this_node::getAdvertisedTopics(names);
    LOG("  -> Advertised Topics (%lu)", names.size());

    auto it = names.begin();
    for (; it != names.end(); ++it)
	    LOG("    -> %s", (*it).c_str());


    /* Subscribed Topics */
    names.clear();
    ros::this_node::getSubscribedTopics(names);
    LOG("  -> Subscribed Topics (%lu)", names.size());
    it = names.begin();
    for (; it != names.end(); ++it)
	    LOG("    -> %s",  (*it).c_str());

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Add user specific Quantities for data storage
 *   and visualization to DataDictionary
 * - Called once at program start
 * - no realtime conditions
 *
 */
void
CMRosIF_CMNode_DeclQuants (void)
{

    tDDefault *df = DDefaultCreate("CMRosIF.");

    DDefULong   (df, "CycleNoRel",         "ms", &CMNode.CycleNoRel,               DVA_None);
    DDefInt     (df, "Sync.Cycles",        "-",  &CMNode.Sync.nCycles,             DVA_None);
    DDefDouble  (df, "Sync.Time",          "s",  &CMNode.Sync.Duration,            DVA_None);
    DDefInt     (df, "Sync.CyclePrepDone", "-",  &CMNode.Sync.CyclePrepDone,       DVA_None);
    DDefInt     (df, "Sync.CycleJobDone" , "-",  &CMNode.Sync.CycleJobDone,        DVA_None);
    DDefDouble4 (df, "Sync.SynthDelay",     "s", &CMNode.Sync.SynthDelay,          DVA_IO_In);

    DDefUChar   (df, "Cfg.Mode",           "-",  (unsigned char*)&CMNode.Cfg.Mode, DVA_None);
    DDefInt     (df, "Cfg.nCyclesClock",   "ms", &CMNode.Cfg.nCyclesClock,         DVA_None);
    DDefChar    (df, "Cfg.SyncMode",       "-",  (char*)&CMNode.Cfg.SyncMode,      DVA_None);
    DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",  &CMNode.Cfg.SyncTimeMax,          DVA_IO_In);

    DDefInt     (df, "Mdl.CycleNo",        "-",  &CMNode.Model.CycleNo,            DVA_None);
    DDefInt     (df, "Mdl.CycleLastOut",   "ms", &CMNode.Model.CycleLastOut,       DVA_None);
    DDefInt     (df, "Mdl.CycleLastIn",    "ms", &CMNode.Model.CycleLastIn,        DVA_None);
    DDefInt     (df, "Mdl.CycleLastFlush", "ms", &CMNode.Model.CycleLastFlush,     DVA_None);

    DDefaultDelete(df);


    /* ------------------------ Initialize custom DAV -> start ------------------------ */ 
    
    // tDDefault *Mydf = DDefaultCreate("MyDAV.");
    // DDefInt     (Mydf, "MsgLightBrake", "-",  (int*)&CMNode.Topics.Sub.Ext2CM.Msg.LightsIndL,   DVA_None);
    // DDefaultDelete(Mydf);
    
    /* ------------------------ Initialize custom DAV -> end   ------------------------ */ 
}


/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when starting a new TestRun
 * - In separate Thread (no realtime conditions)
 * - After standard Infofiles are read in
 * - Return values
 *   - "rv <  0" = Error, TestRun start will be aborted
 *   - "rv >= 0" = Everything OK
 *
 * Arguments:
 * - Inf = CarMaker Infofile for CMRosIF with content after TestRun start
 *         - Please note that the Infofile provided at initialization might have been updated!
 *
 * ToDo:
 * - New Param_Get() function to read parameters from Infofile
 *
 */
int
CMRosIF_CMNode_TestRun_Start_atBegin (struct tInfos *Inf)
{

    /* Node can be disabled via Infofile */
    tCMNode_Mode     *pmode     = &CMNode.Cfg.Mode;
    tCMNode_SyncMode *psyncmode = &CMNode.Cfg.SyncMode;

    if (Inf != NULL) {
    	*pmode     =     (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode",      CMNode_Mode_Disabled);
    	*psyncmode = (tCMNode_SyncMode)iGetIntOpt(Inf, "Node.Sync.Mode", CMNode_SyncMode_Disabled);
    }

    if (SimCore.CycleNo == 0 || Inf == NULL || *pmode == CMNode_Mode_Disabled) {
        *pmode = CMNode_Mode_Disabled;
        LOG("CarMaker ROS Node is disabled!");
        return 0;
    }

    char sbuf[512];
    char key[256];
    char *str        = NULL;
    int rv           = 0;
    bool rvb         = false;

    int cycletime    = 0;
    int *pcycletime  = NULL;
    int cycleoff     = 0;
    tCMCRJob *job    = NULL;
    auto srv         = &CMNode.Services.Init;

    LOG("CarMaker ROS Node is enabled! Mode=%d, SyncMode=%d", *pmode, *psyncmode);
    LOG("  -> Node Name = %s", ros::this_node::getName().c_str());


    /* Update synchronization */
    if (*psyncmode != CMNode_SyncMode_Disabled && *psyncmode != CMNode_SyncMode_Tpc) {
        LogErrF(EC_Sim, "CMNode: Invalid synchronization mode '%d'!",*psyncmode);
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    CMNode.Cfg.SyncTimeMax = iGetDblOpt(Inf, "Node.Sync.TimeMax", 1.0);


    /* Reset for next cycle */
    CMNode.CycleNoRel           =  0;
    CMNode.Sync.Duration        =  0.0;
    CMNode.Sync.nCycles         = -1;
    CMNode.Sync.CycleJobDone    = -1;
    CMNode.Sync.CyclePrepDone   = -1;
    CMNode.Model.CycleNo        = -1;
    CMNode.Model.CycleLastIn    = -1;
    CMNode.Model.CycleLastOut   = -1;
    CMNode.Model.CycleLastFlush = -1;

    /* ------------------------ Initialize custom variables -> start ------------------------ */   
    
    // gCM_CycleCounter = 0;
    // gCM_Seq          = 0;
    memset(&CMNode.Topics.Sub.Ext2CM.Msg.ControlCmd, 0, sizeof(CMNode.Topics.Sub.Ext2CM.Msg.ControlCmd));

    /* ------------------------ Initialize custom variables -> end   ------------------------ */


    /* Allow an update of the clock only if it was enabled before! */
    if (CMNode.Cfg.nCyclesClock > 0) {
        if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
            CMNode.Cfg.nCyclesClock = rv;
    }

    /* Necessary to ensure /clock is zeroed here?
    * ToDo: Create function? */
    if (CMNode.Cfg.nCyclesClock > 0) {
        LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
        CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
        CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }


    /* Prepare external node for next simulation */
    if (!srv->Clnt.exists()) {
        // ToDo: possible to get update if external ROS Node name changes?
        LogErrF(EC_Sim, "ROS Service is not ready! Please start external ROS Node providing Service '%s'!",
            srv->Clnt.getService().c_str());
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    LOG("  -> Send Service Request");

    /* ToDo: Async?*/
    if (!srv->Clnt.call(srv->Msg)) {
        LogErrF(EC_Sim, "ROS Service error!");
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    /* Update cycle time with information of external node */
#if 1
    /* Variant 1:
     * - Receiving parameters via ROS Parameter Server
     * - Parameter may be set externally e.g. by other node or arguments to command
     * - ROS parameters are more flexible than ROS services!
     */
    strcpy(sbuf, hellocm::prm_cycletime_name.c_str());
    if ((rv = CMNode.Cfg.Ros.Node->hasParam(sbuf)) == true)
	    CMNode.Cfg.Ros.Node->getParam(sbuf, rv);
#else
    /* Variant 2:
     * - Receiving parameters from external Node via Service
     * - Services might be too "static"
     * - Not recommended!
     */
    rv = srv->Msg.response.cycletime;
#endif

    pcycletime = &CMNode.Topics.Sub.Ext2CM.CycleTime;

    if (*pcycletime != rv) {
        LOG("  -> Cycle time of external node changed from %dms to %dms", *pcycletime, rv);
        *pcycletime = rv;
    }


    /* Plausibility check for Cycle Time */
    if (CMNode.Cfg.nCyclesClock > 0 && (*pcycletime < CMNode.Cfg.nCyclesClock
	    || *pcycletime%CMNode.Cfg.nCyclesClock != 0)) {
	    
        LogErrF(EC_Sim, "Ext. ROS Node has invalid cycle time! Expected multiple of %dms but got %dms",
            CMNode.Cfg.nCyclesClock, *pcycletime);
            
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }

    
    
    /* Prepare Jobs for publish and subscribe
     * - Special use case:
     *   - Topic in and Topic out use same cycle time with relative shift!
     */

    /* Start to publish when simulation starts */
    job       = CMNode.Topics.Pub.CM2Ext.Job;
    cycletime = CMNode.Topics.Pub.CM2Ext.CycleTime;
    cycleoff  = CMNode.Topics.Pub.CM2Ext.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
    

    /* Synchronization with external node
     * - external node provides cycle time (see service above)
     * - other parameterization methods (e.g. ROS parameter, ...) are possible!
     * - Expect sync Topic are delayed (communication time, ...)
     * - This example shows sync via ROS Timer in external node
     *   - Therefore "/clock" topic needs to be published by CarMaker!
     *   - Other mechanism, e.g. data triggered on external node side
     *     via publishing Topic directly inside subscription callback is also possible!
     * - time=0.0 can't be detected by external node, therefore
     *   first receive needs to start after expected cycle time
     *   of external ROS node
     */

    job       = CMNode.Topics.Sub.Ext2CM.Job;
    cycletime = CMNode.Topics.Sub.Ext2CM.CycleTime;
    cycleoff  = CMNode.Topics.Sub.Ext2CM.CycleOffset = 0; /* No offset allowed if ROS Timer is used for sync!*/


    /* Create the synchronization jobs */
    if (*psyncmode == CMNode_SyncMode_Tpc) {
        CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Ext);

        LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
            CMNode.Topics.Sub.Ext2CM.Sub.getTopic().c_str(), cycletime, cycleoff);

    } else
	    CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Default);


    LOG("External ROS Node is ready to simulate");

    return 1;
}



/*!
 * ToDo:
 * - Put everything to TestRun_Start_atBegin?
 *
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Repeating call for several CarMaker cycles until return value is 1
 * - May be called even previous return value was 1
 * - See "User.c:User_TestRun_RampUp()"
 *
 */
int
CMRosIF_CMNode_TestRun_RampUp (void)
{

    /* Return immediately if node is disabled */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
        return 1;

        /* Put your code here */
        //if (NotReady) return 0;


    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when TestRun ends (no realtime conditions)
 * - See "User.c:User_TestRun_End()"
 *
 */
int
CMRosIF_CMNode_TestRun_End (void)
{

    /* Put your code here */


    /* Disable after simulation has finished */
    CMNode.Cfg.Mode = CMNode_Mode_Disabled;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called at very beginning of CarMaker cycle
 * - Process all topics/services using different modes or synchronization modes
 * - See "User.c:User_In()"
 *
 * ToDo:
 * - Additional spin mechanism
 *   - e.g. for HIL
 *   - e.g. spinning in new thread, copying incoming data here, ...
 *
 */
int
CMRosIF_CMNode_In (void)
{

    int rv                   = 0;
    int rx_done              = 0;
    const char *job_name     = NULL;
    tCMCRJob *job            = NULL;
    ros::WallTime tStart     = ros::WallTime::now();
    ros::WallDuration tDelta = ros::WallDuration(0.0);
    CMNode.Sync.nCycles      = 0;
    CMNode.Sync.Duration     = 0.0;


    // gCM_CycleCounter++;      // lei: increments by 1 every cycle
    // gCM_Seq++;
    // Log("gCM_CycleCounter = %d\n", gCM_CycleCounter); 

    switch (CMNode.Cfg.Mode) {
	case CMNode_Mode_Disabled:
	    /* Comment next line if no messages/services
	     * shall be processed in disabled Node state
	     */
	    ros::spinOnce();
	    break;

	case CMNode_Mode_Default:

	    if (CMNode.Cfg.SyncMode != CMNode_SyncMode_Tpc) {
            /* Process messages in queue, but do not block */
		    ros::spinOnce();

	    } else {
            /* Synchronization based on expected Topics
            * - Blocking call (process publish and wait for answer)
            * - Stop simulation if maximum time is exceeded
            */
            do {
                ros::spinOnce();

                /* Only do anything if simulation is running */
                if (SimCore.State != SCState_Simulate) {
                    rx_done = 1;
                    break;
                }

                rx_done = 0;

                /* Check all jobs if preparation is done */
                job = CMNode.Topics.Sub.Ext2CM.Job;

                if ((rv = CMCRJob_DoPrep(job, CMNode.CycleNoRel, 0, NULL, NULL)) < CMCRJob_RV_OK) {
                    LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(job), CMCRJob_RVStr(rv));
                    rx_done = 0;
                    break;
                }

                /* If job is not done, remember name and prevent loop to finish */
                job_name = (rv != CMCRJob_RV_DoSomething ? NULL : CMCRJob_GetName(job));
                rx_done  = rv == CMCRJob_RV_DoNothing ? 1 : 0;

                if (rx_done == 1)
                    break;

                /* Wait a little that data can arrive. WallTime, NOT ROS time!!!*/
                ros::WallDuration(0.0).sleep();
                tDelta = ros::WallTime::now() - tStart;
                CMNode.Sync.nCycles++;

            } while (ros::ok() && rx_done == 0 && tDelta.toSec() < CMNode.Cfg.SyncTimeMax);

            /* Final calculation to get duration including last cycle before receive */
            tDelta = ros::WallTime::now() - tStart;

            CMNode.Sync.Duration = tDelta.toSec();

            if (rx_done != 1 && CMNode.Cfg.SyncTimeMax > 0.0 && tDelta.toSec() >= CMNode.Cfg.SyncTimeMax)
                LogErrF(EC_Sim, "CMNode: Synchronization error! tDelta=%.3f, Last invalid Job='%s'\n", tDelta.toSec(), job_name);
	    }

	    break;

	case CMNode_Mode_Threaded:
	    /* ToDo
	     * - Spinning in parallel thread started before
	     * - Lock variables!
	     * - e.g. for HIL
	     */
	    break;

	default:
	    /* Invalid!!! */;
    }

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after driving maneuver calculation
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_DrivManCalc()"
 */
int
CMRosIF_CMNode_DrivMan_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
        return 0;

    /* Put your code here */

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after CMRosIF_CMNode_DrivManCalc
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_VehicleControl_Calc()"
 */
int
CMRosIF_CMNode_VehicleControl_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	    return 0;

    /* Put your code here */ 
    
    /* --------------------------- write par into UAQ -> start --------------------------- */     
    
    // if (gCM_CycleCounter < CarParWriteCycle)
    //     return 1;
    // else
    //     gCM_CycleCounter = 0;

    // Log("Write par into carmaker: CMSimTime=%0.4f, WallTime=%0.3fs, LightsIndL=%d\n", 
    //     SimCore.Time, ros::WallTime::now().toSec(), CMNode.Topics.Sub.Ext2CM.Msg.LightsIndL);

    int  r;
    auto in = &CMNode.Topics.Sub.Ext2CM.Msg;


    if (CMNode.Topics.Sub.Ext2CM.Msg.ControlCmd.ControlCmdEnable == 1)
    {
        // write command: brake pedal
        // notice: The braking type of the control module is deceleration, which needs to be converted
        //         into pedal or wheel braking pressure
        // pedal: VC.Brake
        // wheel braking pressure: Brake.Trq_WB_FL/Brake.Trq_WB_FR/Brake.Trq_WB_RL/Brake.Trq_WB_RR

        // r = DVA_WriteRequest("VC.Brake", OWMode_Abs, CarParWriteCycle, 0, 0, in->ControlCmd.BrakePedalReqCmd, NULL);
        // if (r < 0)
        //     LOG("No DVA write to VC.Brake possible\n");


        // write command: steering wheel
        // notice: The output angle of the control module is the front wheel angle, which needs to be 
        //         converted into the steering wheel angle

        r = DVA_WriteRequest("VC.Steer.Ang", OWMode_Abs, CarParWriteCycle, 0, 0, in->ControlCmd.SteeringWhlRadReqCmd, NULL);
        if (r < 0)
            LOG("No DVA write to VC.Steer.Ang possible\n");


        // write command: gear
        // notice: in carmaker, -9->P; -1->R; 0->N; 1->D; 2->M
        // r = DVA_WriteRequest("VC.SelectorCtrl", OWMode_Abs, CarParWriteCycle, 0, 0, in->ControlCmd.GearReqCmd, NULL);
        // if (r < 0)
        //     LOG("No DVA write to VC.SelectorCtrl possible\n");


        // write command: throttle
        // notice:The output of the control module is the opening of the throttle pedal, and the driving torque or 
        //        the opening of the throttle pedal can be used in the carmaker
        // pedal: VC.Gas
        // driving torque: PT.WFL.Trq_Drive/PT.WFL.Trq_Drive/PT.WFL.Trq_Drive/PT.WFL.Trq_Drive 
        // r = DVA_WriteRequest("VC.Gas", OWMode_Abs, CarParWriteCycle, 0, 0, in->ControlCmd.ThrottlePedalReqCmd, NULL);
        // if (r < 0)
        //     LOG("No DVA write to VC.Gas possible\n");
    }
    
    /* --------------------------- write par into UAQ -> end   --------------------------- */
    
    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after vehicle model has been calculated
 * - See "User.c:User_Calc()"
 */
int
CMRosIF_CMNode_Calc (double dt)
{

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	    return 0;

    /* Put your code here
     * - Update model parameters here?
     * - Do some calculation...
     */

    /* Update model with values from external node only in specific cycle?
     * - This data handling is optionl, but necessary for deterministic behaviour
     * - if synchronization is active, incoming data remains in msg buffer until correct cycle
     */
    int rv;
    auto sync = &CMNode.Topics.Sub.Ext2CM;

    if ((rv = CMCRJob_DoJob(sync->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
	    && rv != CMCRJob_RV_DoSomething) {
	    LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
        /* Something to do in sync cycle? */
        //CMCRJob_Info(in->Job, CMNode.CycleNoRel, "CMNode: Do Something for Sync: ");

        /* Update model parameters here? */
        CMNode.Model.CycleNo = CMNode.Topics.Sub.Ext2CM.Msg.cycleno;


        /* Remember cycle for debugging */
        CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
        CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
    }

    /* Do some calculation... */

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called close to end of CarMaker cycle
 * - See "User.c:User_Out()"
 */
int
CMRosIF_CMNode_Out (void)
{

    ros::WallTime wtime = ros::WallTime::now();

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	    return 0;

    int rv;
    auto out = &CMNode.Topics.Pub.CM2Ext;
    tDDictEntry *f;


    /* ----------------- update the message about UAQ -> start ----------------- */
    
    // Test
    f = DDictGetEntry("Car.tx");
    out->Msg.CarPosX_m = (float)f->GetFunc(f->Var);

    f = DDictGetEntry("Car.ty");
    out->Msg.CarPosY_m = (float)f->GetFunc(f->Var);

    f = DDictGetEntry("Car.Yaw");
    out->Msg.CarPosA_rad = (float)f->GetFunc(f->Var);

    f = DDictGetEntry("Car.YawRate");
    out->Msg.CarYawRate_rads = (float)f->GetFunc(f->Var);

    f = DDictGetEntry("VC.Steer.Ang");
    out->Msg.CarWhlAng_rad = (float)f->GetFunc(f->Var);

    f = DDictGetEntry("VC.Steer.AngVel");
    out->Msg.CarWhlAngRate_rads = (float)f->GetFunc(f->Var);  

    f = DDictGetEntry("Car.v");
    out->Msg.CarSpd_mps = (float)f->GetFunc(f->Var);



/*
    // Control module
    out->Msg.CarInfo.CarInfoSts = 0;


    // MSDStateReport: 
    out->Msg.chassis_report.header.seq        = gCM_Seq;
    out->Msg.chassis_report.header.stamp.sec  = wtime.sec;
    out->Msg.chassis_report.header.stamp.nsec = wtime.nsec;
    out->Msg.chassis_report.header.frame_id   = "chassis_report";
    out->Msg.chassis_report.meta.timestamp_us = 0;


    // MSDStateReport: brake pedal
    // notice: need to be converted, from opending to (0, 1)
    out->Msg.chassis_report.brake_report.available                                = 1;        // 1:brake_report_data
    out->Msg.chassis_report.brake_report.brake_report_data.timestamp_us           = ros::Time(SimCore.Time).sec;
    out->Msg.chassis_report.brake_report.brake_report_data.driver_work_type.value = 2;        // 2:AUTOPILOT_APA
    out->Msg.chassis_report.brake_report.brake_report_data.brake_type.value       = 2;        // 2:DECELERATION
    out->Msg.chassis_report.brake_report.brake_report_data.override               = false;    // 0:未接管状态
    out->Msg.chassis_report.brake_report.brake_report_data.valid                  = true;     // 0:可用状态
    f = DDictGetEntry("VC.Brake");
    out->Msg.chassis_report.brake_report.brake_report_data.pedal_input = (float)f->GetFunc(f->Var);


    // MSDStateReport: acceleration
    out->Msg.chassis_report.brake_info_report.available                           = 1;
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.timestamp_us = 0;
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.abs_avtive   = false;
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.abs_fault    = false;
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.tcs_avtive   = false;
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.msr_avtive   = false;
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.stationary   = 0;      // 判断acc_standstill_type条件之一 TBD
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.reserved     = true;   // 判断acc_standstill_type条件之一 TBD
    out->Msg.chassis_report.brake_info_report.brake_info_report_data.accleration_on_wheel = (float)f->GetFunc(f->Var); // 计算accel compensation TBD


    // MSDStateReport: throttle pedal
    // notice: need to be converted, from opending to (0, 1)
    out->Msg.chassis_report.throttle_report.available                                   = 1;
    out->Msg.chassis_report.throttle_report.throttle_report_data.timestamp_us           = 0;
    out->Msg.chassis_report.throttle_report.throttle_report_data.type.value             = 2;        // 2:TORQUE
    out->Msg.chassis_report.throttle_report.throttle_report_data.driver_work_type.value = 2;        // 2:AUTOPILOT_APA
    out->Msg.chassis_report.throttle_report.throttle_report_data.override               = false;    // 0:未接管状态, 油门override时reset控制器
    out->Msg.chassis_report.throttle_report.throttle_report_data.valid                  = true;     // 1:可用状态
    f = DDictGetEntry("VC.Gas");
    out->Msg.chassis_report.throttle_report.throttle_report_data.throttle_output = (float)f->GetFunc(f->Var);   // 用于阻力估计ResistanceEstimator


    // MSDStateReport: steering wheel rad
    // notice: need to be converted
    out->Msg.chassis_report.steering_report.available                                   = 1;   
    out->Msg.chassis_report.steering_report.steering_report_data.timestamp_us           = 0;
    out->Msg.chassis_report.steering_report.steering_report_data.driver_work_type.value = 2;        // 2:AUTOPILOT_APA
    out->Msg.chassis_report.steering_report.steering_report_data.steering_type.value    = 2;        // 2:前轮转角

    out->Msg.chassis_report.steering_report.steering_report_data.steering_report        = 0;        // 转向反馈值   TBD
    out->Msg.chassis_report.steering_report.steering_report_data.override               = false;    // 0:未接管状态 TBD
    out->Msg.chassis_report.steering_report.steering_report_data.valid                  = true;     // 1:可用状态   TBD
    f = DDictGetEntry("Vhcl.Steer.Ang");
    out->Msg.chassis_report.steering_report.steering_report_data.steering_wheel_angle_report = (float)f->GetFunc(f->Var);   // 方向盘转向角反馈值


    // MSDStateReport:GearReport
    out->Msg.chassis_report.gear_report.available                               = 1;
    out->Msg.chassis_report.gear_report.gear_report_data.valid                  = 0;      // TBD
    out->Msg.chassis_report.gear_report.gear_report_data.driver_work_type.value = 2;      // 2:AUTOPILOT_APA
    out->Msg.chassis_report.gear_report.gear_report_data.current_state.value    = 0;      // NONE=0, PARK=1, REVERSE=2, NEUTRAL=3; DRIVE=4; LOW=5;
    

    // MSDStateReport:EpbReport
    out->Msg.chassis_report.epb_report.available                              = 1;
    out->Msg.chassis_report.epb_report.epb_report_data.timestamp_us           = 0;
    out->Msg.chassis_report.epb_report.epb_report_data.driver_work_type.value = 2;        // 2:AUTOPILOT_APA 
    out->Msg.chassis_report.epb_report.epb_report_data.epb_report.value       = 0;        // RELEASE=0; PULLING=1; PULL=2; RESERVED=3; parking场景赋值RELEASE
    out->Msg.chassis_report.epb_report.epb_report_data.valid                  = 0;        // 用于获取acc_standstill_type


    // MSDStateReport: wheel_report
    out->Msg.wheel_report.header.seq        = gCM_Seq;
    out->Msg.wheel_report.header.stamp.sec  = wtime.sec;
    out->Msg.wheel_report.header.stamp.nsec = wtime.nsec;
    out->Msg.wheel_report.header.frame_id   = "wheel_report";
    out->Msg.wheel_report.meta.timestamp_us = 0;


    // MSDStateReport: WheelSpeedReport
    out->Msg.wheel_report.wheel_speed_report.available                            = 1;
    out->Msg.wheel_report.wheel_speed_report.wheel_speed_report_data.timestamp_us = 0;
    out->Msg.wheel_report.wheel_speed_report.wheel_speed_report_data.valid        = 0;
    out->Msg.wheel_report.wheel_speed_report.wheel_speed_report_data.front_left   = 0;   // Angular Speed of Wheel
    out->Msg.wheel_report.wheel_speed_report.wheel_speed_report_data.front_right  = 0;   // Angular Speed of Wheel
    out->Msg.wheel_report.wheel_speed_report.wheel_speed_report_data.rear_left    = 0;   // Angular Speed of Wheel
    out->Msg.wheel_report.wheel_speed_report.wheel_speed_report_data.rear_right   = 0;   // Angular Speed of Wheel


    // MSDStateReport: WheelAngleReport
    out->Msg.wheel_report.wheel_angle_report.available                               = 1;
    out->Msg.wheel_report.wheel_angle_report.wheel_angle_report_data.timestamp_us    = 0;
    out->Msg.wheel_report.wheel_angle_report.wheel_angle_report_data.rws_state.value = 0;   // STANDARD_MODE=0;
    out->Msg.wheel_report.wheel_angle_report.wheel_angle_report_data.front_left      = 0;
    out->Msg.wheel_report.wheel_angle_report.wheel_angle_report_data.front_right     = 0;
    out->Msg.wheel_report.wheel_angle_report.wheel_angle_report_data.rear_left       = 0;
    out->Msg.wheel_report.wheel_angle_report.wheel_angle_report_data.rear_right      = 0;


    // MLAlocalization
    out->Msg.egopose.header.seq        = gCM_Seq;
    out->Msg.egopose.header.frame_id   = "egomotion";
    out->Msg.egopose.header.stamp.sec  = wtime.sec;
    out->Msg.egopose.header.stamp.nsec = wtime.nsec;
    out->Msg.egopose.meta.seq          = gCM_Seq;
    out->Msg.egopose.meta.timestamp_us = 0;

    // here is the question
    // MLAlocalization: ego car position (unit:m)
    out->Msg.egopose.position.available        = 2;                             // MLA_POSITION_LOCAL=2;
    f = DDictGetEntry("Sensor.Inertial.IN00.Pos_0.x");
    out->Msg.egopose.position.position_local.x = (float)f->GetFunc(f->Var);     // position in boot coordinate
    f = DDictGetEntry("Sensor.Inertial.IN00.Pos_0.y");
    out->Msg.egopose.position.position_local.y = (float)f->GetFunc(f->Var);     // position in boot coordinate
    f = DDictGetEntry("Sensor.Inertial.IN00.Pos_0.z");
    out->Msg.egopose.position.position_local.z = (float)f->GetFunc(f->Var);     // position in boot coordinate


    // MLAlocalization: ego car spd (unit: m/s)
    out->Msg.egopose.velocity.available         = 2;                            // MLA_VEL_LOCAL=2;
    f = DDictGetEntry("Car.vx");
    out->Msg.egopose.velocity.velocity_local.vx = (float)f->GetFunc(f->Var);    // velocity in boot coordinate
    f = DDictGetEntry("Car.vy");
    out->Msg.egopose.velocity.velocity_local.vy = (float)f->GetFunc(f->Var);    // velocity in boot coordinate
    f = DDictGetEntry("Car.vz");
    out->Msg.egopose.velocity.velocity_local.vz = (float)f->GetFunc(f->Var);    // velocity in boot coordinate

    // ok
    // MLAlocalization: angular velocity                    // need to check
    out->Msg.egopose.angular_velocity.available            = 1;                 // MLA_ANGVEL_LOCAL=1;
    out->Msg.egopose.angular_velocity.angvelocity_local.vx = 0;                 // angular velocity in car coordinate, uint:rad/s
    out->Msg.egopose.angular_velocity.angvelocity_local.vy = 0;                 // angular velocity in car coordinate, uint:rad/s
    out->Msg.egopose.angular_velocity.angvelocity_local.vz = 0;                 // angular velocity in car coordinate, uint:rad/s


    // MLAlocalization: car head direction (unit: rad)      // need to check  Car.Pitch
    out->Msg.egopose.orientation.available          = 12;                        // MLA_EULER_LOCAL=4; MLA_QUATERNION_LOCAL=8;
    f = DDictGetEntry("Car.Pitch");
    out->Msg.egopose.orientation.euler_local.pitch  = (float)f->GetFunc(f->Var); // local euler in boot coordinate, min(-pi)/max(pi)
    f = DDictGetEntry("Car.Roll");
    out->Msg.egopose.orientation.euler_local.roll   = (float)f->GetFunc(f->Var); // local euler in boot coordinate, min(-pi/2)/max(pi/2)
    f = DDictGetEntry("Car.Yaw");
    out->Msg.egopose.orientation.euler_local.yaw    = (float)f->GetFunc(f->Var); // local euler in boot coordinate, min(-pi)/max(pi)
    out->Msg.egopose.orientation.quaternion_local.w = 0;                         // local quaternion in boot coordinate
    out->Msg.egopose.orientation.quaternion_local.x = 0;                         // local quaternion in boot coordinate
    out->Msg.egopose.orientation.quaternion_local.y = 0;                         // local quaternion in boot coordinate
    out->Msg.egopose.orientation.quaternion_local.z = 0;                         // local quaternion in boot coordinate

    // ok
    // MLAlocalization: acceleration (unit: m/s^2)
    out->Msg.egopose.acceleration.available             = 2;                            // MLA_ACC_LOCAL=2;
    f = DDictGetEntry("Car.ax");
    out->Msg.egopose.acceleration.acceleration_local.ax = (float)f->GetFunc(f->Var);    // acceleration in car coordinate
    f = DDictGetEntry("Car.ax");
    out->Msg.egopose.acceleration.acceleration_local.ay = (float)f->GetFunc(f->Var);    // acceleration in car coordinate
    f = DDictGetEntry("Car.ax");
    out->Msg.egopose.acceleration.acceleration_local.az = (float)f->GetFunc(f->Var);    // acceleration in car coordinate

    // ok
    // system_manager_msgs/SysControllerRequest
    out->Msg.sys_controller_request.header.seq               = gCM_Seq;
    out->Msg.sys_controller_request.header.stamp.sec         = 0;
    out->Msg.sys_controller_request.header.stamp.nsec        = 0;
    out->Msg.sys_controller_request.header.frame_id          = "empty";
    out->Msg.sys_controller_request.cmd.value                = 0x000b0001;      // function mode 720897u
    out->Msg.sys_controller_request.info.function_mode.value = 3;               // APA = 3;


    // framework_status_msgs/ModuleControl
    out->Msg.module_control.header.seq        = gCM_Seq;
    out->Msg.module_control.header.stamp.sec  = 0;
    out->Msg.module_control.header.stamp.nsec = 0;
    out->Msg.module_control.header.frame_id   = "empty";
    out->Msg.module_control.timestamp_us      = 0;
    out->Msg.module_control.action            = 1;      // STOP=0; START=1.             
    out->Msg.module_control.module_type.value = 3;      // ROUTING=1; PLANNING=2; CONTROLLER=3; ENDPOINT=4;


    // MSDPlanning
    out->Msg.plan.header.seq             = gCM_Seq;
    out->Msg.plan.header.stamp.sec       = wtime.sec;
    out->Msg.plan.header.stamp.nsec      = wtime.nsec;
    out->Msg.plan.header.frame_id        = "map";
    out->Msg.plan.meta.timestamp_us      = 0;           // timestamp when plan result is sent
    out->Msg.plan.meta.plan_timestamp_us = 0;           // timestamp when plan result is generated


    // path
    double TempPolynomial;
    planning_msgs::PathPoint TempPathPoint;
    out->Msg.plan.trajectory.available = 14;                // VELOCITY=4; ACCELERATION=8;

    TempPolynomial                     = 0;                 // size=4输入形式为polynomial, elseif size>0输入形式为POINTS, for pilot&parking, TBD
    out->Msg.plan.trajectory.polynomial_curve.polynomial.push_back(TempPolynomial); // vec_max_size=10
    
    TempPathPoint.position_enu.x       = 0;                 // enu position of path point
    TempPathPoint.position_enu.y       = 0;                 // enu position of path point
    TempPathPoint.heading_yaw          = 0;                 // heading yaw of path point
    TempPathPoint.curvature            = 0;                 // curvature of path point
    TempPathPoint.path_follow_strength = 0;                 // path follow strength of path point
    out->Msg.plan.trajectory.path.push_back(TempPathPoint); // vec_max_size=320,


    // velocity
    planning_msgs::VelocityPoint TempVelPoint;
    out->Msg.plan.trajectory.velocity.available = 1;                        // TARGET_VALUE=1; VEL_POINTS=2; CRUISE_VELOCITY=4;
    out->Msg.plan.trajectory.velocity.target_value = 0;                     // target value of real time planning
    
    TempVelPoint.target_velocity = 0;                                       // target velocity
    TempVelPoint.relative_time   = 0;                                       // relative time from planning start
    TempVelPoint.distance = 0;                                              // relative distance from planning start
    out->Msg.plan.trajectory.velocity.vel_points.push_back(TempVelPoint);   // vec_max_size=320

    // acceleration
    planning_msgs::AccelerationPoint TempAcc;
    out->Msg.plan.trajectory.acceleration.available         = 0;            // RANGE_LIMI=1; ACC_POINT=2
    out->Msg.plan.trajectory.acceleration.range_limit.max   = 0;            // range limit of real time planning, min(0), max(10)
    out->Msg.plan.trajectory.acceleration.range_limit.min   = 0;            // range limit of real time planning, min(-10), max(0)
    TempAcc.acc  = 0;                                                       // desired acceleration of point, min(-10),max(10)
    TempAcc.jerk = 0;                                                       // TBD
    out->Msg.plan.trajectory.acceleration.acc_points.push_back(TempAcc);    // vec_max_size=320, 


    out->Msg.plan.emergency.available                                             = 0;  // GERK_FACTOR=1; STATIONRY_OBSTACLE_CAR_INFO=2;
    out->Msg.plan.emergency.stationary_obstacle_car_info.close_to_obstacle        = 0;  // whether front car is accident
    out->Msg.plan.emergency.stationary_obstacle_car_info.steering_wheel_rad_limit = 0;  // reference steering wheel when accident


    out->Msg.plan.turn_signal_command.available              = 1;   // TURN_SIGNAL_DATA=1;
    out->Msg.plan.turn_signal_command.turn_signal_data.value = 0;   // NONE=0;(No turn signal)


    out->Msg.plan.gear_command.available       = 1;     // GEAR_DATA=1;
    out->Msg.plan.gear_command.gear_data.value = 0;     // NONE=0; PARK=1; REVERSE=2; NEUTRAL=3; DRIVE=4; LOW=5;


    out->Msg.plan.plan_status.available                        = 1;     // 0 or 1? ALGORITHM_STATUS=1;(when plan algorithm status is available)
    out->Msg.plan.plan_status.algorithm_status.scene           = 0;     // PARKING=4096; APA=4; APOA=8; WAIT=16; PAUSED=8;
    out->Msg.plan.plan_status.algorithm_status.action          = 0;     // PARKING=4096; APA=4; APOA=8; WAIT=16; PAUSED=8;
    out->Msg.plan.plan_status.algorithm_status.action_status   = 0;     // PARKING=4096; APA=4; APOA=8; WAIT=16; PAUSED=8;
    out->Msg.plan.plan_status.algorithm_status.function        = 0;     // PARKING=4096; APA=4; APOA=8; WAIT=16; PAUSED=8;
    out->Msg.plan.plan_status.algorithm_status.function_status = 0;     // PARKING=4096; APA=4; APOA=8; WAIT=16; PAUSED=8;


    out->Msg.plan.comfort.available           = 1;    // MANEUVER_GEAR=1;
    out->Msg.plan.comfort.maneuver_gear.value = 0;    // SLOW=0; NORMAL=1; FAST=2; ACCIDENT=3;


    out->Msg.plan.decision.available = 1;               // LON_DECISION=1; LAT_DECISION=2;
    out->Msg.plan.decision.lon_decision.pnc_start = 0;  // whether need start
    out->Msg.plan.decision.lon_decision.pnc_stop  = 0;  // whether need start
*/
    /* ----------------- update the message about UAQ -> end   ----------------- */


    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
	    && rv != CMCRJob_RV_DoSomething) {
	    LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

        out->Msg.cycleno      = CMNode.CycleNoRel;
        out->Msg.time         = ros::Time(SimCore.Time);
        out->Msg.synthdelay   = CMNode.Sync.SynthDelay;

        /* Header stamp and frame needs to be set manually! */

        /* provide system time close to data is sent */
        // ros::WallTime wtime = ros::WallTime::now();
        out->Msg.header.stamp.sec  = wtime.sec;
        out->Msg.header.stamp.nsec = wtime.nsec;

        out->Pub.publish(out->Msg);

        /* Remember cycle for debugging */
        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }
    
    /* Publish "/clock" topic after all other other topics are published
     * - Is the order of arrival in other node identical? */
    if (CMNode.Cfg.nCyclesClock > 0 && CMNode.CycleNoRel%CMNode.Cfg.nCyclesClock == 0) {
	    CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(SimCore.Time);
	    CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        Log("MyTest02");
    }

    /* ToDo: When increase? */
    CMNode.CycleNoRel++;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called one Time when CarMaker ends
 * - See "User.c:User_End()"
 */
int
CMRosIF_CMNode_End (void)
{

    LOG("%s: End", __func__);

    if (ros::isInitialized()) {

        /* Needs to be called before program exists, otherwise
        * "boost" error due to shared library and default deconstructor */
        CMNode.Cfg.Ros.Node->shutdown();

        /* ToDo:
        * - Blocking call? Wait until shutdown has finished?
        * - Optional? */
        ros::shutdown();
    }

    return 1;
}



/*!
 * Important:
 * - NOT automatically called by CMRosIF extension
 *
 * Description:
 * - Example of user generated function
 * - Can be accessed in other sources, e.g. User.c
 * - Use "CMRosIF_GetSymbol()" to get symbol (see "lib/CMRosIF.h")
 *
 */
int
CMRosIF_CMNode_MyFunc (char *LogMsg)
{

    LOG("%s: %s",  __func__, LogMsg);
    return 1;
}

#ifdef __cplusplus
}
#endif
