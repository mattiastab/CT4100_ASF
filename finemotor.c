/****************************************************************************************************************

FileName : finemotor.c

****************************************************************************************************************/

#include "app.h"

#if defined(SUPPORT_STAB_MOTOR)
#include "httuner.h"
#include "db.h"
#include "e2p.h"

#define FIX_FOR_DEMO
#define FIX_BY_STAB
/*********************************************************************/

static int bASFRs232On = 1;

#if defined(ASF_MAKER)
	#define ASF_PRINT(X)	do { if(bRs232On){HTDBG_EnablePrint(); printf X; HTDBG_DisablePrint();} } while(0)
	#define FM_PRINT(X)		do { if(bASFRs232On){HTDBG_EnablePrint(); printf X; HTDBG_DisablePrint();} } while(0)
#else
	#define ASF_PRINT(X)
	#define FM_PRINT(X)		do { if(bRs232On){HTDBG_EnablePrint(); printf X; HTDBG_DisablePrint();} } while(0)
#endif

//#define DEBUG_FINEMOTOR
#if defined(DEBUG_FINEMOTOR)
#define DEBUG(X) FM_PRINT(X)
#else
#define DEBUG(X)
#endif

/*********************************************************************/

#define FINEMOTOR_TASK_STACK_SIZE		MIN_STACK_SIZE
#if defined(NT78320S)
#define FINEMOTOR_TASK_PRIORITY			(MAX_APP_PRIORITY + 12)
#else
#define FINEMOTOR_TASK_PRIORITY			(MAX_APP_PRIORITY - 50)
#endif

#define MSG_QUEUE_SIZE		32
#if defined(DPC_MAKER)
#define OFFSET				4
#elif defined(DP_MAKER)
#define OFFSET				37  // 46 ->41(seems ok) -> 36 for test 
#elif defined(ASF_MAKER)
#define OFFSET				37
#else
#define OFFSET				28//15
#endif
#define AZIMUTH_TOLERANCE	40//20
#define MOVE_STEP			0.4//0.3
#define MOVE_TIMEOUT		0.8//1.7
#define LONG_TIMEOUT		68

#define MAX_BER_CHECK		6

#define FRAME_NO_REPLY_1ST		0xE0	//FRM_NO_REPLY_1ST

#if defined(SUPPORT_INTERNET_MODEM)
#define ADDR_LNBTYPE			0x10
#endif
#define ADDR_ALL				0x30	//AZ + EL
#define ADDR_AZIMUTH			0x31	//DISEQC_ADDR_PLR_POS
#define ADDR_ELEVATION			0x32	//DISEQC_ADDR_ELEV_POS
#define ELEVATION_TIMEOUT		6		// 4->6 seconds

#if defined(SUPPORT_INTERNET_MODEM)
#define CMD_DPOS_SWITCH_LNB		0x38
#endif
#define CMD_DPOS_HALT			0x60	//POSITIONER_HALT
#define CMD_DPOS_GOTO_REF		0x6B	//POSITIONER_GOTO_POSITION
#define CMD_DPOS_GOTO_XX		0x6E	//POSITIONER_GOTO_XX
#define CMD_DPOS_SAVE			0x6C

#define CMD_DPOS_DOWNLIMIT		0x68

#if defined(SUPPORT_INTERNET_MODEM)
#define CMD_LNB_INIT			0xF0
#define CMD_LNB_TV				0xF3
#define CMD_LNB_INTERNET		0xF7
#endif

enum {
	MSG_BE,
	MSG_SA,
	MSG_CH,
	MSG_LO,
	MSG_LA,
	MSG_GO,
};

enum {
	STATUS_IDLE,

	STATUS_START_GO,

#if defined(ASF_MAKER) && defined(FIX_BY_STAB)
	STATUS_BER_MONITOR,
#endif

	STATUS_MOV_EL,
	STATUS_MOV_AZ,
	STATUS_FINE_POINT,	// Fine Pointing
	STATUS_SAVE_HPOS,	// save current horizontal position

	STATUS_MOV_STEP,	// move step-by-step until BER window limit
	STATUS_MOV_BACK,	// return into (4 steps) BER window
	STATUS_BER_CHECK,	// start counting (how many steps?) the width of the BER window
	STATUS_SAVE_HPOS0,	// save current horizontal position
	STATUS_SAVE_VPOS,	// save current vertical position

	STATUS_MOV_STEP1,	// down to the lower limit of the BER window
	STATUS_MOV_BACK1,	// return into (4 steps) BER window
	STATUS_BER_CHECK1,	// start counting (how many steps?) the width of the BER window
	STATUS_SAVE_HPOS1,	// save current horizontal position
	STATUS_SAVE_VPOS1,	// save current vertical position

	STATUS_MOV_STEP2,	// down to the lower limit of the BER window
	STATUS_MOV_BACK2,	// return into (4 steps) BER window
	STATUS_BER_CHECK2,	// start counting (how many steps?) the width of the BER window
	STATUS_SAVE_HPOS2,	// save current horizontal position
	STATUS_SAVE_VPOS2,	// save current vertical position

	STATUS_END,			// end of fine pointing procedure!
};

#if defined(DEBUG_FINEMOTOR)
const static char *pStr[] = {
	"STATUS_IDLE",

	"STATUS_START_GO",

	"STATUS_MOV_EL",
	"STATUS_MOV_AZ",
	"STATUS_FINE_POINT",	// Fine Pointing
	"STATUS_SAVE_HPOS",		// save current horizontal position

	"STATUS_MOV_STEP",		// move step-by-step until BER window limit
	"STATUS_MOV_BACK",		// return into (4 steps) BER window
	"STATUS_BER_CHECK",		// start counting (how many steps?) the width of the BER window
	"STATUS_SAVE_HPOS0",	// save current horizontal position
	"STATUS_SAVE_VPOS",		// save current vertical position

	"STATUS_MOV_STEP1",		// down to the lower limit of the BER window
	"STATUS_MOV_BACK1",		// return into (4 steps) BER window
	"STATUS_BER_CHECK1",	// start counting (how many steps?) the width of the BER window
	"STATUS_SAVE_HPOS1",	// save current horizontal position
	"STATUS_SAVE_VPOS1",	// save current vertical position

	"STATUS_MOV_STEP2",		// down to the lower limit of the BER window
	"STATUS_MOV_BACK2",		// return into (4 steps) BER window
	"STATUS_BER_CHECK2",	// start counting (how many steps?) the width of the BER window
	"STATUS_SAVE_HPOS2",	// save current horizontal position
	"STATUS_SAVE_VPOS2",	// save current vertical position

	"STATUS_END",			// end of fine pointing procedure!
};
#endif

U32 uFineMotorMsgQueue;

static int nStatus;
static int nTimeout100MS = -1;
static boolean bRs232On = false;
#if defined(SUPPORT_INTERNET_MODEM)
static U8 ucLnbType;
#endif

static float change_satellite, longitudesat, longitude, latitude;
static float azimuth, _azimuth, elevation, _elevation, limit_e, limit_w, pos_h, pos_v;
static char dirStep, direction, fine_pointing, ber, ber_monitor, ber_monitor_step, ber_received, ber_checkcnt;
static int culo;
static unsigned int inc=1;
static boolean MovingStabMotor = false;

#define BER_WINDOW_SIZE 20	//(120)//(127)//(255)
U32 uTunerId = TUNER_0;

#define PI (3.1415926535)
#define delta (6378.0/42164.0)
static const U8 dec2hex[10] = {0x0, 0x2, 0x3, 0x5, 0x6, 0x8, 0xa, 0xb, 0xd, 0xe};

#define acosf(x) ((float)acos((double)(x)))
#define cosf(x) ((float)cos((double)(x)))
#define sinf(x) ((float)sin((double)(x)))
#define tanf(x) ((float)tan((double)(x)))
#define atanf(x) ((float)atan((double)(x)))

extern U8 UIAPP_MakePolDegree(U16 *usDegree);

/****************************************************************************************************************/
static float toRadians(float angdeg)
/****************************************************************************************************************/
{
	return angdeg * PI / 180.0;
}

/****************************************************************************************************************/
static float toDegrees(float angrad)
/****************************************************************************************************************/
{
	return angrad * 180.0 / PI;
}

#if defined(NT78320S)
#define round nt_round
/****************************************************************************************************************/
static int nt_round(float a)
/****************************************************************************************************************/
{
	return (int)floor(a + 0.5);
}
#endif

/****************************************************************************************************************/
void FINEMOTOR_Calculate(float satlon)
/****************************************************************************************************************/
{
	float satrad = toRadians(satlon);
	float longrad = toRadians(longitude);
	float latrad = toRadians(latitude);
	float longsatlong = longrad - satrad;
	float beta = acosf(cosf(longsatlong)*cosf(latrad));
	float azrad = 0;
	float elrad = atanf((cosf(beta)-delta)/sinf(beta));

	if(latitude == 0)
		azrad = PI/2;
	else
		azrad = PI + atanf(tanf(longsatlong)/sinf(latrad));

	/*azimuth*/
	azimuth = 180-toDegrees(azrad);
	azimuth *= 10;
	azimuth = round(azimuth);
	azimuth /= 10;

	/*elevation*/
	elevation = toDegrees(elrad) - OFFSET;
	elevation *= 10;
	elevation = round(elevation);
	elevation /= 10;
#if 0//defined(SUPPORT_INTERNET_MODEM)
	if(ucLnbType == FINELNB_INTERNET)
	{
		azimuth -= 5.0;
	}
#endif

	//limits calculation
	limit_w = azimuth - AZIMUTH_TOLERANCE;
	limit_e = azimuth + AZIMUTH_TOLERANCE;

//limit 60 azimuth
	if(azimuth > 60.0)
		azimuth = 60.0;
	else if(azimuth < -60.0)
		azimuth = -60.0;

	if(limit_e > 60.0)
		limit_e = 60.0;
	if(limit_w < -60.0)
		limit_w = -60.0;

//limit 15 bottom
#if defined(DP_MAKER) || defined(ASF_MAKER)
	if(elevation < -25.0) elevation = -25.0;
#else
	if(elevation < -15.0) elevation = -15.0;
#endif

//	DEBUG(("%s(%d) : satlon(%f), longitude(%f), latitude(%f), azimuth(%f), elevation(%f)\n", \
//		__func__, __LINE__, satlon, longitude, latitude, azimuth, elevation));
}

/****************************************************************************************************************/
void FINEMOTOR_CalculateAZ(float satlon)
/****************************************************************************************************************/
{
	float temp_longitudine = 0.0;
	float temp_latitudine = 0.0;
	float temp_satellite = 0.0;
	float calc1 = 0.0;
	float calc2 = 0.0;
	float calc3 = 0.0;

	temp_longitudine = toRadians(longitude);
	temp_latitudine = toRadians(latitude);
	temp_satellite = toRadians(satlon);

	if(temp_latitudine==0)
		_azimuth = 90.0;
	else
		_azimuth = -(180.0/PI) * atanf(tanf(temp_longitudine - temp_satellite) / sinf(temp_latitudine));

	calc1 = _azimuth * 10;
	calc2 = (int)calc1;
	calc3 = calc1 - calc2;
	if(calc3>=0.5)
		_azimuth = (calc2+1)/10;
	else
		_azimuth = calc2/10;
#if defined(SUPPORT_INTERNET_MODEM)
	if(ucLnbType == FINELNB_INTERNET)
	{
		//_azimuth -= 5.0;
		_azimuth += 8.0;
	}
#endif

//limit 60
	if(_azimuth > 60.0)
		_azimuth = 60.0;
	else if(_azimuth < -60.0)
		_azimuth = -60.0;


//	DEBUG(("%s(%d) : satlon(%f), longitude(%f), latitude(%f), _azimuth(%f)\n", \
//		__func__, __LINE__, satlon, longitude, latitude, _azimuth));
}

/****************************************************************************************************************/
void FINEMOTOR_CalculateEL(float satlon)
/****************************************************************************************************************/
{
	float temp_longitudine = 0.0;
	float temp_latitudine = 0.0;
	float temp_satellite = 0.0;
	float beta = 0.0;
	float calc1 = 0.0;
	float calc2 = 0.0;
	float calc3 = 0.0;

	temp_longitudine = toRadians(longitude);
	temp_latitudine = toRadians(latitude);
	temp_satellite = toRadians(satlon);

	beta = acosf(cosf(temp_longitudine - temp_satellite) * cosf(temp_latitudine));

	_elevation = (atanf((cosf(beta) - delta) / sinf(beta)) * (180.0/PI)) - OFFSET;

//	DEBUG(("%s(%d) : beta(%f), elrad - offset(%d) = (%f)\n", __func__, __LINE__, beta, OFFSET, _elevation));

	calc1 = _elevation * 10;
	calc2 = (int)calc1;
	calc3 = calc1 - calc2;
	if(calc3>=0.5)
		_elevation = (calc2+1)/10;
	else
		_elevation = calc2/10;

#if defined(SUPPORT_INTERNET_MODEM)
	if(ucLnbType == FINELNB_INTERNET)
	{
		_elevation -= 1.0;
	}
#endif

//limit 15 bottom
#if defined(DP_MAKER) || defined(ASF_MAKER)
	if(_elevation < -25.0) _elevation = -25.0;
#else
	if(_elevation < -15.0) _elevation = -15.0;
#endif

//	DEBUG(("%s(%d) : satlon(%f), longitude(%f), latitude(%f), _elevation(%f)\n", \
//		__func__, __LINE__, satlon, longitude, latitude, _elevation));
}

/****************************************************************************************************************/
static void MakeXX(float dfXX, U8 *pucBuf, boolean bSat)
/****************************************************************************************************************/
{
	U8 n8;
	U16 n16;
	float fractional;

//	DEBUG(("XX = (%lf) -> ", dfXX));

	if (dfXX < 0)
	{
		if(bSat)
			pucBuf[0] = 0xf0;
		else
			pucBuf[0] = 0xe0;
		dfXX = -dfXX;
	}
	else
	{
		if(bSat)
			pucBuf[0] = 0xc0;
		else
			pucBuf[0] = 0xd0;
	}

	dfXX *= 10.0;
	n16 = (U16)dfXX;
	fractional = dfXX - n16;
	if(fractional >= 0.5)
		dfXX += 1.0;

	dfXX /= 10.0;
	n8 = (U8)dfXX;
	pucBuf[0] |= n8 >> 4;
	pucBuf[1] = n8 << 4;

	n8 = (U8)((dfXX - n8) * 10.0);
	pucBuf[1] |= dec2hex[n8];
	
//	DEBUG(("n(%u) frac(%lf) bIsSat(%d) ouput(0x%02x%02x)\n", n16, fractional, bSat, pucBuf[0], pucBuf[1]));
}

/****************************************************************************************************************/
static void FINEMOTOR_SetTimeout(int nMilliSecond)
/****************************************************************************************************************/
{
	if(nMilliSecond < 0)
		nMilliSecond -= nMilliSecond;

//	DEBUG(("%s(%d) : %d\n", __func__, __LINE__, nMilliSecond));
	
	if(nMilliSecond > 0)
	{
		nTimeout100MS = (nMilliSecond + 150 /*99*/) / 100;
		TIMER_Setup(TIMEOUT_STAB_MOTOR, TICK_PERIOD, TIMEOUT_STAB_MOTOR, 0, 0, 0, TIMER_MODE_REPEAT, UIAPP_TimeoutCallback);
	}
	else
	{
		nTimeout100MS = -1;
		TIMER_Cancel(TIMEOUT_STAB_MOTOR);
	}
}

/****************************************************************************************************************/
static void SendCmd(U8 *pMsg, int nMsgSize)
/****************************************************************************************************************/
{
#if 0
	static U8 msgBackup[20];
	static int  MsgSizeBackup = 0xffff ;

	if( MsgSizeBackup != 0xffff && pMsg ==NULL && nMsgSize ==0)
	{
		HTANT_DiSEqCWrite(uTunerId, msgBackup, MsgSizeBackup);
		return ;
	}

	if( nMsgSize <20 )
	{
		if( pMsg[1] == ADDR_AZIMUTH && pMsg[2] != CMD_DPOS_HALT )
		{
			memcpy(msgBackup, pMsg, nMsgSize);
			MsgSizeBackup = nMsgSize;
		}
	}
#endif		
	HTANT_DiSEqCWrite(uTunerId, pMsg, nMsgSize);
}


/****************************************************************************************************************/
static int FINEMOTOR_DPosSendPrevCommand(void)
/****************************************************************************************************************/
{
//PRINT(("%s()!!!!!!!\n", __func__));
	SendCmd(NULL, 0);

	return 0;
}
/****************************************************************************************************************/
static int FINEMOTOR_DPosStop(U8 ucAddr)
/****************************************************************************************************************/
{
	U8 msg[4] = {FRAME_NO_REPLY_1ST, 0, CMD_DPOS_HALT, 0};

	MovingStabMotor = false ;
//PRINT(("%s()!!!!!!!\n", __func__));
	msg[1] = ucAddr;
	SendCmd(msg, 4);

	return 0;
}

/****************************************************************************************************************/
static int FINEMOTOR_DPosSave(U8 ucAddr)
/****************************************************************************************************************/
{
	U8 msg[5] = {FRAME_NO_REPLY_1ST, 0, CMD_DPOS_SAVE, 0xA0, 0};
//PRINT(("%s()!!!!!!!\n", __func__));
	msg[1] = ucAddr;
	SendCmd(msg, 5);

	return 0;
}

/****************************************************************************************************************/
static int FINEMOTOR_GotoReference(U8 ucAddr)
/****************************************************************************************************************/
{
	U8 msg[4] = {FRAME_NO_REPLY_1ST, 0, CMD_DPOS_GOTO_REF, 0x00};
//PRINT(("%s()!!!!!!!\n", __func__));
	msg[1] = ucAddr;
	SendCmd(msg, 4);

	return 0;
}

/****************************************************************************************************************/
static int FINEMOTOR_SetDownLimit(U8 ucAddr)
/****************************************************************************************************************/
{
	U8 msg[4] = {FRAME_NO_REPLY_1ST, 0, CMD_DPOS_DOWNLIMIT, 0x00};
//PRINT(("%s()!!!!!!!\n", __func__));
	msg[1] = ucAddr;
	SendCmd(msg, 4);

	return 0;
}

#if defined(SUPPORT_INTERNET_MODEM)
/****************************************************************************************************************/
static int FINEMOTOR_SwitchLNB(U8 ucType)
/****************************************************************************************************************/
{
	U8 msg[4] = {FRAME_NO_REPLY_1ST, ADDR_LNBTYPE, CMD_DPOS_SWITCH_LNB, CMD_LNB_INIT};

	if(ucType == FINELNB_TV)
	{
		DEBUG(("DISH MODE : TV\n"));
	}
	else if(ucType == FINELNB_INTERNET)
	{
		DEBUG(("DISH MODE : Internet\n"));
	}
	else
	{
		DEBUG(("DISH MODE : Unknown\n"));
	}

	if(ucType >= FINELNB_MAX) return -1;

	SendCmd(msg, 4);
	DELAY_MS(10);
	if(ucType == FINELNB_TV)	msg[3] = CMD_LNB_TV;
	else/*FINELNB_INTERNET*/	msg[3] = CMD_LNB_INTERNET;
	SendCmd(msg, 4);

	return 0;
}
#endif

/****************************************************************************************************************/
static int FINEMOTOR_GotoUSALS(U8 ucAddr, float fDegree, float fTimeout)
/****************************************************************************************************************/
{
	U8 msg[5] = {FRAME_NO_REPLY_1ST, 0, CMD_DPOS_GOTO_XX, 0x00, 0x00};

	msg[1] = ucAddr;

	//////////////////////////////
	//Udo says the commands for up and down are reversed.
#if !defined(DPC_MAKER) && !defined(DP_MAKER) && !defined(ASF_MAKER)
//#if !defined(DP_MAKER)
	if(ucAddr == ADDR_ELEVATION)
	{
		fDegree = -fDegree;
	}
#endif
	////////////////////////////

	if(fDegree < 0) direction = -1;
	else			direction = 1;

//	DEBUG(("FINEMOTOR_GotoUSALS ucAddr(0x%02X) Degree(%f)\n", ucAddr, fDegree));

	MakeXX(fDegree, &msg[3], false);

	SendCmd(msg, 5);

#if 1
	if(fTimeout >= 0.0 && fTimeout < 1.0)
		fTimeout = 1.0;
#endif

	FINEMOTOR_SetTimeout(fTimeout * 1000);

	return 0;
}

/****************************************************************************************************************/
static int FINEMOTOR_StepUSALS(U8 ucAddr, float fDegree, float fTimeout)
/****************************************************************************************************************/
{
	U8 msg[5] = {FRAME_NO_REPLY_1ST, 0, CMD_DPOS_SAVE/*0x6C*/, 0x00, 0x00};

	msg[1] = ucAddr;

	//////////////////////////////
	//Udo says the commands for up and down are reversed.
#if !defined(DPC_MAKER) && !defined(DP_MAKER) && !defined(ASF_MAKER)
//#if !defined(DP_MAKER)
	if(ucAddr == ADDR_ELEVATION)
	{
		fDegree = -fDegree;
	}
#endif
	////////////////////////////

	if(fDegree < 0)
		direction = -1;
	else
		direction = 1;

//	DEBUG(("FINEMOTOR_StepUSALS ucAddr(0x%02X) Degree(%f)\n", ucAddr, fDegree));

	MakeXX(fDegree, &msg[3], false);

	SendCmd(msg, 5);

#if 1
	if(fTimeout > 0.0 && fTimeout < 1.0)
		fTimeout = 1.0;
#endif

	FINEMOTOR_SetTimeout(fTimeout * 1000);

	return 0;
}

#if defined(ASF_MAKER)
/****************************************************************************************************************/
static int FINEMOTOR_SendStartCmd(void)
/****************************************************************************************************************/
{
	U8 msg[3] = {0xE1, 0x30, 0x62};

	SendCmd(msg, 3);
	ASF_PRINT(("MSG START > %02X %02X %02X\n", msg[0], msg[1], msg[2]));

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));
	
	return 0;
}

/****************************************************************************************************************/
//static 
int FINEMOTOR_SendResetCmd(void)
/****************************************************************************************************************/
{
	U8 msg[4] = {0xE0, 0x31, 0x6B, 0x00};

	ASF_PRINT(("MSG RESET > %02X %02X %02X %02X\n", msg[0], msg[1], msg[2], msg[3]));
	SendCmd(msg, 4);

	return 0;
}

/****************************************************************************************************************/
static void DegreeToArray(U16 usDegree, U8 *pBuf)
/****************************************************************************************************************/
{
	U8 ucHi, ucLo;

	ucHi = usDegree / 100;
	ucLo = usDegree % 100;

	pBuf[0] = (ucHi / 10) * 16 + (ucHi % 10);
	pBuf[1] = (ucLo / 10) * 16 + (ucLo % 10);
}

/****************************************************************************************************************/
static int FINEMOTOR_SendCoordiCmd(void)
/****************************************************************************************************************/
{
	U8 msg[4] = {0xE1, 0x00, 0x00, 0x00};

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));

	msg[1] = 0xFA;
	MakeXX((double)latitude, &msg[2], false);
	ASF_PRINT(("MSG LAT > %02X %02X %02X %02X\n", msg[0], msg[1], msg[2], msg[3]));
	SendCmd(msg, 4);

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));

	msg[1] = 0xFC;
	MakeXX((double)longitude, &msg[2], false);
	ASF_PRINT(("MSG LON > %02X %02X %02X %02X\n", msg[0], msg[1], msg[2], msg[3]));
	SendCmd(msg, 4);

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));

	msg[1] = 0xFE;
	MakeXX((double)change_satellite, &msg[2], true);
	ASF_PRINT(("MSG SAT > %02X %02X %02X %02X\n", msg[0], msg[1], msg[2], msg[3]));
	SendCmd(msg, 4);

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));

	return 0;
}

/****************************************************************************************************************/
static int FINEMOTOR_SendSatelliteCmd(void)
/****************************************************************************************************************/
{
	U8 msg[4] = {0xE1, 0x00, 0x00, 0x00};

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));
  
	msg[1] = 0xFE;
	MakeXX((double)change_satellite, &msg[2], true);
	ASF_PRINT(("MSG SAT > %02X %02X %02X %02X\n", msg[0], msg[1], msg[2], msg[3]));
	SendCmd(msg, 4);

	DELAY_MS(1000);
	ASF_PRINT(("wait 1000 ms\n"));
}

/****************************************************************************************************************/
static int FINEMOTOR_SendBERCmd(U8 ucValue)
/****************************************************************************************************************/
{
	U8 msg[3] = {0xE1, 0x41, 0x00};

	msg[2] = ucValue;
	SendCmd(msg, 3);
	ASF_PRINT(("MSG LOCK > %02X %02X %02X\n", msg[0], msg[1], msg[2]));

	DELAY_MS(250);
	ASF_PRINT(("wait 250 ms\n"));

	return 0;
}
#endif

#if defined(ASF_MAKER)// && defined(FIX_BY_STAB)
/****************************************************************************************************************/
static void FINEMOTOR_TimeoutCallback(void)
/****************************************************************************************************************/
{
	U16 usSat, usAZ, usEL;
	static U32 uStartTime = 0;

	if(fine_pointing == 0)
		return;

	switch(nStatus)
	{
	case STATUS_START_GO:
		FINEMOTOR_SendCoordiCmd();      // send LAT,LON,SAT commands to the motor, with a timeout of 1000 ms after each command
		FINEMOTOR_SendStartCmd();       // send START command to the motor, with a timeout of 1000 ms after command

		// now the engine has all the information to search the satellite automatically,
		// it needs only the BER STATUS in order to complete the alignment successfully.
		nStatus = STATUS_BER_MONITOR;

		uStartTime = HTOS_GetTime() / GetClocksPerSecond();
		FINEMOTOR_SetTimeout(1);
		break;

	case STATUS_BER_MONITOR:

		// I write here only the concept, in order to understanding what must be done in this step,
		// please correct the code implementing, if necessary, a suitable function for BER MONITOR.
        
		if(HTTUNER_GetLock(uTunerId) && HTTUNER_GetQuality(uTunerId) > 60)
		{
			FINEMOTOR_SendBERCmd(0xAA);

			if(((HTOS_GetTime() / GetClocksPerSecond()) - uStartTime) > 120)	//2min
				nStatus = STATUS_END;
		}
		else
		{
			FINEMOTOR_SendBERCmd(0xFF);
			uStartTime = HTOS_GetTime() / GetClocksPerSecond();
		}
		FINEMOTOR_SetTimeout(1);
		break;
      
	case STATUS_END:
		// Please save to flash the satellite on which the search was made.
		// example: for HOTBIRD 13°E --> Save the "MakeXX((double)change_satellite, &msg[2], true);"
		//                               in order to use the function: "FINEMOTOR_SendSatelliteCmd();"
		//                               to drive the motor (with the command: E1 FE C0 D0) when the receiver is turned on.
		fine_pointing = 0;
		nStatus = STATUS_IDLE;
		FINEMOTOR_SetTimeout(1);

#if defined(FIX_FOR_DEMO)
		UI_SendMsg(MSG_UI_MOTOR_EVENT, 0, 0, 0, 0);
#endif
		break;
      
	case STATUS_IDLE:
	default:
		break;
	}
}
#else
/****************************************************************************************************************/
static void FINEMOTOR_TimeoutCallback(void)
/****************************************************************************************************************/
{
	U16 usSat, usAZ, usEL;

	if(fine_pointing == 0)
		return;

//	DEBUG(("%s(%d) : nPrvStatus(%s), azimuth(%f), elevation(%f), ber.. %d \n", \
//		__func__, __LINE__, pStr[nStatus], azimuth, elevation, ber));

	switch(nStatus)
	{
	case STATUS_START_GO:
//		DEBUG(("%s(%d) : AZIMUTH_TOLERANCE(%d), MOVE_STEP(%f)\n", __func__, __LINE__, AZIMUTH_TOLERANCE, MOVE_STEP));

#if defined(DPC_MAKER) 
		FINEMOTOR_GotoUSALS(ADDR_ELEVATION, elevation, fabs(elevation) * MOVE_TIMEOUT*2);
#else
		FINEMOTOR_GotoUSALS(ADDR_ELEVATION, elevation, fabs(elevation) * MOVE_TIMEOUT);
#endif
		nStatus = STATUS_MOV_EL;
		break;

	case STATUS_MOV_EL:
		pos_v = elevation;
		if(azimuth <= 0)
			FINEMOTOR_GotoUSALS(ADDR_AZIMUTH, limit_e, fabs(azimuth + AZIMUTH_TOLERANCE) * MOVE_TIMEOUT);
		else
			FINEMOTOR_GotoUSALS(ADDR_AZIMUTH, limit_w, fabs(azimuth + AZIMUTH_TOLERANCE) * MOVE_TIMEOUT);
		nStatus = STATUS_MOV_AZ;
		ber_checkcnt = 0;
		break;

	case STATUS_MOV_AZ:
		if(azimuth <= 0)
			pos_h = limit_e;
		else
			pos_h = limit_w;

		if(ber_checkcnt >= MAX_BER_CHECK)
		{
//			DEBUG(("%s(%d) : spiral search ===> (paused)\n", __func__, __LINE__));
			UI_SendMsg(MSG_UI_MOTOR_EVENT, 1, 0, 0, 0);
			break;
		}
//		DEBUG(("%s(%d) : spiral search ===> (%d/%d)\n", __func__, __LINE__, ber_checkcnt + 1, MAX_BER_CHECK));

		ber_monitor = 1;
		switch(ber_monitor_step)
		{
		case 0:
			pos_v = elevation - culo;
			if(pos_h >= azimuth)
				FINEMOTOR_GotoUSALS(ADDR_AZIMUTH, limit_w, LONG_TIMEOUT);
			else
				FINEMOTOR_GotoUSALS(ADDR_AZIMUTH, limit_e, LONG_TIMEOUT);
			ber_monitor_step++;
			break;

		case 1:
			if(pos_h >= azimuth)
				pos_h = limit_w;
			else
				pos_h = limit_e;
			//////////////////////////////
			//Udo says increase the elevation gap of 2¡Æ intead of 1¡Æ
			if(culo <= 0)
				culo -= 2;//culo--;
			FINEMOTOR_GotoUSALS(ADDR_ELEVATION, elevation + culo, fabs(culo)*MOVE_TIMEOUT);
			culo = -culo;
			ber_monitor_step++;
			break;

		case 2:
			pos_v = elevation - culo;
			if(pos_h >= azimuth)
				FINEMOTOR_GotoUSALS(ADDR_AZIMUTH, limit_e, LONG_TIMEOUT);
			else
				FINEMOTOR_GotoUSALS(ADDR_AZIMUTH, limit_w, LONG_TIMEOUT);
			ber_monitor_step++;
			break;

		case 3:
			if(pos_h >= azimuth) pos_h = limit_w;
			else				 pos_h = limit_e;
			//////////////////////////////
			//Udo says increase the elevation gap of 2¡Æ intead of 1¡Æ
			if(culo<=0)
				culo -= 2;//culo--;
			FINEMOTOR_GotoUSALS(ADDR_ELEVATION, elevation + culo, fabs(culo)*MOVE_TIMEOUT);
			culo = -culo;
			ber_monitor_step = 0;
			ber_checkcnt++;
			break;
		}
		break;

	case STATUS_FINE_POINT:
		ber_received = 0;
		dirStep = direction;
		FINEMOTOR_SetTimeout(4 * 1000);
		nStatus = STATUS_SAVE_HPOS;
		break;

	case STATUS_SAVE_HPOS:
		FINEMOTOR_DPosSave(ADDR_AZIMUTH);
		FINEMOTOR_SetTimeout(2 * 1000);
		nStatus = STATUS_MOV_STEP;
		break;

	case STATUS_MOV_STEP:	//stati(6)
		if(ber >= 0)
		{
			if(dirStep >= 0)
				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, -MOVE_STEP * inc, 4);
			else
				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * inc, 4);
			inc++;
			ber = -1;
		}
		else
		{
			FINEMOTOR_DPosSave(ADDR_AZIMUTH);
			inc = 4;
			FINEMOTOR_SetTimeout(2 * 1000);
			nStatus = STATUS_MOV_BACK;
		}
		break;

	case STATUS_MOV_BACK:	//stati(7)
		if(dirStep >= 0)
			FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * inc, 4);
		else
			FINEMOTOR_StepUSALS(ADDR_AZIMUTH, -MOVE_STEP * inc, 4);
		inc++;
		nStatus = STATUS_BER_CHECK;
		break;

	case STATUS_BER_CHECK: //stati(8)
		if(ber >= 0)
		{
			if(dirStep >= 0)
				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * inc, 4);
			else
				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, -MOVE_STEP * inc, 4);
			inc++;
			ber = -1;
		}
		else
		{
			if(dirStep >= 0)
				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * (int)(inc/2), MOVE_TIMEOUT * (int)(inc/2));
			else
				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, -MOVE_STEP * (int)(inc/2), MOVE_TIMEOUT * (int)(inc/2));
			inc = 4;
			nStatus = STATUS_SAVE_HPOS0;
		}
		break;

	case STATUS_SAVE_HPOS0:	//stati(9)
		FINEMOTOR_DPosSave(ADDR_AZIMUTH);
		FINEMOTOR_SetTimeout(2 * 1000);
		nStatus = STATUS_SAVE_VPOS;
		break;

	case STATUS_SAVE_VPOS:	//stati(10)
		FINEMOTOR_DPosSave(ADDR_ELEVATION);
		FINEMOTOR_SetTimeout(2 * 1000);
		inc = 1;
		nStatus = STATUS_MOV_STEP1;
		break;

	case STATUS_MOV_STEP1:	//stati(11)
		if(ber >= 0)
		{
			FINEMOTOR_StepUSALS(ADDR_ELEVATION, -MOVE_STEP * inc, ELEVATION_TIMEOUT );
			inc++;
			ber = -1;
		}
		else
		{
			FINEMOTOR_DPosSave(ADDR_ELEVATION);
			FINEMOTOR_SetTimeout(2 * 1000);
			inc = 4;
			nStatus = STATUS_MOV_BACK1;
		}
		break;

	case STATUS_MOV_BACK1:	//stati(12)
		FINEMOTOR_StepUSALS(ADDR_ELEVATION, MOVE_STEP * inc, ELEVATION_TIMEOUT);
		inc++;
		nStatus = STATUS_BER_CHECK1;
		break;

	case STATUS_BER_CHECK1:	//stati(13)
		if(ber >= 0)
		{
			FINEMOTOR_StepUSALS(ADDR_ELEVATION, MOVE_STEP * inc, ELEVATION_TIMEOUT);
			inc++;
			ber = -1;
		}
		else
		{
			FINEMOTOR_StepUSALS(ADDR_ELEVATION, MOVE_STEP * (int)(inc/2), MOVE_TIMEOUT * (int)(inc/2));
			nStatus = STATUS_SAVE_HPOS1;
		}
		break;

	case STATUS_SAVE_HPOS1:	//stati(14)
		FINEMOTOR_DPosSave(ADDR_AZIMUTH);
		FINEMOTOR_SetTimeout(2 * 1000);
		nStatus = STATUS_SAVE_VPOS1;
		break;

	case STATUS_SAVE_VPOS1:	//stati(15)
		FINEMOTOR_DPosSave(ADDR_ELEVATION);
		FINEMOTOR_SetTimeout(2 * 1000);
		inc = 1;
#if defined(FIX_FOR_DEMO)
		nStatus = STATUS_END;
#else
		nStatus = STATUS_MOV_STEP2;
#endif	
		break;

	case STATUS_MOV_STEP2:	//stati(16)
		if(ber >= 0)
		{
			FINEMOTOR_StepUSALS(ADDR_AZIMUTH, -MOVE_STEP * inc, 4);
			inc++;
			ber = -1;
		}
		else
		{
			FINEMOTOR_DPosSave(ADDR_AZIMUTH);
			FINEMOTOR_SetTimeout(2 * 1000);
			inc = 4;
			nStatus = STATUS_MOV_BACK2;
		}
		break;

	case STATUS_MOV_BACK2:	//stati(17)
		FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * inc, 4);
		inc++;
		nStatus = STATUS_BER_CHECK2;
		break;

	case STATUS_BER_CHECK2:	//stati(18)
		if(ber >= 0)
		{
			FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * inc, 4);
			inc++;
			ber = -1;
		}
		else
		{
			FINEMOTOR_StepUSALS(ADDR_AZIMUTH, MOVE_STEP * (int)(inc/2), MOVE_TIMEOUT * (int)(inc/2));
			nStatus = STATUS_SAVE_HPOS2;
		}
		break;

	case STATUS_SAVE_HPOS2:	//stati(19)
		FINEMOTOR_DPosSave(ADDR_AZIMUTH);
		FINEMOTOR_SetTimeout(2 * 1000);
		nStatus = STATUS_SAVE_VPOS2;
		break;

	case STATUS_SAVE_VPOS2:	//stati(20)
		FINEMOTOR_DPosSave(ADDR_ELEVATION);
		FINEMOTOR_SetTimeout(2 * 1000);
		nStatus = STATUS_END;
		break;

	case STATUS_END:		//stati(21)
		_azimuth = azimuth;
		_elevation = elevation;
		fine_pointing = 0;
		ber_monitor = 0;
		FINEMOTOR_SetTimeout(0); //  2       * 1000);
		nStatus = STATUS_IDLE;

		if(longitudesat < 0)
			usSat = (change_satellite * 10) + 3600;
		else
			usSat = (change_satellite * 10);

		if(azimuth < 0)
			usAZ = (_azimuth * 10) + 3600;
		else
			usAZ = (_azimuth * 10);

		if(elevation < 0)
			usEL = (_elevation * 10) + 3600;
		else
			usEL = (_elevation * 10);

#if 1
		longitudesat = change_satellite;
#endif

#if defined(SUPPORT_INTERNET_MODEM)
		FINEMOTOR_SwitchLNB(ucLnbType);

		if(ucLnbType == FINELNB_TV)
#endif
		{
			E2P_WriteFineSatAZEL(usSat, usAZ, usEL);
//			DEBUG(("Save longitudesat(%f)(%d), ref_azimuth(%f)(%d), ref_elevation(%f)(%d)\n", \
//				longitudesat, usSat, azimuth, usAZ, elevation, usEL));
		}
#if defined(ASF_MAKER)
		//FINEMOTOR_SendResetCmd();
#endif

#if defined(FIX_FOR_DEMO)
		UI_SendMsg(MSG_UI_MOTOR_EVENT, 0, 0, 0, 0);
#endif
		break;

	case STATUS_IDLE:
	default:
		break;
	}
}
#endif

/****************************************************************************************************************/
static void FINEMOTOR_GetDegreeStr(U8 ucLongitude, U16 usDegree, char *pucBuf)
/****************************************************************************************************************/
{
	U8 ucPos = UIAPP_MakePolDegree(&usDegree);

	if(ucLongitude)
		sprintf(pucBuf, "%03d.%d%c", usDegree / 10, usDegree % 10, (ucPos ? 'W' : 'E'));
	else
		sprintf(pucBuf, "%03d.%d%c", usDegree / 10, usDegree % 10, (ucPos ? 'S' : 'N'));
}

/****************************************************************************************************************/
static void FINEMOTOR_Task(void)
/****************************************************************************************************************/
{
#if defined(HT_OS21) || defined(NT78320S)
	tMsg Msg, *pMsg;
#elif defined(HT_LINUX)
	tMsg *pMsg;
#endif
	HT_STATUS nError;
	char buf[64];
	U16 usLon, usLat, usSat, usAZ, usEL;

#if defined(HT_OS21) || defined(NT78320S)
	pMsg = &Msg;
#elif defined(HT_LINUX)
	pMsg = HTOS_AllocateMemory(sizeof(tMsg));
#endif
	
//	DEBUG(("%s(%d) : change_satellite(%f), longitude(%f), latitude(%f)\n", \
//		__func__, __LINE__, change_satellite, longitude, latitude));

#if 0//defined(ASF_MAKER) && defined(FIX_BY_STAB)
	FINEMOTOR_SendSatelliteCmd();
#endif

	while(true)
	{
#if defined(ASF_MAKER) && !defined(FIX_BY_STAB)
		nError = HTOS_ReceiveFreeMsg(uFineMotorMsgQueue, pMsg, sizeof(tMsg), 200);	//250ms
		if(nError)
		{
			if(nError == HT_ERROR_TIMEOUT)
			{
				//if(fine_pointing)
#if defined(FIX_FOR_DEMO)
				if(fine_pointing && nStatus <= STATUS_BER_CHECK1)
#else
				if(fine_pointing && nStatus <= STATUS_BER_CHECK2)
#endif
				{
					U8 L = HTTUNER_GetLock(uTunerId);
					int LQ = HTTUNER_GetQuality(uTunerId);
					//U32 LB = HTTUNER_GetBER(uTunerId);
					//U16 LR = 0;

					//_DVBS2_GetSYR(&LR);
					//DEBUG(("SIGNAL CHECK > L(%d), LQ(%d), LB(%u), LR(%u), UC(%lf), PE(%lf)\n", L, LQ, LB, LR, DVBS2_Get_UCE(), DVBS2_Get_PER()));
	
					//if(HTTUNER_GetLock(uTunerId))
					if(L && LQ > 60)
					{
						FINEMOTOR_SendBERCmd(0xAA);
					}
					else
					{
						FINEMOTOR_SendBERCmd(0xFF);
					}
				}
			}
			continue;
		}
#else
		nError = HTOS_ReceiveFreeMsg(uFineMotorMsgQueue, pMsg, sizeof(tMsg), HT_TIME_INFINITY);
		if(nError)
			continue;
#endif

		if(pMsg == NULL)
			continue;

		switch(pMsg->nCmd)
		{
		case MSG_BE:
			if(ber_checkcnt >= MAX_BER_CHECK)
			{
				DEBUG(("BE%06u ---> IGNORE(User popup)\n", pMsg->nParam1));
				break;
			}

			if(pMsg->nParam1 > 999999)
			{
				FM_PRINT(("BE999999\n"));
			}
			else
			{
				FM_PRINT(("BE%06u\n", pMsg->nParam1));
			}
			if(fine_pointing || ber_monitor)
			{
				DEBUG(("%s(%d) : fine_pointing(%d), ber_monitor(%d)\n", \
					__func__, __LINE__, fine_pointing, ber_monitor));
			}

			if(fine_pointing && ber_monitor)
			{
				ber = 0;

				if(nStatus == STATUS_MOV_AZ)
				{
					FINEMOTOR_DPosStop(ADDR_AZIMUTH);
					nStatus = STATUS_FINE_POINT;

					ber_received = 1;
				}

				if(nStatus == STATUS_MOV_BACK)
				{
					ber_received = 1;
				}

				if(ber_received)
				{
					FINEMOTOR_TimeoutCallback();
				}
			}
			break;

		case MSG_SA:
			FINEMOTOR_GetDegreeStr(1, pMsg->nParam1, buf);
			FM_PRINT(("SA%s\n", buf));
			if(!fine_pointing)
			{
#if 1
				if(pMsg->nParam1 > 1800)
					change_satellite = (pMsg->nParam1 - 3600);
				else
					change_satellite = pMsg->nParam1;
				change_satellite /= 10.0;
#else
				if(pMsg->nParam1 > 1800)
					longitudesat = (pMsg->nParam1 - 3600);
				else
					longitudesat = pMsg->nParam1;
				longitudesat /= 10.0;
				change_satellite = longitudesat;

				FINEMOTOR_Calculate(longitudesat);
				if(longitudesat < 0)	usSat = (longitudesat * 10) + 3600;
				else					usSat = (longitudesat * 10);
				if(azimuth < 0)			usAZ = (azimuth * 10) + 3600;
				else					usAZ = (azimuth * 10);
				if(elevation < 0)		usEL = (elevation * 10) + 3600;
				else					usEL = (elevation * 10);
				E2P_WriteFineSatAZEL(usSat, usAZ, usEL);
				DEBUG(("Save longitudesat(%f)(%d), ref_azimuth(%f)(%d), ref_elevation(%f)(%d)\n", \
					longitudesat, usSat, azimuth, usAZ, elevation, usEL));
#endif
			}
			break;

		case MSG_CH:
			FINEMOTOR_GetDegreeStr(1, pMsg->nParam1, buf);
			FM_PRINT(("CH%s\n", buf));

			if(!fine_pointing)
			{
				if(pMsg->nParam1 > 1800)
					change_satellite = (pMsg->nParam1 - 3600);
				else
					change_satellite = pMsg->nParam1;
				change_satellite /= 10.0;

#if defined(ASF_MAKER) && defined(FIX_BY_STAB)
				FINEMOTOR_SendSatelliteCmd();
#else
				FINEMOTOR_CalculateAZ(change_satellite);
				FINEMOTOR_CalculateEL(change_satellite);
				FINEMOTOR_Calculate(longitudesat);

//				DEBUG(("CH : go_azimuth(%f) ref_azimuth(%f) go_elevation(%f) ref_elevation(%f)\n", \
//					azimuth, _azimuth, elevation, _elevation));

				FINEMOTOR_StepUSALS(ADDR_AZIMUTH, _azimuth - azimuth, 1);
				FINEMOTOR_StepUSALS(ADDR_ELEVATION, _elevation - elevation, 1);
#endif
				nStatus = STATUS_IDLE;
			}
			break;

		case MSG_LO:
			FINEMOTOR_GetDegreeStr(1, pMsg->nParam1, buf);
			FM_PRINT(("LO%s\n", buf));

			if(!fine_pointing)
			{
				usLon = E2P_Read(EEPROM_ANTLON_ADDR);
				if(usLon > 1800)
					longitude = (usLon - 3600);
				else
					longitude = usLon;
				longitude /= 10.0;
			}
			break;

		case MSG_LA:
			FINEMOTOR_GetDegreeStr(0, pMsg->nParam1, buf);
			FM_PRINT(("LA%s\n",buf));

			if(!fine_pointing)
			{
				usLat = E2P_Read(EEPROM_ANTLAT_ADDR);
				if(usLat > 1800)
					latitude = (usLat - 3600);
				else
					latitude = usLat;
				latitude /= 10.0;
			}
			break;

		case MSG_GO:
			FM_PRINT(("GO\n"));

			bASFRs232On = 0;

			if(!fine_pointing)
			{
#if defined(SUPPORT_INTERNET_MODEM)
				FINEMOTOR_SwitchLNB(FINELNB_TV);
#endif
#if 1
				FINEMOTOR_Calculate(change_satellite);
#else
				FINEMOTOR_Calculate(longitudesat);
#endif
#if defined(ASF_MAKER)
				//FINEMOTOR_SendResetCmd();
	#if !defined(FIX_BY_STAB)
				FINEMOTOR_SendCoordiCmd();
				FINEMOTOR_SendStartCmd();
	#endif
#endif
				fine_pointing = 1;
				nStatus = STATUS_START_GO;
				FINEMOTOR_TimeoutCallback();
			}
			break;
		}
	}
}

/****************************************************************************************************************/
static int FINEMOTOR_SendMsg(int nCmd, int nParam1, int nParam2, int nParam3)
/****************************************************************************************************************/
{
#if defined(SUPPORT_WITHOUT_ALLOC_MSG)
	tMsg stMsg;

	if(E2P_Read(EEPROM_POWERSTATE_ADDR) == 0)
		return 1;

	stMsg.nIdx = 0;
	stMsg.nCmd = nCmd;
	stMsg.nParam1 = nParam1;
	stMsg.nParam2 = nParam2;
	stMsg.nParam3 = nParam3;
	stMsg.nParam4 = 0;
	
	return HTOS_SendMsg(uFineMotorMsgQueue, &stMsg);
#else
	tMsg* pMsg;

//	PRINT(("%s()!!!!!!! %d\n", __func__, nCmd));
	if(E2P_Read(EEPROM_POWERSTATE_ADDR) == 0)
		return 1;

	pMsg = HTOS_AllocateMsg(uFineMotorMsgQueue, HT_TIME_INFINITY);
	if (pMsg == NULL)
	{
		FM_PRINT(("FINEMOTOR> HTOS_AllocateMsg failed\n"));
		return HT_ERROR_NO_MEMORY;
	}

	pMsg->nIdx = 0;
	pMsg->nCmd = nCmd;
	pMsg->nParam1 = nParam1;
	pMsg->nParam2 = nParam2;
	pMsg->nParam3 = nParam3;
	pMsg->nParam4 = 0;
	
	return HTOS_SendMsg(uFineMotorMsgQueue, pMsg);
#endif
}

/****************************************************************************************************************/
static void FINEMOTOR_InitValue(void)
/****************************************************************************************************************/
{
	U8 ucDPosSat;
	U16 usSatLon;
	U16 usLon, usLat;
	U16 usSat, usAZ, usEL;
//	PRINT(("%s()!!!!!!!\n", __func__));
#if defined(SUPPORT_INTERNET_MODEM)
	E2P_ReadFineLNBType(&ucLnbType);
	FINEMOTOR_SwitchLNB(ucLnbType);
#endif

#if defined(SUPPORT_SERIAL_OUTPUT)
	if(E2P_Read(EEPROM_DISHOPEN_ADDR))
		bRs232On = true;
	else
#endif
		bRs232On = false;

	E2P_ReadFineSatAZEL(&usSat, &usAZ, &usEL);
	ucDPosSat = E2P_Read(EEPROM_DPOSSAT_ADDR);
	usLon = E2P_Read(EEPROM_ANTLON_ADDR);
	usLat = E2P_Read(EEPROM_ANTLAT_ADDR);
	if(usLon == 0 && usLat == 0)
	{
		usLon = 3610;	//invalid angle
		usLat = 3610;	//invalid angle
		FINEMOTOR_GotoReference(ADDR_AZIMUTH);
		FINEMOTOR_GotoReference(ADDR_ELEVATION);
	}

	if(usSat > 1800)
		longitudesat = (usSat - 3600);
	else
		longitudesat = usSat;
	longitudesat /= 10.0;

	if(usAZ > 1800)
		azimuth = (usAZ - 3600);
	else
		azimuth = usAZ;
	azimuth /= 10.0;

	if(usEL > 1800)
		elevation = (usEL - 3600);
	else
		elevation = usEL;
	elevation /= 10.0;

	if(usLon > 1800)
		longitude = (usLon - 3600);
	else
		longitude = usLon;
	longitude /= 10.0;

	if(usLat > 1800)
		latitude = (usLat - 3600);
	else
		latitude = usLat;
	latitude /= 10.0;

//	DEBUG(("Read longitudesat(%f), ref_azimuth(%f), ref_elevation(%f)\n", \
//			longitudesat, azimuth, elevation));

	if(ucDPosSat < MAX_NET)	//if the last motor position is valid
	{
		usSatLon = DB_GetNetDegree(DB_GetNet(ucDPosSat));

		if(usSatLon > 1800)
			change_satellite = (usSatLon - 3600);
		else
			change_satellite = usSatLon;
		change_satellite /= 10.0;

		FINEMOTOR_CalculateAZ(change_satellite);
		FINEMOTOR_CalculateEL(change_satellite);
		FINEMOTOR_Calculate(longitudesat);

//		DEBUG(("Read change_satellite(%f), go_azimuth(%f), go_elevation(%f)\n", \
//			change_satellite, _azimuth, _elevation));
	}
}

/****************************************************************************************************************/
int FINEMOTOR_Init(void)
/****************************************************************************************************************/
{
	FINEMOTOR_InitValue();

	if((uFineMotorMsgQueue = HTOS_CreateMsgQueue(sizeof(tMsg), MSG_QUEUE_SIZE)) == INVALID_HANDLE)
	{
		FM_PRINT(("FINEMOTOR> FATAL : fail to create FINEMOTOR MESSAGE QUEUE\n"));
		return HT_ERROR_INVALID_HANDLE;
	}

	if(HTOS_CreateTask(FINEMOTOR_Task, NULL, FINEMOTOR_TASK_STACK_SIZE, FINEMOTOR_TASK_PRIORITY, "FINEMOTOR_Task") == INVALID_HANDLE)
	{
		FM_PRINT(("FINEMOTOR> FATAL : fail to create FINEMOTOR_Task\n"));
		return HT_ERROR_INVALID_HANDLE;
	}

	return 0;
}

/****************************************************************************************************************/
void FINEMOTOR_SendBEMsg(void)
/****************************************************************************************************************/
{
	U32 uBerValue;
	int nRetry;

	nRetry = 0;

#if defined(FIX_FOR_DEMO)
	if(MANAGER_CurrentUI() != UI_MOTODISH && MANAGER_LowerUI() != UI_MOTODISH && !fine_pointing)
	{
		if(MANAGER_CurrentUI() == UI_MOTODISH)
			DEBUG(("DON`T SEND [BE] MSG!(MOTODISH MENU), fine_pointing(%d)\n", fine_pointing));
		else
			DEBUG(("DON`T SEND [BE] MSG!(MOTODISH POPUP), fine_pointing(%d)\n", fine_pointing));
		return;
	}
#endif

	while(++nRetry < 3)
	{		
		if(HTTUNER_GetLock(uTunerId))
		{
			DELAY_MS(30); // 25);

			if((uBerValue = HTTUNER_GetBER(uTunerId)) == 0)
			{
//sttbx_DirectPrint("===>>>>BER :  uBerValue OK  \n");
#if 0
				if( MovingStabMotor == true)
				{
//					FINEMOTOR_DPosStop(ADDR_AZIMUTH);

					MOTODISH_TuneAnotherTP(1);
					MovingStabMotor = false ;

//					DELAY_MS(10000);
//					FINEMOTOR_DPosSendPrevCommand();

					break ;
				}
#endif
				FINEMOTOR_SendMsg(MSG_BE, (int)uBerValue, 0, 0);
				break;
			}			
		}
// del		DELAY_MS(25);
	}
}

/****************************************************************************************************************/
void FINEMOTOR_SendSAMsg(U16 usDegree, boolean bWithLOLA)
/****************************************************************************************************************/
{
//	PRINT(("%s()!!!!!!!\n", __func__));
	FINEMOTOR_SendMsg(MSG_SA, usDegree, 0, 0);
	if(bWithLOLA)
	{
		FINEMOTOR_SendMsg(MSG_LO, E2P_Read(EEPROM_ANTLON_ADDR), 0, 0);
		FINEMOTOR_SendMsg(MSG_LA, E2P_Read(EEPROM_ANTLAT_ADDR), 0, 0);
	}
}

/****************************************************************************************************************/
void FINEMOTOR_SendCHMsg(U16 usDegree)
/****************************************************************************************************************/
{
//	PRINT(("%s()!!!!!!!\n", __func__));
	FINEMOTOR_SendMsg(MSG_CH, usDegree, 0, 0);
}

/****************************************************************************************************************/
void FINEMOTOR_SendGOMsg(void)
/****************************************************************************************************************/
{
	MovingStabMotor = true ;
//	PRINT(("%s()!!!!!!!\n", __func__));
	FINEMOTOR_SendMsg(MSG_GO, 0, 0, 0);
}

/****************************************************************************************************************/
void FINEMOTOR_FinePointingHalt(void)
/****************************************************************************************************************/
{
	FINEMOTOR_SetTimeout(0);
	ber_checkcnt = 0;
	fine_pointing = 0;
	ber_monitor = 0;
#if defined(SUPPORT_INTERNET_MODEM)
	FINEMOTOR_SwitchLNB(ucLnbType);
#endif
	nStatus = STATUS_IDLE;
//	DEBUG(("%s(%d) : spiral search ===> (stop)\n", __func__, __LINE__));
}

/****************************************************************************************************************/
void FINEMOTOR_PrintOnOff(boolean bOn)
/****************************************************************************************************************/
{
	bRs232On = true;
	if(bOn)
	{
		FM_PRINT(("ON\n"));
		FINEMOTOR_CalculateAZ(change_satellite);
		FINEMOTOR_CalculateEL(change_satellite);
		FINEMOTOR_Calculate(longitudesat);

//		DEBUG(("go to previous satellite: go_azimuth(%f) ref_azimuth(%f) go_elevation(%f) ref_elevation(%f)\n", \
//			azimuth, _azimuth, elevation, _elevation));

		FINEMOTOR_StepUSALS(ADDR_AZIMUTH, _azimuth - azimuth, 0);
		FINEMOTOR_StepUSALS(ADDR_ELEVATION, _elevation - elevation, 0);
	}
	else
	{
		FM_PRINT(("OFF\n"));

		FINEMOTOR_FinePointingHalt();
		FINEMOTOR_GotoReference(ADDR_AZIMUTH);
		FINEMOTOR_SetDownLimit(ADDR_ELEVATION);
	}
	bRs232On = bOn;
}

#if defined(SUPPORT_INTERNET_MODEM)
/****************************************************************************************************************/
U8 FINEMOTOR_GetLNBType(void)
/****************************************************************************************************************/
{
	return ucLnbType;
}

/****************************************************************************************************************/
void FINEMOTOR_SetLNBType(U8 ucType)
/****************************************************************************************************************/
{
	U16 usDegree;

	if(ucType < FINELNB_MAX)
	{
		ucLnbType = ucType;
		E2P_WriteFineLNBType(ucLnbType);
		FINEMOTOR_SwitchLNB(ucLnbType);

		if(change_satellite < 0) usDegree = (change_satellite * 10) + 3600;
		else					 usDegree = (change_satellite * 10);
		FINEMOTOR_SendMsg(MSG_CH, usDegree, 0, 0);
	}
}
#endif

/****************************************************************************************************************/
void FINEMOTOR_Timeout100MS(void)
/****************************************************************************************************************/
{
	static int nBERCheck = 0;

	if(++nBERCheck >= 10)
	{
		FINEMOTOR_SendBEMsg();
		nBERCheck = 0;
	}

	if(nTimeout100MS > 0)
	{
		nTimeout100MS--;
	}
	else if(nTimeout100MS == 0)
	{
		nTimeout100MS = -1;
		FINEMOTOR_TimeoutCallback();
	}
}

/****************************************************************************************************************/
int FINEMOTOR_IsFindPointing(void)
/****************************************************************************************************************/
{
//	PRINT(("%s()!!!!!!! %d\n", __func__, nStatus));
	if(nStatus != STATUS_IDLE)
		return 1;
	
	return 0;
}

/****************************************************************************************************************/
void FINEMOTOR_FinePointResume(void)
/****************************************************************************************************************/
{
	ber_checkcnt = 0;
	FINEMOTOR_SetTimeout(500);
//	DEBUG(("%s(%d) : spiral search ===> (resume)\n", __func__, __LINE__));
}
#endif
