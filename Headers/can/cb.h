
#include "PeripheralHeaderIncludes.h"
#include "cand.h"

typedef struct _DavinciVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t rev;
    uint8_t dirty;
    char branch;
} DavinciVersion;

typedef enum version_state {
    VERSION_MAJOR,
    VERSION_MINOR,
    VERSION_REV,
    VERSION_DIRTY,
    VERSION_BRANCH,
    VERSION_DONE
} DavinciVersionState;

#define VERSION_RESYNC  0xff

void CBSendStatus( void );
void CBSendEncoder( Uint16 enc );
void CBSendVoltage( float v );
void CBSendVersionV2( DavinciVersion* v );
void IFBSendVersionV2( DavinciVersion* v );
void MDBSendTorques(int16 az, int16 roll);
void MDBRequestBIT(CAND_DestinationID did);
void SendGyrosToCopter(int16 az_gyro, int16 el_gyro, int16 rl_gyro);
void SendDebug1ToAz(int16 debug_1, int16 debug_2, int16 debug_3);
