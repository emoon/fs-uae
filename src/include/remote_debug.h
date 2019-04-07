 /*
  * UAE - The Un*x Amiga Emulator
  *
  * Debugger
  *
  * (c) 1995 Bernd Schmidt
  *
  * Remote debugger code (c) 2015 Daniel Collin.
  */

#ifndef UAE_REMOTE_DEBUG_H
#define UAE_REMOTE_DEBUG_H

#define REMOTE_DEBUGGER

#ifdef REMOTE_DEBUGGER

#ifdef __cplusplus
extern "C" {
#endif

//
// Set to 1 if remote debugging is enabled otherwise 0
//

extern int remote_debugging;

struct TrapContext;

//
// port allows to modify the server socket port
// time_out allows to set the time UAE will wait at startup for a connection.
// This is useful when wanting to debug things at early startup.
// If this is zero no time-out is set and if -1 no remote connection will be setup
//

void remote_debug_init (int port, int time_out);

// Main function that will be called when doing the actual debugging

void remote_debug (void);

// Main function for copper debugging

void remote_debug_copper (uaecptr addr, uae_u16 word1, uae_u16 word2, int hpos, int vpos);

// This function needs to be called at regular interval to keep the socket connection alive

void remote_debug_update (void);

void remote_debug_start_executable (struct TrapContext* ctx);
void remote_debug_end_executable (struct TrapContext* ctx);
void remote_debug_check_exception();

#ifdef __cplusplus
}
#endif

// Config options
#define OPTION_REMOTE_DEBUGGER_START_TIMER "remote_debugger"
#define OPTION_REMOTE_DEBUGGER_DEFAULT_PORT "remote_debugger_port"
#define OPTION_REMOTE_DEBUGGER_LOG "remote_debugger_log"

// Default server socket port
#define REMOTE_DEBUGGER_DEFAULT_PORT 6860

// Error codes for the remote protocol
#define ERROR_PACKET_NOT_SUPPORTED            "E09" // Packet not supported
#define ERROR_SEND_MEMORY_PARSE               "E0f" // Error during the packet parse for command send memory
#define ERROR_UNKOWN_REGISTER                 "E10" // Unknown register
#define ERROR_INVALID_FRAME_ID                "E11" // Invalid Frame Id
#define ERROR_INVALID_MEMORY_LOCATION         "E12" // Invalid memory location
#define ERROR_SET_MEMORY_PARSE                "E20" // Error during the packet parse for command set memory
#define ERROR_SET_MEMORY_PARSE_MISSING_END    "E21" // Missing end packet for a set memory message
#define ERROR_SET_MEMORY_INVALID_ADDRESS      "E22" // Address not safe for a set memory command
#define ERROR_SET_REGISTER_PARSE              "E25" // Error during the packet parse for command set register
#define ERROR_SET_REGISTER_NON_SUPPORTED      "E26" // Error during set registed - not supported register name
#define ERROR_GET_REGISTER_PARSE              "E30" // Error during the packet parse for command get register
#define ERROR_VCONT_PARSE                     "E31" // Error during the packet parse for command vcont
#define ERROR_SET_BREAKPOINT_PARSE            "E35" // Error during the packet parse for command set Breakpoint
#define ERROR_SET_BREAKPOINT_EXCP_PARSE       "E36" // Error during the packet parse for command set exception breakpoint
#define ERROR_MAX_BREAKPOINTS_REACHED         "E37" // The maximum of breakpoints have been reached
#define ERROR_UNKNOWN_BREAKPOINT_TYPE         "E38" // Unknown breakpoint type
#define ERROR_UNKNOWN_BREAKPOINT_KIND         "E39" // Unknown breakpoint kind

// Registers Indexes
// order of registers are assumed to be
// d0-d7, a0-a7, sr, pc [optional fp0-fp7, control, iar]
#define REGISTER_D0_INDEX 0 // -> 0 to 7
#define REGISTER_A0_INDEX 8  // -> 8 to 15
#define REGISTER_SR_INDEX 16
#define REGISTER_PC_INDEX 17
#define REGISTER_FP0_INDEX 18 // -> 18 to 25
#define REGISTER_CTRL_INDEX 26
#define REGISTER_IAR_INDEX 27
#define REGISTER_LAST_VALID_INDEX 27

// Other infos sent in stop packets after registers 'T'
#define REGISTER_COPPER_ADDR_INDEX 30

#endif // REMOTE_DEBUGGER

#endif // UAE_REMOTE_DEBUG

