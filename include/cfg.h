#if !defined(CFG_H)
#define CFG_H

#if defined(CLIENT)
inline char CLIENT_NAME[] = "Client1";
#endif	/* CLIENT */

inline char SERVER_NAME[] = "Server1";

#define WIRED_LAN
#define PORT 8088
#define W5500_RST_PORT 15

//#define DBG_PRT

#endif
