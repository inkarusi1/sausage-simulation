#ifndef __Version_h__
#define __Version_h__

#define STRINGIZE_HELPER(x) #x
#define STRINGIZE(x) STRINGIZE_HELPER(x)
#define WARNING(desc) message(__FILE__ "(" STRINGIZE(__LINE__) ") : Warning: " #desc)

#define GIT_SHA1 "da07dbfa18ac56ef1e9bae9b7372281866dbecce"
#define GIT_REFSPEC "refs/heads/v4-debug"
#define GIT_LOCAL_STATUS "DIRTY"

#define PBD_VERSION "2.2.2"

#ifdef DL_OUTPUT
#pragma WARNING(Local changes not committed.)
#endif

#endif
