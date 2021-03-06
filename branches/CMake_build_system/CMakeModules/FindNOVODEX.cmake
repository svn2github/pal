# Locate Novodex/PhysX
# This module defines
# NOVODEX_LIBRARY, NOVODEX_LIBRARY_DEBUG
#NOVODEX_FOUND, if false, do not try to link to Novodex/PhysX
# NOVODEX_INCLUDE_DIR, where to find the headers

SET(NOVODEX_HEADERS Cooking Foundation Character Physics PhysXLoader)		# Extensions
SET(NOVODEX_INCLUDE_DIR_ERROR "NO")

FOREACH(CUR_DIR ${NOVODEX_HEADERS})
	FIND_PATH(NOVODEX_${CUR_DIR}_INCLUDE_DIR
		NAMES "${CUR_DIR}.h" "Nx${CUR_DIR}.h"
		HINTS
		$ENV{NOVODEX_DIR}
		$ENV{NOVODEX_PATH}
		${ADDITIONAL_SEARCH_PATHS}
		PATH_SUFFIXES include Include "SDKs/${CUR_DIR}/include" "SDKs/${CUR_DIR}/Include" "SDKs/Nx${CUR_DIR}/include" "SDKs/Nx${CUR_DIR}/Include"
		PATHS
			~/Library/Frameworks
			/Library/Frameworks
			/usr/local
			/usr
			/sw # Fink
			/opt/local # DarwinPorts
			/opt/csw # Blastwave
			/opt
	)

	# Combine all dirs to one variable
	IF(NOVODEX_${CUR_DIR}_INCLUDE_DIR AND NOT NOVODEX_INCLUDE_DIR_ERROR)
		SET(NOVODEX_INCLUDE_DIR "${NOVODEX_INCLUDE_DIR};${NOVODEX_${CUR_DIR}_INCLUDE_DIR}")
	ELSE()
		SET(NOVODEX_INCLUDE_DIR "NOVODEX_INCLUDE_DIR-NOTFOUND")
		SET(NOVODEX_INCLUDE_DIR_ERROR "YES")
	ENDIF()
ENDFOREACH()




SET(NOVODEX_LIBS NxCharacter NxCooking PhysXLoader)		# PhysXCore NxPhysics NxFoundation

SET(NOVODEX_LIBRARY_ERROR "NO")
SET(NOVODEX_LIBRARY_DEBUG_ERROR "NO")

FOREACH(CUR_LIB ${NOVODEX_LIBS})
	STRING(TOLOWER "${CUR_LIB}" CUR_LIB_LOWER)
	FIND_LIBRARY(NOVODEX_LIBRARY_${CUR_LIB}
		NAMES ${CUR_LIB} ${CUR_LIB_LOWER}
		HINTS
			$ENV{NOVODEX_DIR}
			$ENV{NOVODEX_PATH}
			${ADDITIONAL_SEARCH_PATHS}
		PATH_SUFFIXES lib64 lib lib/win32 SDKs/lib/win32 latest
		PATHS
			~/Library/Frameworks
			/Library/Frameworks
			/usr/local
			/usr
			/sw
			/opt/local
			/opt/csw
			/opt
	)

	# FIND_LIBRARY(NOVODEX_LIBRARY_${CUR_LIB}_DEBUG
		# NAMES "${CUR_LIB}d" "${CUR_LIB_LOWER}d" "${CUR_LIB}_d" "${CUR_LIB_LOWER}_d"
		# HINTS
			# $ENV{NOVODEX_DIR}
			# $ENV{NOVODEX_PATH}
			# ${ADDITIONAL_SEARCH_PATHS}
		# PATH_SUFFIXES lib64 lib lib/win32 SDKs/lib/win32 latest
		# PATHS
			# ~/Library/Frameworks
			# /Library/Frameworks
			# /usr/local
			# /usr
			# /sw
			# /opt/local
			# /opt/csw
			# /opt
	# )

	# Combine all libs to two variables
	IF(NOVODEX_LIBRARY_${CUR_LIB} AND NOT NOVODEX_LIBRARY_ERROR)
		SET(NOVODEX_LIBRARY "${NOVODEX_LIBRARY};${NOVODEX_LIBRARY_${CUR_LIB}}")
	ELSE()
		SET(NOVODEX_LIBRARY "NOVODEX_LIBRARY-NOTFOUND")
		SET(NOVODEX_LIBRARY_ERROR "YES")
	ENDIF()

	IF(NOVODEX_LIBRARY_${CUR_LIB}_DEBUG AND NOT NOVODEX_LIBRARY_DEBUG_ERROR)
		SET(NOVODEX_LIBRARY_DEBUG "${NOVODEX_LIBRARY_DEBUG};${NOVODEX_LIBRARY_${CUR_LIB}_DEBUG}")
	ELSE()
		SET(NOVODEX_LIBRARY_DEBUG "NOVODEX_LIBRARY_DEBUG-NOTFOUND")
		SET(NOVODEX_LIBRARY_DEBUG_ERROR "YES")
	ENDIF()
ENDFOREACH()

# Special for DLL copy
IF(PAL_MODULE_COPY)
	SET(NOVODEX_LIBRARY_MODULE_ERROR "NO")
	SET(NOVODEX_LIBRARY_MODULE_DEBUG_ERROR "NO")

	FOREACH(CUR_LIB ${NOVODEX_LIBS})
		STRING(TOLOWER "${CUR_LIB}" CUR_LIB_LOWER)
		FIND_FILE(NOVODEX_LIBRARY_${CUR_LIB}_MODULE
			NAMES "${CUR_LIB}${MODULE_EXT}" "${CUR_LIB_LOWER}${MODULE_EXT}"
			HINTS
			$ENV{NOVODEX_DIR}
			$ENV{NOVODEX_PATH}
			${ADDITIONAL_SEARCH_PATHS}
			PATH_SUFFIXES bin bin/win32
			DOC "Optional path of the release DLL, to be copied after the build."
			PATHS
				~/Library/Frameworks
				/Library/Frameworks
				/usr/local
				/usr
				/sw
				/opt/local
				/opt/csw
				/opt
		)

		# FIND_FILE(NOVODEX_LIBRARY_${CUR_LIB}_MODULE_DEBUG 
			# NAMES "${CUR_LIB}d${MODULE_EXT}" "${CUR_LIB_LOWER}d${MODULE_EXT}" "${CUR_LIB}_d${MODULE_EXT}" "${CUR_LIB_LOWER}_d${MODULE_EXT}"
			# HINTS
			# $ENV{NOVODEX_DIR}
			# $ENV{NOVODEX_PATH}
			# ${ADDITIONAL_SEARCH_PATHS}
			# PATH_SUFFIXES bin bin/win32
			# DOC "Optional path of the debug DLL, to be copied after the build."
			# PATHS
				# ~/Library/Frameworks
				# /Library/Frameworks
				# /usr/local
				# /usr
				# /sw
				# /opt/local
				# /opt/csw
				# /opt
		# )

		# Combine all libs to two variables
		IF(NOVODEX_LIBRARY_${CUR_LIB}_MODULE AND NOT NOVODEX_LIBRARY_MODULE_ERROR)
			SET(NOVODEX_LIBRARY "${NOVODEX_LIBRARY};${NOVODEX_LIBRARY_${CUR_LIB}_MODULE}")
		ELSE()
			SET(NOVODEX_LIBRARY "NOVODEX_LIBRARY-NOTFOUND")
			SET(NOVODEX_LIBRARY_MODULE_ERROR "YES")
		ENDIF()

		IF(NOVODEX_LIBRARY_${CUR_LIB}_MODULE_DEBUG AND NOT NOVODEX_LIBRARY_MODULE_DEBUG_ERROR)
			SET(NOVODEX_LIBRARY_DEBUG "${NOVODEX_LIBRARY_DEBUG};${NOVODEX_LIBRARY_${CUR_LIB}_MODULE_DEBUG}")
		ELSE()
			SET(NOVODEX_LIBRARY_DEBUG "NOVODEX_LIBRARY_DEBUG-NOTFOUND")
			SET(NOVODEX_LIBRARY_MODULE_DEBUG_ERROR "YES")
		ENDIF()
	ENDFOREACH()
ENDIF()


SET(NOVODEX_FOUND "NO")
IF(NOVODEX_LIBRARY AND NOVODEX_INCLUDE_DIR)
  SET(NOVODEX_FOUND "YES")
ENDIF()

