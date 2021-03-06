# Convinience functions/macros to setup ${TRGTNAME}_LIBRARIES variable


# Sets the ${TRGTNAME}_LIBRARIES variable according to debug and release libraries given.
FUNCTION(FIND_PACKAGE_ADD_TARGET_LIBRARIES TRGTNAME RELEASE_LIB DEBUG_LIB)
	#IF(${TRGTNAME}_FOUND AND RELEASE_LIB)		# Can't test _FOUND if we use this function before the end !
	IF(RELEASE_LIB)
		IF(DEBUG_LIB)
			LIST(APPEND ${TRGTNAME}_LIBRARIES optimized "${RELEASE_LIB}" debug "${DEBUG_LIB}")
		ELSE()
			LIST(APPEND ${TRGTNAME}_LIBRARIES "${RELEASE_LIB}")		# Could add "general" keyword, but it is optional
		ENDIF()
	ENDIF()
	SET(${TRGTNAME}_LIBRARIES ${${TRGTNAME}_LIBRARIES} PARENT_SCOPE)
ENDFUNCTION()

# Convinience macro for FIND_PACKAGE_ADD_TARGET_LIBRARIES that uses ${TRGTNAME}_LIBRARY and ${TRGTNAME}_LIBRARY_DEBUG variables.
FUNCTION(FIND_PACKAGE_SET_TARGET_LIBRARIES TRGTNAME)
	SET(${TRGTNAME}_LIBRARIES)
	FIND_PACKAGE_ADD_TARGET_LIBRARIES("${TRGTNAME}" "${${TRGTNAME}_LIBRARY}" "${${TRGTNAME}_LIBRARY_DEBUG}")
	SET(${TRGTNAME}_LIBRARIES ${${TRGTNAME}_LIBRARIES} PARENT_SCOPE)
ENDFUNCTION()


MACRO(FIND_PACKAGE_SET_STD_INCLUDE_AND_LIBS TRGTNAME)
	IF(${TRGTNAME}_FOUND)
		FIND_PACKAGE_SET_TARGET_LIBRARIES(${TRGTNAME})
		SET(${TRGTNAME}_INCLUDE_DIRS ${${TRGTNAME}_INCLUDE_DIR})
	ELSE()
		SET(${TRGTNAME}_LIBRARIES)
		SET(${TRGTNAME}_INCLUDE_DIRS)
	ENDIF()
ENDMACRO()
