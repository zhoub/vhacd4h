CMAKE_MINIMUM_REQUIRED( VERSION 3.2 )

PROJECT( HACD4H )

# We use the libraries from Houdini as much as possible.
FIND_PATH( HOUDINI_DIR REQUIRED )
IF ( MSVC )
    SET( HOUDINI_INCLUDE_DIR "${HOUDINI_DIR}/toolkit/include" )
    SET( HOUDINI_LIBRARY_DIR "${HOUDINI_DIR}/custom/houdini/dsolib" )
ELSEIF( APPLE )
    SET( HOUDINI_INCLUDE_DIR "${HOUDINI_DIR}/Resources/toolkit/include" )
    SET( HOUDINI_LIBRARY_DIR "${HOUDINI_DIR}/Libraries" )
ELSE()
    SET( HOUDINI_INCLUDE_DIR "${HOUDINI_DIR}/toolkit/include" )
    SET( HOUDINI_LIBRARY_DIR "${HOUDINI_DIR}/dsolib" )
ENDIF()

ADD_SUBDIRECTORY( lib )
ADD_SUBDIRECTORY( src )
