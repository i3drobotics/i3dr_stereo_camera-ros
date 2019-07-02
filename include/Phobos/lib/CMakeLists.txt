
#this is only an istaller of paraview with our plugins
set( CPACK_SIGNTOOL_EXE "${SIGNTOOL_EXE}" )
string( REPLACE "\\" "\\\\" _certificate_file "${SIGNTOOL_CERTIFICATE_FILE}" )
set( CPACK_SIGNTOOL_CERTIFICATE_FILE "${_certificate_file}" )
set( CPACK_SIGNTOOL_CERTIFICATE_PASSWORD "${SIGNTOOL_CERTIFICATE_PASSWORD}" )
if ( SIGNTOOL_TIMESTAMP_SERVER )
    set( CPACK_SIGNTOOL_TIMESTAMP "/tr ${SIGNTOOL_TIMESTAMP_SERVER}")
else()
    set( CPACK_SIGNTOOL_TIMESTAMP "")
endif()


if( HAVE_PARAVIEW )
	FILE( WRITE ${PROJECT_BINARY_DIR}/.plugins "
	<?xml version=\"1.0\"?>
	<Plugins>
	  <Plugin name=\"AnalyzeNIfTIIO\" auto_load=\"0\"/>
	  <Plugin name=\"ArrowGlyph\" auto_load=\"0\"/>
	  <Plugin name=\"CoProcessingPlugin\" auto_load=\"0\"/>
	  <Plugin name=\"EyeDomeLightingView\" auto_load=\"0\"/>
	  <Plugin name=\"GMVReader\" auto_load=\"0\"/>
	  <Plugin name=\"H5PartReader\" auto_load=\"0\"/>
	  <Plugin name=\"MobileRemoteControl\" auto_load=\"0\"/>
	  <Plugin name=\"Moments\" auto_load=\"0\"/>
	  <Plugin name=\"NonOrthogonalSource\" auto_load=\"0\"/>
	  <Plugin name=\"PointSprite_Plugin\" auto_load=\"0\"/>
	  <Plugin name=\"PrismPlugin\" auto_load=\"0\"/>
	  <Plugin name=\"QuadView\" auto_load=\"0\"/>
	  <Plugin name=\"SciberQuestToolKit\" auto_load=\"0\"/>
	  <Plugin name=\"SierraPlotTools\" auto_load=\"0\"/>
	  <Plugin name=\"SLACTools\" auto_load=\"0\"/>
	  <Plugin name=\"StreamingParticles\" auto_load=\"0\"/>
	  <Plugin name=\"SurfaceLIC\" auto_load=\"0\"/>
	  <Plugin name=\"PacMan\" auto_load=\"0\"/>
	  <Plugin name=\"UncertaintyRendering\" auto_load=\"0\"/>
	</Plugins>" )
ENDIF()

if(WIN32)
	SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64") 
endif()
IF ( RSGBASE_FOUND )
    INSTALL( DIRECTORY ${RSGBASE_DIR}/bin/ DESTINATION bin COMPONENT ParaView FILES_MATCHING PATTERN "*.dll" )
ENDIF ( RSGBASE_FOUND )

if( HAVE_PARAVIEW )
	install(
		FILES
		#${CMAKE_RUNTIME_OUTPUT_DIRECTORY}${RELEASE_OUT_DIR}/paraview.bat
		${CMAKE_RUNTIME_OUTPUT_DIRECTORY}${RELEASE_OUT_DIR}/paraview.exe
		${PROJECT_BINARY_DIR}/.plugins
		${DIGVTK_PYD_FILES_RELEASE}
		#${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/PointCloudPlugin/doc/PointCloudPlugin.qch
		#${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/EnsensoPlugin/doc/EnsensoPlugin.qch
		#${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/EnsensoPlugin/doc/DigVTKPlugin.qch
		#${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/DepthImagePlugin/doc/DepthImagePlugin.qch
		DESTINATION bin
		COMPONENT ParaView
    )
ENDIF()

############################################
# opencv has pdb files, remove it....
set(cleaned )
FOREACH(item ${PHOBOS_REQUIRED_FILES_RELEASE})
   #message(asdf1- ${item})
  STRING(REGEX MATCH "[A-Za-z0-9_]*pdb" item2 ${item})
  #message(asdf2- ${item2})
  if (item2 STREQUAL "")
	LIST(APPEND cleaned ${item})
  else()
  ENDIF()
ENDFOREACH(item ${My_Sources})
set(PHOBOS_REQUIRED_FILES_RELEASE ${cleaned})

#FOREACH( file ${DIGVTK_REQUIRED_FILES_RELEASE})
#    message(asdf ${file})
#ENDFOREACH()
#end remove pdb
#############################################

install(
    DIRECTORY 
	${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../Licenses
    DESTINATION Licenses/..
    COMPONENT runtime3p
    )
	
if(WIN32)
	install(
		FILES
	    ${PHOBOS_REQUIRED_FILES_RELEASE}
		${DIGVTK_REQUIRED_FILES_RELEASE}
		${DIGTKREQUIRED}
		${DIBGEN_REQUIRED_FILES_RELEASE}
		${DIBGIOM_REQUIRED_FILES_RELEASE}
		${DIBGEOMETRY_REQUIRED_FILES_RELEASE}
		${DIBQTGUI_REQUIRED_FILES_RELEASE}
		${DIBMATCH_REQUIRED_FILES_RELEASE}	
		${KINECT_REQUIRED_FILES_RELEASE}
		${CMAKE_RUNTIME_OUTPUT_DIRECTORY}${RELEASE_OUT_DIR}/VersionInfo.dll
		#${cudadlls}
		DESTINATION bin
		COMPONENT runtime3p
	)
else()
    SET(CPACK_SYSTEM_NAME x86_64 )
    find_program( PATCHELF patchelf)
	if(0)
		set(_resolvedFiles "")
		foreach(_file ${PHOBOS_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIGVTK_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIGTK_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIBGEN_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIBGIOM_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIBGEOMETRY_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIBQTGUI_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${DIBMATCH_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()
		foreach(_file ${KINECT_REQUIRED_FILES_RELEASE})
			get_filename_component(_resolvedFile "${_file}" REALPATH)
			list (APPEND _resolvedFiles "${_resolvedFile}")
		endforeach()

		install(
			FILES
		${_resolvedFiles}
			${DIGVTK_REQUIRED_FILES_RELEASE}
			#${cudadlls}
			DESTINATION lib
			COMPONENT runtime3p
		)
	else()
		set( INSTALL_UPDATE_INSTALLED_3RD_PARTY_RPATH TRUE CACHE BOOL "" )
	    set( CMAKE_INSTALL_RPATH "\$ORIGIN/../\$LIB" CACHE PATH "" )

        set( INSTALL_LAPACK_LIBRARIES TRUE CACHE BOOL "Specify wether the found lapack libraries shall be installed" )
        if( INSTALL_LAPACK_LIBRARIES )
            if ( ${LAPACK_SELECTED_VARIANT} AND ${LAPACK_SELECTED_VARIANT} STREQUAL "MKL" )
                INSTALL( DIRECTORY ${LAPACK_LIBRARY_DIR}/ DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME}
                         COMPONENT runtime3p FILES_MATCHING PATTERN "*.so" )
            else()
                GET_TARGET_PROPERTY( LAPACKE_LIBRARY lapacke IMPORTED_LOCATION_RELEASE )
                install( PROGRAMS ${LAPACKE_LIBRARY} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

                GET_TARGET_PROPERTY( LAPACK_LIBRARY lapack IMPORTED_LOCATION_RELEASE )
                install( PROGRAMS ${LAPACK_LIBRARY} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

                GET_TARGET_PROPERTY( BLAS_LIBRARY blas IMPORTED_LOCATION_RELEASE )
                install( PROGRAMS ${BLAS_LIBRARY} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()
        endif( INSTALL_LAPACK_LIBRARIES )
        set( INSTALL_BOOST_LIBRARIES TRUE CACHE BOOL "Specify wether the found Boost Libraries shall be installed" )
        if( INSTALL_BOOST_LIBRARIES )
            file( GLOB BOOST_LIB_FILES ${BOOST_DATE_TIME_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_FILESYSTEM_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_CHRONO_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_THREAD_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_PROGRAM_OPTIONS_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_PYTHON_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_SIGNALS_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_REGEX_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            file( GLOB BOOST_LIB_FILES ${BOOST_SERIALIZATION_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )

            if ( BOOST_VERSION GREATER "1.48.0" )
                file( GLOB BOOST_LIB_FILES ${BOOST_TIMER_LIBRARY}* )
                install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()

            if ( BOOST_VERSION GREATER "1.62.0" )
                file( GLOB BOOST_LIB_FILES ${BOOST_NUMPY_LIBRARY}* )
                install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()

            file( GLOB BOOST_LIB_FILES ${BOOST_SYSTEM_LIBRARY}* )
            install( PROGRAMS ${BOOST_LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
        endif( INSTALL_BOOST_LIBRARIES )

        set( INSTALL_GDAL_LIBRARIES TRUE CACHE BOOL "Specify wether the found GDAL Libraries shall be installed" )
        if( INSTALL_GDAL_LIBRARIES )
            file( GLOB LIB_FILES ${GDAL_GDAL_LIBRARY_RELEASE}* )
            install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            # install some usefull programs
            install( PROGRAMS ${GDAL_OGR_INFO} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_OGR2OGR} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_INFO} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_TRANSLATE} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_ADDO} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_BUILDVRT} DESTINATION bin COMPONENT runtime3p )
            if ( INSTALL_UPDATE_INSTALLED_3RD_PARTY_RPATH )
                set( _gdal_apps ogr2ogr ogrinfo gdalinfo gdal_translate gdaladdo gdalbuildvrt )
                install( CODE "\n\
                               message( STATUS \"Updating rpath\" )\n\
                               foreach( _gdal_app ${_gdal_apps} )\n\
                                 set( _gdal_exe \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/\${_gdal_app} ) \n\
                                 #message( \"app: \" \${_gdal_exe} )\n\
                                 execute_process(COMMAND ${PATCHELF} --set-rpath \"\$ORIGIN/../\$LIB\" \${_gdal_exe} RESULT_VARIABLE _patchelf_result ) \n\
                                 if ( NOT \${_patchelf_result} STREQUAL \"0\" )\n\
                                   message( FATAL_ERROR \"${PATCHELF} execution failed \" )\n\
                                 endif()\n\
                               endforeach()"
                         COMPONENT runtime3p)
            endif()

            if ( PROJ_FOUND )
                # also install proj which is used by gdal in case certain cartographic transofmations are done
                file( GLOB LIB_FILES ${PROJ_LIBRARY_RELEASE}* )
                install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()

            if ( OPENJPEG_FOUND )
                # also install openjepg which is required by gdal from 1.10.0 onwards
                file( GLOB LIB_FILES ${OPENJPEG_LIBRARY_RELEASE}* )
                install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()

            if ( EXPAT_FOUND )
                # also install expat which may be required by gdal from 1.10.1 onwards when using the WithExpat Variant to support *.kml files
                file( GLOB LIB_FILES ${EXPAT_LIBEXPAT_LIBRARY_RELEASE}* )
                install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()
        else()
            # install only the usefull programs, as some PHOBOS progs expect them  to be in the PHOBOS installation
            # e.g. PythonRunPackageConf.runProgStdout( "gdalbuildvrt" )
            # but these installed progs will use the gdal lib from the found Toolchain root or system location.
            install( PROGRAMS ${GDAL_OGR_INFO} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_OGR2OGR} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_INFO} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_TRANSLATE} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_ADDO} DESTINATION bin COMPONENT runtime3p )
            install( PROGRAMS ${GDAL_BUILDVRT} DESTINATION bin COMPONENT runtime3p )
        endif()
        set( INSTALL_QT_LIBRARIES TRUE CACHE BOOL "Specify wether the found QT Libraries shall be installed" )
        if( INSTALL_QT_LIBRARIES )
            foreach ( _qtlib Qt5::Core Qt5::Widgets Qt5::Gui Qt5::Network Qt5::Sql Qt5::Xml Qt5::Svg Qt5::PrintSupport Qt5::DBus)
                GET_TARGET_PROPERTY( _qtlib_loc ${_qtlib} IMPORTED_LOCATION_RELEASE )
                GET_TARGET_PROPERTY( _qtlib_soname ${_qtlib} IMPORTED_SONAME_RELEASE )
                GET_FILENAME_COMPONENT( _qtlib_dir ${_qtlib_loc} PATH )

                file( GLOB _lib_files ${_qtlib_dir}/${_qtlib_soname}* )
                install( PROGRAMS ${_lib_files} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endforeach()

            # also install the additional icu related libs which are used indirectly by the qt5 libraries
            GET_TARGET_PROPERTY( _loc Qt5::Core IMPORTED_LOCATION_RELEASE )
            GET_FILENAME_COMPONENT( _loc_dir ${_loc} PATH )
            file( GLOB LIB_FILES ${_loc_dir}/libicu*.so.* )
            install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            file( GLOB LIB_FILES ${_loc_dir}/libQt5XcbQpa.so.* )
            if( LIB_FILES )
                install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
            endif()

            # possible plugin lists:  Qt Network, Qt SQL, Qt GUI, and Qt Widgets
            foreach( plugin ${Qt5Gui_PLUGINS})
                get_target_property(_loc ${plugin} LOCATION)
                get_filename_component( _loc_name ${_loc} NAME )
                get_filename_component( _loc_dir ${_loc} PATH )
                get_filename_component( _loc_dir_name ${_loc_dir} NAME )
                install( PROGRAMS ${_loc} DESTINATION bin/${_loc_dir_name} COMPONENT runtime3p )

                if ( INSTALL_UPDATE_INSTALLED_3RD_PARTY_RPATH )
        #             install( CODE "MESSAGE( STATUS \"Updating rpath\" )\nfile( GLOB _installed_lib \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${_loc_dir_name}/${_loc_name} )\nEXECUTE_PROCESS(COMMAND patchelf --set-rpath \"\$ORIGIN/../../\$LIB\" \${_installed_lib} )" COMPONENT runtime3p)
                    install( CODE "message( STATUS \"Updating rpath\" )\n\
                                   file( GLOB _installed_plugins \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${_loc_dir_name}/${_loc_name} )\n\
                                   foreach( _installed_plugin \${_installed_plugins} )\n\
                                   #message( \"lib: \" \${_installed_plugin} )\n\
                                   execute_process(COMMAND ${PATCHELF} --set-rpath \"\$ORIGIN/../../\$LIB\" \${_installed_plugin} RESULT_VARIABLE _patchelf_result ) \n\
                                   if ( NOT \${_patchelf_result} STREQUAL \"0\" )\n\
                                       message( FATAL_ERROR \"${PATCHELF} execution failed\" )\n\
                                   endif()\n\
                                   endforeach()"
                             COMPONENT runtime3p)

                endif()
            endforeach()

            foreach( plugin ${Qt5Svg_PLUGINS})
                get_target_property(_loc ${plugin} LOCATION)
                get_filename_component( _loc_name ${_loc} NAME )
                get_filename_component( _loc_dir ${_loc} PATH )
                get_filename_component( _loc_dir_name ${_loc_dir} NAME )
                install( PROGRAMS ${_loc} DESTINATION bin/${_loc_dir_name} COMPONENT runtime3p )

                if ( INSTALL_UPDATE_INSTALLED_3RD_PARTY_RPATH )
        #             install( CODE "MESSAGE( STATUS \"Updating rpath\" )\nfile( GLOB _installed_lib \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${_loc_dir_name}/${_loc_name} )\nEXECUTE_PROCESS(COMMAND patchelf --set-rpath \"\$ORIGIN/../../\$LIB\" \${_installed_lib} )" COMPONENT runtime3p)
                    install( CODE "message( STATUS \"Updating rpath\" )\n\
                                   file( GLOB _installed_plugins \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/bin/${_loc_dir_name}/${_loc_name} )\n\
                                   foreach( _installed_plugin \${_installed_plugins} )\n\
                                   #message( \"lib: \" \${_installed_plugin} )\n\
                                   execute_process(COMMAND ${PATCHELF} --set-rpath \"\$ORIGIN/../../\$LIB\" \${_installed_plugin} RESULT_VARIABLE _patchelf_result ) \n\
                                   if ( NOT \${_patchelf_result} STREQUAL \"0\" )\n\
                                       message( FATAL_ERROR \"${PATCHELF} execution failed\" )\n\
                                   endif()\n\
                                   endforeach()"
                             COMPONENT runtime3p)
                endif()
            endforeach()

        endif( INSTALL_QT_LIBRARIES )

            set( INSTALL_TBB_LIBRARIES TRUE CACHE BOOL "Specify wether the found TBB Libraries shall be installed" )
            if( INSTALL_TBB_LIBRARIES  )
            file( GLOB LIB_FILES ${TBB_LIBRARY_RELEASE}* )
            install( PROGRAMS ${LIB_FILES} DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p )
        endif( INSTALL_TBB_LIBRARIES )

		set( INSTALL_CUDA_LIBRARIES TRUE CACHE BOOL "Specify wether the found CUDA Libraries shall be installed" )
		if( INSTALL_CUDA_LIBRARIES )
			get_filename_component( _real_cuda_lib_path ${CUDA_TOOLKIT_ROOT_DIR}/lib64/ REALPATH )
			install( DIRECTORY ${_real_cuda_lib_path}/ DESTINATION ${PHOBOS_INSTALL_LIBDIR_NAME} COMPONENT runtime3p
					 FILES_MATCHING PATTERN "lib*.so*" )
		endif()

		if ( INSTALL_UPDATE_INSTALLED_3RD_PARTY_RPATH )
			# the rpath in the installed libs shall be changed!!!
			install( CODE "message( STATUS \"Updating rpath\" )\n\
			file( GLOB _installed_libs \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${PHOBOS_INSTALL_LIBDIR_NAME}/lib*.so* )\n\
			foreach( _installed_lib \${_installed_libs} )\n\
			  if ( NOT IS_SYMLINK \${_installed_lib} ) \n\
				# message( \"lib: \" \${_installed_lib} )\n\
				execute_process(COMMAND ${PATCHELF} --set-rpath \"\$ORIGIN\" \${_installed_lib} RESULT_VARIABLE _patchelf_result ) \n\
				if ( NOT \${_patchelf_result} STREQUAL \"0\" )\n\
				  message( ERROR \"${PATCHELF} execution failed for \${_installed_lib}\" )\n\
				endif()\n\
			  endif()\n\
			endforeach()"
					 COMPONENT runtime3p)
		endif()
		#i dont know why this is not found automatically
		install(
			FILES
			/opt/cub/1.6.4/License.txt
			DESTINATION Licenses/cub
			COMPONENT runtime3p)
		install(
			FILES
			/opt/qt/5.5.1/License.txt
			DESTINATION Licenses/qt
			COMPONENT runtime3p)
		#install(
		#    FILES
		#    /opt/eigen/3.2.8/License.txt
	    #    DESTINATION Licenses/Eigen
        #    COMPONENT runtime3p)    
    endif()    
endif()       


IF(MSVC)
	# possible plugin lists:  Qt Network, Qt SQL, Qt GUI, and Qt Widgets
	FOREACH( plugin ${Qt5Gui_PLUGINS})
		GET_TARGET_PROPERTY(_loc ${plugin} LOCATION)
		GET_FILENAME_COMPONENT( _loc_dir ${_loc} PATH )
		GET_FILENAME_COMPONENT( _loc_dir_name ${_loc_dir} NAME )
		INSTALL( PROGRAMS ${_loc} DESTINATION bin/${_loc_dir_name} COMPONENT runtime )
	ENDFOREACH()
	
    FILE(TO_CMAKE_PATH "$ENV{SYSTEMROOT}" SYSTEMROOT)
    IF(MSVC12)
            IF(CMAKE_CL_64)
                SET( MSVC_ARCH x64 )
            ELSE(CMAKE_CL_64)
                SET( MSVC_ARCH x86 )
            ENDIF(CMAKE_CL_64)
            
            # Find the runtime library redistribution directory.
            #FIND_PATH( MSVC120_REDIST_DIR NAMES ${MSVC_ARCH}/Microsoft.VC120.CRT/Microsoft.VC120.CRT.manifest
            #                PATHS "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\12.0;InstallDir]/../../VC/redist"
            #                        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VCExpress\\12.0;InstallDir]/../../VC/redist"
            #                      "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\VisualStudio\\12.0;InstallDir]/../../VC/redist"
            #                        "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\VCExpress\\12.0;InstallDir]/../../VC/redist"                        )
            FIND_PATH( MSVC120_REDIST_DIR NAMES ${MSVC_ARCH}/Microsoft.VC120.CRT/msvcp120.dll
                            PATHS "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\12.0;InstallDir]/../../VC/redist"
                                    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VCExpress\\12.0;InstallDir]/../../VC/redist"
                                  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\VisualStudio\\12.0;InstallDir]/../../VC/redist"
                                    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Wow6432Node\\Microsoft\\VCExpress\\12.0;InstallDir]/../../VC/redist"                        )
            
            #MARK_AS_ADVANCED(MSVC120_REDIST_DIR)
            SET(MSVC120_CRT_DIR "${MSVC120_REDIST_DIR}/${MSVC_ARCH}/Microsoft.VC120.CRT")
            SET(MSVC120_OPENMP_DIR "${MSVC120_REDIST_DIR}/${MSVC_ARCH}/Microsoft.VC120.OpenMP")
            SET(MSVC120_CXXAMP_DIR "${MSVC120_REDIST_DIR}/${MSVC_ARCH}/Microsoft.VC120.CXXAMP")
            # Install the manifest that allows DLLs to be loaded from the
            # directory containing the executable.
            #INSTALL( PROGRAMS "${MSVC120_CRT_DIR}/Microsoft.VC120.CRT.manifest" DESTINATION bin COMPONENT runtime )
            INSTALL( PROGRAMS "${MSVC120_CRT_DIR}/msvcp120.dll" DESTINATION bin COMPONENT runtime )
            INSTALL( PROGRAMS "${MSVC120_CRT_DIR}/msvcr120.dll" DESTINATION bin COMPONENT runtime )
            INSTALL( PROGRAMS "${MSVC120_CRT_DIR}/vccorlib120.dll" DESTINATION bin COMPONENT runtime )
            INSTALL( PROGRAMS "${MSVC120_OPENMP_DIR}/vcomp120.dll" DESTINATION bin COMPONENT runtime )
            INSTALL( PROGRAMS "${MSVC120_CXXAMP_DIR}/vcamp120.dll" DESTINATION bin COMPONENT runtime )
    ENDIF(MSVC12)
ENDIF(MSVC)
        
set(CPACK_PACKAGE_NAME "Phobos")
set(CPACK_PACKAGE_VENDOR "www.joanneum.at")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Phobos - Installation")
set(CPACK_PACKAGE_VERSION_MAJOR 1)
set(CPACK_PACKAGE_VERSION_MINOR 0)
set(CPACK_PACKAGE_VERSION_PATCH 0)

IF(Subversion_FOUND)
    Subversion_WC_INFO(${CMAKE_SOURCE_DIR} Phobos)
    message("Revision is " ${Phobos_WC_REVISION} )
    set(CPACK_PACKAGE_VERSION_PATCH ${Phobos_WC_REVISION})
ENDIF(Subversion_FOUND)
set(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})
set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/Licence.txt)

set(CPACK_COMPONENTS_ALL runtime runtime3p phobos )
#set( CPACK_COMPONENTS_ALL runtime runtime3p tools scripts documentation )
SET(CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};runtime;runtime;/;${PROJECT_BINARY_DIR};runtime3p;runtime3p;/;${PROJECT_BINARY_DIR};phobos;phobos;/" )

set(CPACK_DEBIAN_PACKAGE_NAME phobos)
set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "JOANNEUM RESEARCH")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Phobos gpu stereo matcher")
set(CPACK_DEBIAN_INSTALL_DIRECTORY "${CMAKE_INSTALL_PREFIX}")
set(CPACK_DEBIAN_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

set(CPACK_DEVELOP_COMPONENT_INSTALL OFF )
set(CPACK_develop_COMPONENT_INSTALL OFF )
set(CPACK_COMPONENT_RUNTIME3P_HIDDEN 1)
set(CPACK_COMPONENT_SIGNTOOL_HIDDEN 1)
set(CPACK_COMPONENT_RUNTIME_DISPLAY_NAME "Runtime binaries")
set(CPACK_COMPONENT_RUNTIME3P_DISPLAY_NAME "3rd party binaries")
set(CPACK_COMPONENT_DEVELOP_DISPLAY_NAME "Develop (Headers)")
set(CPACK_COMPONENT_PHOBOS_DISPLAY_NAME "Develop (Headers)")

SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\paraview.exe")
SET(CPACK_PACKAGE_EXECUTABLES "MultiGPUMatcher;MultiGPUMatcher")

#SET(CPACK_NSIS_CREATE_ICONS_EXTRA "CreateShortCut \\\"$DESKTOP\\\\JR3DViewer.lnk\\\" \\\"$INSTDIR\\\\bin\\\\JR3DViewer.exe\\\"")
SET(CPACK_NSIS_CREATE_ICONS_EXTRA "CreateShortCut \\\"$DESKTOP\\\\ImageViewer.lnk\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe\\\"")

SET(CPACK_NSIS_PACKAGE_NAME "JOANNEUM RESEARCH/Phobos")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "JOANNEUM RESEARCH/Phobos")
SET(CPACK_NSIS_DISPLAY_NAME "JOANNEUM RESEARCH/Phobos")

IF( WIN32 )
  SET( CPACK_PACKAGE_INSTALL_DIRECTORY JOANNEUM\ RESEARCH\\\\Phobos )
ELSE( WIN32 )
  #SET( CPACK_PACKAGE_INSTALL_DIRECTORY JOANNEUM\ RESEARCH/Phobos )
  SET( CPACK_PACKAGE_INSTALL_DIRECTORY "${CMAKE_INSTALL_PREFIX}")
  SET( CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
  set( CPACK_GENERATOR DEB )
  #set( CPACK_GENERATOR RPM )
message(CPACK_PACKAGING_INSTALL_PREFIX ${CPACK_PACKAGING_INSTALL_PREFIX})
ENDIF( WIN32 )

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

set( CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} signtool )

if( WIN32)
# File types association:
# why does CPACK_NSIS_DEFINES not work????
#SET(CPACK_NSIS_DEFINES "!include \\\"${PROJECT_SOURCE_DIR}/Applications/Paraview\\\\FileAssoc.nsh\\\"")
SET(CPACK_NSIS_DEFINES_ADDITIONAL "!include \\\"${CMAKE_CURRENT_SOURCE_DIR}\\\\FileAssoc.nsh\\\"")

if( HAVE_PARAVIEW ) 
    SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
      !insertmacro APP_ASSOCIATE \\\"pgm\\\" \\\"paraview.pgm\\\" \\\"pgm image\\\" \\\"$INSTDIR\\\\bin\\\\paraview.exe,0\\\" \\\"Open with ParaView\\\" \\\"$INSTDIR\\\\bin\\\\paraview.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"vtk\\\" \\\"paraview.vtk\\\" \\\"vtk object\\\" \\\"$INSTDIR\\\\bin\\\\paraview.exe,0\\\" \\\"Open with ParaView\\\" \\\"$INSTDIR\\\\bin\\\\paraview.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jpeg\\\" \\\"ImageViewer.jpeg\\\" \\\"jpeg image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jpg\\\" \\\"ImageViewer.jpg\\\" \\\"jpg image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"img\\\" \\\"ImageViewer.img\\\" \\\"jpg image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jp2\\\" \\\"ImageViewer.jp2\\\" \\\"jp2 image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"j2k\\\" \\\"ImageViewer.j2k\\\" \\\"j2k image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"ers\\\" \\\"ImageViewer.ers\\\" \\\"ers image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"tif\\\" \\\"ImageViewer.tif\\\" \\\"tif image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"tiff\\\" \\\"ImageViewer.tiff\\\" \\\"tiff image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"pgm\\\" \\\"ImageViewer.pgm\\\" \\\"pgm image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"ppm\\\" \\\"ImageViewer.ppm\\\" \\\"ppm image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"pnm\\\" \\\"ImageViewer.pnm\\\" \\\"pnm image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"bmp\\\" \\\"ImageViewer.bmp\\\" \\\"bmp image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"mha\\\" \\\"ImageViewer.mha\\\" \\\"mha image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jrmha\\\" \\\"ImageViewer.jrmha\\\" \\\"jr mha image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"mhd\\\" \\\"ImageViewer.mhd\\\" \\\"mhd image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"gif\\\" \\\"ImageViewer.gif\\\" \\\"gif image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"dcm\\\" \\\"ImageViewer.dcm\\\" \\\"DICOM image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"png\\\" \\\"ImageViewer.png\\\" \\\"png image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"pcd\\\" \\\"ImageViewer.pcd\\\" \\\"pcd image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"match\\\" \\\"ImageViewer.match\\\" \\\"match image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"rplidar\\\" \\\"ImageViewer.rplidar\\\" \\\"rplidar file\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"rplidar\\\" \\\"paraview.rplidar\\\" \\\"rplidar file\\\" \\\"$INSTDIR\\\\bin\\\\paraview.exe,0\\\" \\\"Open with ParaView\\\" \\\"$INSTDIR\\\\bin\\\\paraview.exe $\\\\\\\"%1$\\\\\\\"\\\"
      
      ")

    SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "
      !insertmacro APP_UNASSOCIATE \\\"pgm\\\" \\\"paraview.pgm\\\"
      !insertmacro APP_UNASSOCIATE \\\"vtk\\\" \\\"paraview.vtk\\\"
      !insertmacro APP_UNASSOCIATE \\\"jpeg\\\" \\\"ImageViewer.jpeg\\\"
      !insertmacro APP_UNASSOCIATE \\\"jpg\\\" \\\"ImageViewer.jpg\\\"
      !insertmacro APP_UNASSOCIATE \\\"img\\\" \\\"ImageViewer.img\\\"
      !insertmacro APP_UNASSOCIATE \\\"jp2\\\" \\\"ImageViewer.jp2\\\"
      !insertmacro APP_UNASSOCIATE \\\"j2k\\\" \\\"ImageViewer.j2k\\\"
      !insertmacro APP_UNASSOCIATE \\\"ers\\\" \\\"ImageViewer.ers\\\"
      !insertmacro APP_UNASSOCIATE \\\"tif\\\" \\\"ImageViewer.tif\\\"
      !insertmacro APP_UNASSOCIATE \\\"tiff\\\" \\\"ImageViewer.tiff\\\"
      !insertmacro APP_UNASSOCIATE \\\"pgm\\\" \\\"ImageViewer.pgm\\\"
      !insertmacro APP_UNASSOCIATE \\\"ppm\\\" \\\"ImageViewer.ppm\\\"
      !insertmacro APP_UNASSOCIATE \\\"pnm\\\" \\\"ImageViewer.pnm\\\"
      !insertmacro APP_UNASSOCIATE \\\"bmp\\\" \\\"ImageViewer.bmp\\\"
      !insertmacro APP_UNASSOCIATE \\\"mha\\\" \\\"ImageViewer.mha\\\"
      !insertmacro APP_UNASSOCIATE \\\"jrmha\\\" \\\"ImageViewer.jrmha\\\"
      !insertmacro APP_UNASSOCIATE \\\"mhd\\\" \\\"ImageViewer.mhd\\\"
      !insertmacro APP_UNASSOCIATE \\\"gif\\\" \\\"ImageViewer.gif\\\"
      !insertmacro APP_UNASSOCIATE \\\"dcm\\\" \\\"ImageViewer.dcm\\\"
      !insertmacro APP_UNASSOCIATE \\\"png\\\" \\\"ImageViewer.png\\\"
      !insertmacro APP_UNASSOCIATE \\\"pcd\\\" \\\"ImageViewer.pcd\\\"
      !insertmacro APP_UNASSOCIATE \\\"match\\\" \\\"ImageViewer.match\\\"
      !insertmacro APP_UNASSOCIATE \\\"rplidar\\\" \\\"ImageViewer.rplidar\\\"
      !insertmacro APP_UNASSOCIATE \\\"rplidar\\\" \\\"paraview.rplidar\\\"
    ")
ELSE()
    SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
      !insertmacro APP_ASSOCIATE \\\"jpeg\\\" \\\"ImageViewer.jpeg\\\" \\\"jpeg image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jpg\\\" \\\"ImageViewer.jpg\\\" \\\"jpg image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"img\\\" \\\"ImageViewer.img\\\" \\\"jpg image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jp2\\\" \\\"ImageViewer.jp2\\\" \\\"jp2 image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"j2k\\\" \\\"ImageViewer.j2k\\\" \\\"j2k image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"ers\\\" \\\"ImageViewer.ers\\\" \\\"ers image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"tif\\\" \\\"ImageViewer.tif\\\" \\\"tif image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"tiff\\\" \\\"ImageViewer.tiff\\\" \\\"tiff image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"pgm\\\" \\\"ImageViewer.pgm\\\" \\\"pgm image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"ppm\\\" \\\"ImageViewer.ppm\\\" \\\"ppm image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"pnm\\\" \\\"ImageViewer.pnm\\\" \\\"pnm image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"bmp\\\" \\\"ImageViewer.bmp\\\" \\\"bmp image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"mha\\\" \\\"ImageViewer.mha\\\" \\\"mha image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"jrmha\\\" \\\"ImageViewer.jrmha\\\" \\\"jr mha image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"mhd\\\" \\\"ImageViewer.mhd\\\" \\\"mhd image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"gif\\\" \\\"ImageViewer.gif\\\" \\\"gif image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"dcm\\\" \\\"ImageViewer.dcm\\\" \\\"DICOM image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"png\\\" \\\"ImageViewer.png\\\" \\\"png image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"pcd\\\" \\\"ImageViewer.pcd\\\" \\\"pcd image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"      
      !insertmacro APP_ASSOCIATE \\\"match\\\" \\\"ImageViewer.match\\\" \\\"match image\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      !insertmacro APP_ASSOCIATE \\\"rplidar\\\" \\\"ImageViewer.rplidar\\\" \\\"rplidar file\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe,0\\\" \\\"Open with ImageViewer\\\" \\\"$INSTDIR\\\\bin\\\\ImageViewer.exe $\\\\\\\"%1$\\\\\\\"\\\"
      ")

    SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS ${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS} "
      !insertmacro APP_UNASSOCIATE \\\"jpeg\\\" \\\"ImageViewer.jpeg\\\"
      !insertmacro APP_UNASSOCIATE \\\"jpg\\\" \\\"ImageViewer.jpg\\\"
      !insertmacro APP_UNASSOCIATE \\\"img\\\" \\\"ImageViewer.img\\\"
      !insertmacro APP_UNASSOCIATE \\\"jp2\\\" \\\"ImageViewer.jp2\\\"
      !insertmacro APP_UNASSOCIATE \\\"j2k\\\" \\\"ImageViewer.j2k\\\"
      !insertmacro APP_UNASSOCIATE \\\"ers\\\" \\\"ImageViewer.ers\\\"
      !insertmacro APP_UNASSOCIATE \\\"tif\\\" \\\"ImageViewer.tif\\\"
      !insertmacro APP_UNASSOCIATE \\\"tiff\\\" \\\"ImageViewer.tiff\\\"
      !insertmacro APP_UNASSOCIATE \\\"pgm\\\" \\\"ImageViewer.pgm\\\"
      !insertmacro APP_UNASSOCIATE \\\"ppm\\\" \\\"ImageViewer.ppm\\\"
      !insertmacro APP_UNASSOCIATE \\\"pnm\\\" \\\"ImageViewer.pnm\\\"
      !insertmacro APP_UNASSOCIATE \\\"bmp\\\" \\\"ImageViewer.bmp\\\"
      !insertmacro APP_UNASSOCIATE \\\"mha\\\" \\\"ImageViewer.mha\\\"
      !insertmacro APP_UNASSOCIATE \\\"jrmha\\\" \\\"ImageViewer.jrmha\\\"
      !insertmacro APP_UNASSOCIATE \\\"mhd\\\" \\\"ImageViewer.mhd\\\"
      !insertmacro APP_UNASSOCIATE \\\"gif\\\" \\\"ImageViewer.gif\\\"
      !insertmacro APP_UNASSOCIATE \\\"dcm\\\" \\\"ImageViewer.dcm\\\"
      !insertmacro APP_UNASSOCIATE \\\"png\\\" \\\"ImageViewer.png\\\"
      !insertmacro APP_UNASSOCIATE \\\"pcd\\\" \\\"ImageViewer.pcd\\\"
      !insertmacro APP_UNASSOCIATE \\\"match\\\" \\\"ImageViewer.match\\\"
      !insertmacro APP_UNASSOCIATE \\\"rplidar\\\" \\\"ImageViewer.rplidar\\\"
    ")
ENDIF()
#message(CMAKE_PROJECT_NAME ${CMAKE_PROJECT_NAME})
#set( CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};${CMAKE_PROJECT_NAME};signtool;/" )
endif()

include(CPack)