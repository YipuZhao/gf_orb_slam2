CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
IF(COMMAND cmake_policy)
	CMAKE_POLICY(SET CMP0003 NEW)
	CMAKE_POLICY(SET CMP0012 NEW) # silence the FindOpenMP if(true) warning
	CMAKE_POLICY(SET CMP0014 OLD) # working on a secret project, are we? this helps to get rid of warnings
ENDIF(COMMAND cmake_policy)

PROJECT(SLAM_plus_plus)

# dont ask me about those ...
MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY CMAKE_INSTALL_PREFIX EXECUTABLE_OUTPUT_PATH
	LIBRARY_OUTPUT_PATH SLAM_P_P_DATA_PATH SLAM_P_P_SUPPLIED_LIBS_PATH)

SET(SLAM_P_P_SOURCE_DIR ${PROJECT_SOURCE_DIR})

# Path project-specific CMake modules.
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

IF(NOT CMAKE_SIZEOF_VOID_P)
	MESSAGE(WARNING "'CMAKE_SIZEOF_VOID_P' is undefined, assuming 8 (x64).")
	SET(CMAKE_SIZEOF_VOID_P 8) # assume 8?
ENDIF(NOT CMAKE_SIZEOF_VOID_P)
# this is a quirk of a newer CMake, apparently, or some exotic platform problem

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	OPTION(SLAM_P_P_32BIT_BUILD "Build 32-bit binaries on 64-bit system." "OFF")
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

#MESSAGE(STATUS "UNIX = " ${UNIX})
#MESSAGE(STATUS "CMAKE_SYSTEM_PROCESSOR = " ${CMAKE_SYSTEM_PROCESSOR})
#MESSAGE(STATUS "CMAKE_SIZEOF_VOID_P = " ${CMAKE_SIZEOF_VOID_P})
#MESSAGE(STATUS "CMAKE_C_SIZEOF_DATA_PTR = " ${CMAKE_C_SIZEOF_DATA_PTR})
#MESSAGE(STATUS "SLAM_P_P_32BIT_BUILD = " ${SLAM_P_P_32BIT_BUILD})
#MESSAGE(STATUS "APPLE = " ${APPLE})
# debug

# Set default build type (note this does not cause the cmake -i display line asking for this value, this just makes sure it is set)
IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Default build type." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

IF(NOT SLAM_P_P_LIB_TYPE)
        SET(SLAM_P_P_LIB_TYPE "STATIC") # "SHARED"
ENDIF(NOT SLAM_P_P_LIB_TYPE)

# build target options
OPTION(SLAM_P_P_BUILD_EXAMPLES "Build examples." TRUE)
#IF(SLAM_P_P_BUILD_EXAMPLES)
#	OPTION(SLAM_P_P_BUILD_TOMS_EXAMPLE "Build TOMS example (requires external libraries for benchmarking)." FALSE)
#ENDIF(SLAM_P_P_BUILD_EXAMPLES)
OPTION(SLAM_P_P_BUILD_MAIN_APP "Build SLAM++ main application." TRUE)
IF(SLAM_P_P_BUILD_MAIN_APP)
	OPTION(SLAM_P_P_MAIN_APP_UNIT_TESTS "Enable SLAM++ built-in unit tests (in the main application)." FALSE)
	OPTION(SLAM_P_P_MAIN_APP_PERF_TESTS "Enable SLAM++ built-in perf tests (in the main application)." FALSE)
	
	mark_as_advanced(CLEAR SLAM_P_P_MAIN_APP_UNIT_TESTS)
	mark_as_advanced(CLEAR SLAM_P_P_MAIN_APP_PERF_TESTS) # unhide
ELSE(SLAM_P_P_BUILD_MAIN_APP)
	mark_as_advanced(FORCE SLAM_P_P_MAIN_APP_UNIT_TESTS)
	mark_as_advanced(FORCE SLAM_P_P_MAIN_APP_PERF_TESTS) # hide from the user but do not delete from cache
ENDIF(SLAM_P_P_BUILD_MAIN_APP)

# offer solvers
SET(SLAM_P_P_LINEAR_SOLVER_TYPE 3 CACHE STRING "Linear solver type (0 = CSparse, 1 = CXSparse, 2 = CHOLMOD, 3 = native). Note that the new fast L only works with native.")

IF(SLAM_P_P_LINEAR_SOLVER_TYPE EQUAL 2)
	# support for GPU BLAS
	OPTION(SLAM_P_P_GPU_BLAS "Use GPU acceleration for CHOLMOD." FALSE)

	# try using cholmod with int instead of long
	IF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
		OPTION(SLAM_P_P_SHORT_CHOLMOD "Use 32-bit indices for 64-bit CHOLMOD." FALSE)
	ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
	OPTION(SLAM_P_P_USE_METIS "Use the METIS library for CHOLMOD." FALSE)

	mark_as_advanced(CLEAR SLAM_P_P_GPU_BLAS)
	mark_as_advanced(CLEAR SLAM_P_P_SHORT_CHOLMOD)
	mark_as_advanced(CLEAR SLAM_P_P_USE_METIS) # unhide
ELSE(SLAM_P_P_LINEAR_SOLVER_TYPE EQUAL 2)
	mark_as_advanced(FORCE SLAM_P_P_GPU_BLAS)
	mark_as_advanced(FORCE SLAM_P_P_SHORT_CHOLMOD)
	mark_as_advanced(FORCE SLAM_P_P_USE_METIS) # hide from the user but do not delete from cache
ENDIF(SLAM_P_P_LINEAR_SOLVER_TYPE EQUAL 2)

# try using cholmod with int instead of long
IF(SLAM_P_P_LINEAR_SOLVER_TYPE EQUAL 1 AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
	OPTION(SLAM_P_P_SHORT_CXSPARSE "Use 32-bit indices for 64-bit CXSparse." FALSE)
	
	mark_as_advanced(CLEAR SLAM_P_P_SHORT_CXSPARSE) # unhide
ELSE(SLAM_P_P_LINEAR_SOLVER_TYPE EQUAL 1 AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
	mark_as_advanced(FORCE SLAM_P_P_SHORT_CXSPARSE) # hide from the user but do not delete from cache
ENDIF(SLAM_P_P_LINEAR_SOLVER_TYPE EQUAL 1 AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)

# memory alignment in SLAM++
OPTION(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY "Use memory alignment for graph primitives." FALSE)

# const vertices in SLAM++ (will slow down a bit, dont want it in benchmarks where there are no const vertices)
OPTION(SLAM_P_P_MAIN_APP_USE_CONST_VERTICES "SLAM++ main application const vertex support." FALSE)

# explicitly ask about compiler flags
IF(UNIX)
	IF(APPLE)
		OPTION(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS "Use aggressive optimizations." FALSE)
	ELSE(APPLE)
		OPTION(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS "Use aggressive optimizations." TRUE)
	ENDIF(APPLE)

	SET(MY_CXX_FLAGS_RELEASE "-O3" CACHE STRING "C++ compiler flags")
	SET(MY_C_FLAGS_RELEASE "-O3" CACHE STRING "C compiler flags")

	# removed the following problematic flags: -ftemplate-depth=1024
	IF(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS)
		SET(MY_C_CXX_AGGRESSIVE_FLAGS "-funsafe-loop-optimizations -fprefetch-loop-arrays" CACHE STRING "C/C++ aggressive optimization compiler flags")
		OPTION(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS_FM "Use fast math (not reccomended, only if aggressive optimizations are enabled)." FALSE)
		IF(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS_FM)
			SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffast-math ${MY_CXX_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffast-math ${MY_C_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -ffast-math ${MY_CXX_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -ffast-math ${MY_C_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -ffast-math ${MY_CXX_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -ffast-math ${MY_C_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
		ELSE(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS_FM)
			SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${MY_CXX_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${MY_C_FLAGS_RELEASE} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${MY_CXX_FLAGS_RELWITHDEBINFO} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${MY_C_FLAGS_RELWITHDEBINFO} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} ${MY_CXX_FLAGS_MINSIZEREL} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
			SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} ${MY_C_FLAGS_MINSIZEREL} ${MY_C_CXX_AGGRESSIVE_FLAGS}")
		ENDIF(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS_FM)
	ELSE(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS)
		SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${MY_CXX_FLAGS_RELEASE}")
		SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${MY_C_FLAGS_RELEASE}")
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${MY_CXX_FLAGS_RELEASE}")
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${MY_C_FLAGS_RELEASE}")
		SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} ${MY_CXX_FLAGS_RELEASE}")
		SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} ${MY_C_FLAGS_RELEASE}")
	ENDIF(SLAM_P_P_AGGRESSIVE_OPTIMIZATIONS)

	IF(APPLE)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-string-plus-int -Wno-comment -Wno-null-dereference")
		# disable OS X silly warnings (-Wno-string-plus-int = string + int does not concatenate,
		# -Wno-comment = a comment inside block comment, -Wno-null-dereference - casting and dereferencing
		# null pointer; used in one place to return something, after an exception is thrown - hence in unreachable
		# code, still, need to resolve it differently)
	ENDIF(APPLE)

	SET(MY_EXE_LINKER_FLAGS " " CACHE STRING "Linker flags")
	SET(CMAKE_EXE_LINKER_FLAGS "${MY_EXE_LINKER_FLAGS}")
ENDIF(UNIX)

# apply some OpenMP definitions, make eigen not parallel
OPTION(SLAM_P_P_USE_OPENMP "Use OpenMP for parallelization." TRUE)
IF(SLAM_P_P_USE_OPENMP)
	FIND_PACKAGE(OpenMP)
	IF(OPENMP_FOUND)
		ADD_DEFINITIONS(-DEIGEN_DONT_PARALLELIZE)
		SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
		SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
		SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} ${OpenMP_C_FLAGS}")
		SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} ${OpenMP_CXX_FLAGS}")
		SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${OpenMP_C_FLAGS}")
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${OpenMP_CXX_FLAGS}")

		#SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${OpenMP_C_FLAGS}")
		#SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS}")
		# no OpenMP in debug, difficult to debug with that

		MESSAGE(STATUS "Compiling with OpenMP support")
		# no linker flags set, the compiller will somehow pass it onto the linker automatically
	ELSE(OPENMP_FOUND)
		IF(APPLE)
			MESSAGE(STATUS "Compiling without OpenMP support (disabling OpenMP)")
			SET(SLAM_P_P_USE_OPENMP FALSE CACHE BOOL "Use OpenMP for parralelization." FORCE)

			mark_as_advanced(FORCE OpenMP_C_FLAGS)
			mark_as_advanced(FORCE OpenMP_CXX_FLAGS)
			# dont want to confuse the user with that
		ENDIF(APPLE)
	ENDIF(OPENMP_FOUND)
ENDIF(SLAM_P_P_USE_OPENMP)

# switch to Eigen 3.3
OPTION(SLAM_P_P_EIGEN33 "Use Eigen 3.3 with the AVX instruction set support (not always faster)." FALSE)
IF(SLAM_P_P_EIGEN33)
	OPTION(EIGEN_UNALIGNED_VECTORIZE "Vectorize also unaligned vectors and matrices (Eigen 3.3 only)." TRUE)
	IF(EIGEN_UNALIGNED_VECTORIZE)
		ADD_DEFINITIONS(-DEIGEN_UNALIGNED_VECTORIZE=1)
	ELSE(EIGEN_UNALIGNED_VECTORIZE)
		ADD_DEFINITIONS(-DEIGEN_UNALIGNED_VECTORIZE=0)
	ENDIF(EIGEN_UNALIGNED_VECTORIZE)

	mark_as_advanced(CLEAR EIGEN_UNALIGNED_VECTORIZE) # unhide
ELSE(SLAM_P_P_EIGEN33)
	mark_as_advanced(FORCE EIGEN_UNALIGNED_VECTORIZE) # hide from the user but do not delete from cache
ENDIF(SLAM_P_P_EIGEN33)

# compile for native architecture, might run faster (makes a potentially less portable binary)
IF(UNIX AND NOT APPLE)
	OPTION(SLAM_P_P_USE_MARCH_NATIVE "Use native architecture (-march=native)." TRUE)
	IF(SLAM_P_P_USE_MARCH_NATIVE)
#                SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native -std=c++11")
#                SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -march=native -std=c++11")
#                SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -std=c++11")
#                SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native -std=c++11")
#                SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -march=native -std=c++11")
#                SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -march=native -std=c++11")
#                SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -std=c++11")
#                SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -march=native -std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -march=native")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -march=native")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -march=native")
SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -march=native")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")
SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -march=native")
	ENDIF(SLAM_P_P_USE_MARCH_NATIVE)
ELSE(UNIX AND NOT APPLE)
	SET(SLAM_P_P_USE_MARCH_NATIVE FALSE)
ENDIF(UNIX AND NOT APPLE)

IF("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
    OPTION(SLAM_P_P_USE_NEON "Use NEON instruction set (-mfpu=neon)." TRUE)
	# show the NEON option in the gui
ELSE("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
	OPTION(SLAM_P_P_USE_NEON "Use NEON instruction set when cross-compiling (-mfpu=neon)." FALSE)
	mark_as_advanced(FORCE SLAM_P_P_USE_NEON) # hide from the user but do not delete from cache
	# even if the host CPU is not ARM, we can still potentially cross-compile for it. just hide it away.
ENDIF("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")

IF(SLAM_P_P_USE_NEON)
	SET(SLAM_P_P_ARM_FLOAT_ABI 0 CACHE STRING "Floating-point ABI (0 = not set, 1 = soft, 2 = softfp, 3 = hard).")
	IF(UNIX)
#		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
#		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} 	-mfpu=neon")
#		IF(SLAM_P_P_ARM_FLOAT_ABI EQUAL 1)
#			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=soft")
#			SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=soft")
#		ELSEIF(SLAM_P_P_ARM_FLOAT_ABI EQUAL 2)
#			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=softfp")
#			SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=softfp")
#		ELSEIF(SLAM_P_P_ARM_FLOAT_ABI EQUAL 3)
#			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfloat-abi=hard")
#			SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard")
#		ENDIF(SLAM_P_P_ARM_FLOAT_ABI EQUAL 1)
	    add_definitions("-DENABLE_SSE")
	    MESSAGE("SSE flags: " ${SSE_FLAGS})
	    # Optimized for Jetson TX2 only; refer to https://devtalk.nvidia.com/default/topic/1028179/jetson-tx2/gcc-options-for-tx2
	    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto ") 
	    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto") 
	ELSE(UNIX)
		ADD_DEFINITIONS(-D__ARM_NEON)
	ENDIF(UNIX)
ENDIF(SLAM_P_P_USE_NEON)

# enable or disable explicit instruction sets
# this block adds the following options: SLAM_P_P_USE_FMA, SLAM_P_P_USE_AVX512, SLAM_P_P_USE_AVX2, SLAM_P_P_USE_AVX, SLAM_P_P_USE_SSE4, SLAM_P_P_USE_SSE3
IF(NOT SLAM_P_P_USE_MARCH_NATIVE) # explicit instruction sets only make sense if not using -march=natice
	SET(MSVC_ARCH_AVX_SET FALSE) # in MSVC, AVX rules out older instruction sets

	IF(SLAM_P_P_EIGEN33)
		OPTION(SLAM_P_P_USE_FMA "Use the FMA instructions (Eigen 3.3 only; may be implied by -march=native)." FALSE)
		mark_as_advanced(CLEAR SLAM_P_P_USE_FMA) # unhide
		IF(SLAM_P_P_USE_FMA)
			IF(UNIX)
				SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mfma")
				SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mfma")
				SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfma")
				SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mfma")
				SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -mfma")
				SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -mfma")
				SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -mfma")
				SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -mfma")
			ELSE(UNIX)
				ADD_DEFINITIONS(-DEIGEN_VECTORIZE_FMA)
			ENDIF(UNIX)
		ENDIF(SLAM_P_P_USE_FMA)

		OPTION(SLAM_P_P_USE_AVX512 "Use the AVX512 instruction set (Eigen 3.3 only; experimental; may be implied by -march=native)." FALSE)
		mark_as_advanced(CLEAR SLAM_P_P_USE_AVX512) # unhide
		IF(SLAM_P_P_USE_AVX512)
			IF(UNIX)
				SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -mavx -mavx2 -mavx512f -mavx512dq")
				SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -mavx -mavx2 -mavx512f -mavx512dq")
			ELSE(UNIX)
				ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX512)
				ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX512DQ)

				IF(MSVC AND (NOT (MSVC_VERSION LESS 1800))) # this was added after VS 2008 = 9.0 = 1500 (not sure when) but definitely is in VS 2013 = 12.0 = 1800
					SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
					SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX2")
					SET(MSVC_ARCH_AVX_SET TRUE)
				ELSE(MSVC AND (NOT (MSVC_VERSION LESS 1800)))
					ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX)
					ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX2)
				ENDIF(MSVC AND (NOT (MSVC_VERSION LESS 1800)))
				# new visual studios can add it like this as well (if not enabled,
				# they ultimately complain upon detecting the AVX intrincics in Eigen)
			ENDIF(UNIX)
			ADD_DEFINITIONS(-DEIGEN_ENABLE_AVX512)

			mark_as_advanced(FORCE SLAM_P_P_USE_AVX2)
			mark_as_advanced(FORCE SLAM_P_P_USE_AVX) # hide from the user but do not delete from cache
		ELSE(SLAM_P_P_USE_AVX512)
			OPTION(SLAM_P_P_USE_AVX2 "Use the AVX2 instruction set (Eigen 3.3 only; may be implied by -march=native)." FALSE)
			mark_as_advanced(CLEAR SLAM_P_P_USE_AVX2) # unhide
			IF(SLAM_P_P_USE_AVX2)
				IF(UNIX)
					SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mavx -mavx2")
					SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mavx -mavx2")
					SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx -mavx2")
					SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mavx -mavx2")
					SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -mavx -mavx2")
					SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -mavx -mavx2")
					SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -mavx -mavx2")
					SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -mavx -mavx2")
				ELSE(UNIX)
					IF(MSVC AND (NOT (MSVC_VERSION LESS 1800))) # this was added after VS 2008 = 9.0 = 1500 (not sure when) but definitely is in VS 2013 = 12.0 = 1800
						SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
						SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX2")
						SET(MSVC_ARCH_AVX_SET TRUE)
					ELSE(MSVC AND (NOT (MSVC_VERSION LESS 1800)))
						ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX)
						ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX2)
					ENDIF(MSVC AND (NOT (MSVC_VERSION LESS 1800)))
					# new visual studios can add it like this as well (if not enabled,
					# they ultimately complain upon detecting the AVX intrincics in Eigen)
				ENDIF(UNIX)

				mark_as_advanced(FORCE SLAM_P_P_USE_AVX) # hide from the user but do not delete from cache
			ELSE(SLAM_P_P_USE_AVX2)
				OPTION(SLAM_P_P_USE_AVX "Use the AVX instruction set (Eigen 3.3 only; may be implied by -march=native)." FALSE)
				mark_as_advanced(CLEAR SLAM_P_P_USE_AVX) # unhide
				IF(SLAM_P_P_USE_AVX)
					IF(UNIX)
						SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -mavx")
						SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -mavx")
						SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mavx")
						SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mavx")
						SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -mavx")
						SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -mavx")
						SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -mavx")
						SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -mavx")
					ELSE(UNIX)
						IF(MSVC AND (NOT (MSVC_VERSION LESS 1700))) # this was added after VS 2008 = 9.0 = 1500 (not sure when) but definitely is in VS 2012 = 11.0 = 1700
							SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX")
							SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:AVX")
							SET(MSVC_ARCH_AVX_SET TRUE)
						ELSE(MSVC AND (NOT (MSVC_VERSION LESS 1700)))
							ADD_DEFINITIONS(-DEIGEN_VECTORIZE_AVX)
						ENDIF(MSVC AND (NOT (MSVC_VERSION LESS 1700)))
						# new visual studios can add it like this as well (if not enabled,
						# they ultimately complain upon detecting the AVX intrincics in Eigen)
					ENDIF(UNIX)
				ENDIF(SLAM_P_P_USE_AVX)
			ENDIF(SLAM_P_P_USE_AVX2)
		ENDIF(SLAM_P_P_USE_AVX512)
	ELSE(SLAM_P_P_EIGEN33)
		mark_as_advanced(FORCE SLAM_P_P_USE_FMA)
		mark_as_advanced(FORCE SLAM_P_P_USE_AVX512)
		mark_as_advanced(FORCE SLAM_P_P_USE_AVX2)
		mark_as_advanced(FORCE SLAM_P_P_USE_AVX) # hide from the user but do not delete from cache
	ENDIF(SLAM_P_P_EIGEN33)

	# support SSE4 (good on Intel platforms)
	OPTION(SLAM_P_P_USE_SSE4 "Use the SSE4 instruction set (may be implied by -march=native)." TRUE)
	mark_as_advanced(CLEAR SLAM_P_P_USE_SSE4) # unhide
	IF(SLAM_P_P_USE_SSE4)
		IF(UNIX)
			SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -msse4")
			SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -msse4")
			SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4")
			SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -msse4")
			SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -msse4")
			SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -msse4")
			SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -msse4")
			SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -msse4")
		ELSE(UNIX)
			IF(NOT MSVC_ARCH_AVX_SET) # /arch:AVX implies also SSE3 and SSE4 in Eigen config
				ADD_DEFINITIONS(-DEIGEN_VECTORIZE_SSE3)
				ADD_DEFINITIONS(-DEIGEN_VECTORIZE_SSE4)
			ENDIF(NOT MSVC_ARCH_AVX_SET)
		ENDIF(UNIX)

		mark_as_advanced(FORCE SLAM_P_P_USE_SSE3) # hide from the user but do not delete from cache
	ELSE(SLAM_P_P_USE_SSE4)
		# support SSE3 (good on Intel platforms)
		OPTION(SLAM_P_P_USE_SSE3 "Use the SSE3 instruction set (may be implied by -march=native)." TRUE)
		mark_as_advanced(CLEAR SLAM_P_P_USE_SSE3) # unhide
		IF(SLAM_P_P_USE_SSE3)
			IF(UNIX)
				SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -msse3")
				SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -msse3")
				SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse3")
				SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -msse3")
				SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -msse3")
				SET(CMAKE_C_FLAGS_MINSIZEREL "${CMAKE_C_FLAGS_MINSIZEREL} -msse3")
				SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -msse3")
				SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} -msse3")
			ELSE(UNIX)
				IF(NOT MSVC_ARCH_AVX_SET) # /arch:AVX implies also SSE3 and SSE4 in Eigen config
					ADD_DEFINITIONS(-DEIGEN_VECTORIZE_SSE3)
				ENDIF(NOT MSVC_ARCH_AVX_SET)
			ENDIF(UNIX)
		ENDIF(SLAM_P_P_USE_SSE3)
	ENDIF(SLAM_P_P_USE_SSE4)
ELSE(NOT SLAM_P_P_USE_MARCH_NATIVE)
	mark_as_advanced(FORCE SLAM_P_P_USE_FMA)
	mark_as_advanced(FORCE SLAM_P_P_USE_AVX512)
	mark_as_advanced(FORCE SLAM_P_P_USE_AVX2)
	mark_as_advanced(FORCE SLAM_P_P_USE_AVX)
	mark_as_advanced(FORCE SLAM_P_P_USE_SSE4)
	mark_as_advanced(FORCE SLAM_P_P_USE_SSE3) # hide from the user but do not delete from cache
ENDIF(NOT SLAM_P_P_USE_MARCH_NATIVE)

# find CUDA
OPTION(GPU_ENABLED "Use GPU acceleration for SLAM++." FALSE)
if(GPU_ENABLED)
	SET(CULA_FIND_REQUIRED TRUE)
	FIND_PACKAGE(CULA)
	# find CULA

	SET(CUDA_FIND_REQUIRED TRUE)
	FIND_PACKAGE(CUDA)
	# find CUDA

	mark_as_advanced(CUDA_BUILD_CUBIN)
	mark_as_advanced(CUDA_BUILD_EMULATION)
	mark_as_advanced(CUDA_CHECK_DEPENDENCIES_DURING_COMPILE)
	mark_as_advanced(CUDA_GENERATE_DEPENDENCIES_DURING_CONFIGURE)
	mark_as_advanced(CUDA_HOST_COMPILER)
	mark_as_advanced(CUDA_SEPARABLE_COMPILATION)
	mark_as_advanced(CUDA_SDK_ROOT_DIR)
	mark_as_advanced(CUDA_VERBOSE_BUILD)
	# average Joe doesn't need to know about those

	SET(CUDA_INCLUDE_IFENABLED ${CUDA_INCLUDE_DIRS} ${CULA_INCLUDE})
	SET(CUDA_LIBRARIES_IFENABLED ${CUDA_CUDA_LIBRARY} ${CUDA_cublas_LIBRARY}
		${CUDA_cusparse_LIBRARY} ${CULA_CULA_LIBRARY} ${CULA_lapack_LIBRARY})
	# requires libcuda.so / cuda.lib. cudart won't do.

	#message("includes: ${CUDA_INCLUDE_IFENABLED}")
	#message("libs: ${CUDA_LIBRARIES_IFENABLED}")
	# debug
ELSE(GPU_ENABLED)
	SET(CUDA_INCLUDE_IFENABLED "")
	SET(CUDA_LIBRARIES_IFENABLED "")
	ADD_DEFINITIONS(-D__DISABLE_GPU)
ENDIF(GPU_ENABLED)

# Show make output print actualy executed commands?
OPTION(SLAM_P_P_VERBOSE_MAKEFILE "Verbose make output." "OFF")
IF(SLAM_P_P_VERBSOSE_MAKEFILE)
	SET(CMAKE_VERBOSE_MAKEFILE "ON")
ENDIF(SLAM_P_P_VERBSOSE_MAKEFILE)

# Linux platform specific flags and settings.
IF(UNIX)
	ADD_DEFINITIONS(-D_UNIX)
	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
		ADD_DEFINITIONS(-D_DEBUG)
		SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -ggdb")
		SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -ggdb")
	ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
		ADD_DEFINITIONS(-DNDEBUG)
		#SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O3")
		#SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3") # nope, does not make any sense; and -O3 is added via a user-settable flag
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	IF(NOT APPLE)
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")
	ENDIF(NOT APPLE)
	# add debug / release flags

	IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
		IF(SLAM_P_P_32BIT_BUILD)
			IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
				SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -m32")
				SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -m32")
			ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
				SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
				SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
			ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
			SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
		ENDIF(SLAM_P_P_32BIT_BUILD)
	ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	# add 32-bit build flags
ELSE(UNIX)
	if(MSVC)
		ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE)
		# get rid of warnings

		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /Zm500")
		# otherwise some builds fail

		SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP")
		SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MP")
		SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MP")
		# enable parallel builds in release
		
		#IF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
		#	ADD_DEFINITIONS(-D_SECURE_SCL=0) # disable secure iterators in release
		#ENDIF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
		# this does not work for MSVC, only works for CMake-like

		#SET_PROPERTY(GLOBAL PROPERTY COMPILE_DEFINITIONS_RELEASE _SECURE_SCL=0)
		# does not work

		SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /D _SECURE_SCL=0")
		SET(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /D _SECURE_SCL=0")
		# (do not disable secure SCL in release with debug)
		# hack but it works fine, even shows in the "Preprocessor" settings tab rather than in "Commandline" tab
	ENDIF(MSVC)
	IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
		IF(NOT SLAM_P_P_32BIT_BUILD)
			SET(CMAKE_CL_64 TRUE)
			# support for x64 builds using visual studio
		ENDIF(NOT SLAM_P_P_32BIT_BUILD)
	ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)
ENDIF(UNIX)

# check environment first 
if (NOT DEFINED ENV{GF_ORB_SLAM2_ROOT})
    message(FATAL_ERROR "env variable: $ENV{GF_ORB_SLAM2_ROOT} is NOT defined, please run build_dep_ubuntu20.04 first")
endif()

# customize armadillo dir
set(ARMADILLO_INCLUDE_DIR $ENV{GF_ORB_SLAM2_ROOT}/armadillo/include)
set(ARMADILLO_LIBRARIES   $ENV{GF_ORB_SLAM2_ROOT}/armadillo/lib/libarmadillo.so)

# customize armadillo dir
# set(ARMADILLO_INCLUDE_DIR /opt/armadillo/include)
# set(ARMADILLO_LIBRARIES /opt/armadillo/lib/libarmadillo.so)
MESSAGE("ARMADILLO libs: " ${ARMADILLO_LIBRARIES})


# Set output path for libraries.
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib CACHE PATH
	"Output directory for compiled libraries.")

# Set output path for binaries.
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin CACHE PATH
	"Output directory for compiled executables.")

# Set program data directory.
SET(SLAM_P_P_DATA_PATH ${CMAKE_INSTALL_PREFIX}/data CACHE PATH
	"Output directory program data.")

# set compiler output directories (not make install, just make)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# if doing 64-bit build, we need CAMD with both DLONG and NLONG to be able to run short CHOLMOD
IF(SLAM_P_P_SHORT_CHOLMOD AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
#        SET(SLAM_P_P_LIBRARIES pthread slampp_base csparse cxsparse camd_dlong cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
#        SET(SLAM_EXAMPLE_LIBRARIES pthread slampp_base csparse cxsparse camd_dlong cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
        SET(SLAM_P_P_LIBRARIES slampp_base csparse cxsparse camd_dlong cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
        SET(SLAM_EXAMPLE_LIBRARIES slampp_base csparse cxsparse camd_dlong cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
ELSE(SLAM_P_P_SHORT_CHOLMOD AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
#        SET(SLAM_P_P_LIBRARIES pthread slampp_base csparse cxsparse cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
#        SET(SLAM_EXAMPLE_LIBRARIES pthread slampp_base csparse cxsparse cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
        SET(SLAM_P_P_LIBRARIES slampp_base csparse cxsparse cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
        SET(SLAM_EXAMPLE_LIBRARIES slampp_base csparse cxsparse cholmod eigen ${CUDA_LIBRARIES_IFENABLED} ${ARMADILLO_LIBRARIES})
ENDIF(SLAM_P_P_SHORT_CHOLMOD AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)

# Include project headers.
IF(SLAM_P_P_EIGEN33)
	INCLUDE_DIRECTORIES(
      ${ARMADILLO_INCLUDE_DIR}
	  ${CUDA_INCLUDE_IFENABLED}
	  ${PROJECT_SOURCE_DIR}/include/
      $ENV{GF_ORB_SLAM2_ROOT}/eigen33/include/eigen3/
    #   /opt/eigen33/include/eigen3/
#	  ${PROJECT_SOURCE_DIR}/include/eigen33/
#	  ${PROJECT_SOURCE_DIR}/external/Ceres/include/
#	  ${PROJECT_SOURCE_DIR}/external/Ceres/internal/
#	  ${PROJECT_SOURCE_DIR}/external/NIST_BLAS/include/
#	  ${PROJECT_SOURCE_DIR}/external/g2o/
	  ${PROJECT_SOURCE_DIR}/include/cholmod/SuiteSparse/)
ELSE(SLAM_P_P_EIGEN33)
	INCLUDE_DIRECTORIES(
      ${ARMADILLO_INCLUDE_DIR}
	  ${CUDA_INCLUDE_IFENABLED}
	  ${PROJECT_SOURCE_DIR}/include/
      /usr/include/eigen3/
#	  ${PROJECT_SOURCE_DIR}/include/eigen32/
#	  ${PROJECT_SOURCE_DIR}/external/Ceres/include/
#	  ${PROJECT_SOURCE_DIR}/external/Ceres/internal/
#	  ${PROJECT_SOURCE_DIR}/external/NIST_BLAS/include/
#	  ${PROJECT_SOURCE_DIR}/external/g2o/
	  ${PROJECT_SOURCE_DIR}/include/cholmod/SuiteSparse/)
ENDIF(SLAM_P_P_EIGEN33)

# Path to supplied dependency libraries.
IF(UNIX)
	IF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
		SET(SLAM_P_P_SUPPLIED_LIBS_PATH ${PROJECT_SOURCE_DIR}/lib/linux64
			CACHE STRING "Path to supplied dependency libraries.")
	ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
		SET(SLAM_P_P_SUPPLIED_LIBS_PATH ${PROJECT_SOURCE_DIR}/lib/linux32
			CACHE STRING "Path to supplied dependency libraries.")
	ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD)
ENDIF(UNIX)

OPTION(SLAM_P_P_HAVE_IGRAPH "Use igraph for Schur ordering." FALSE)
IF(SLAM_P_P_HAVE_IGRAPH)
	ADD_DEFINITIONS(-DHAVE_IGRAPH)
	ADD_SUBDIRECTORY(external/igraph) # need this for testing
	INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/external/igraph/include/)
	SET(SLAM_P_P_LIBRARIES ${SLAM_P_P_LIBRARIES} igraph)
	SET(SLAM_EXAMPLE_LIBRARIES ${SLAM_P_P_LIBRARIES} igraph)
ENDIF(SLAM_P_P_HAVE_IGRAPH)

ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)

# set default startup project to the SLAM++ console app (CMake 3.6 and onward, probably no effect in earlier versions)
IF(MSVC)
	IF(SLAM_P_P_BUILD_MAIN_APP)
		IF("${CMAKE_VERSION}" VERSION_GREATER 3.5.99)
			SET_PROPERTY(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT slam_plus_plus)
		ELSE("${CMAKE_VERSION}" VERSION_GREATER 3.5.99)
			SET(CMAKE_DEFAULT_STARTUP_PROJECT slam_plus_plus) # does not seem to work except 3.5
		ENDIF("${CMAKE_VERSION}" VERSION_GREATER 3.5.99)
	ENDIF(SLAM_P_P_BUILD_MAIN_APP)
ENDIF(MSVC)

# debug - see the properties that SLAM++ is compiled with
#get_directory_property( DirDefs COMPILE_DEFINITIONS )
#message( "COMPILE_DEFINITIONS = ${DirDefs}" )
#IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
#	message( "CMAKE_CXX_FLAGS_DEBUG = ${CMAKE_CXX_FLAGS_DEBUG}" )
#	message( "CMAKE_C_FLAGS_DEBUG = ${CMAKE_C_FLAGS_DEBUG}" )
#ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
#	message( "CMAKE_CXX_FLAGS_RELEASE = ${CMAKE_CXX_FLAGS_RELEASE}" )
#	message( "CMAKE_C_FLAGS_RELEASE = ${CMAKE_C_FLAGS_RELEASE}" )
#ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
#message( "CMAKE_EXE_LINKER_FLAGS = ${CMAKE_EXE_LINKER_FLAGS}" )
