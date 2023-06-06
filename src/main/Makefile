export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
export DESPOT = /home/bhuvanesh/CLionProjects/despot/include/despot
export MY_JNI_HEADERS=${OUT_DIR}/jniHeaders
export PROJECT_PACKAGE=${PWD}/java/com/jni/example
export AJAN_PACKAGE=${PWD}/java/de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP
export CPP_FOLDER=${PWD}/cpp

export OUT_DIR=${PWD}/out
TEMPERATURE_JAVA_FILES := $(wildcard $(PROJECT_PACKAGE)/Temperature*.java)
AJAN_POMDP_FILES := $(wildcard ${AJAN_PACKAGE}/*.java)
AJAN_POMDP_IMPL_FILES := $(wildcard ${AJAN_PACKAGE}/implementation/*.java)
DESPOT_SRC := ${DESPOT}
DESPOT_CORE:= ${DESPOT}/core
DESPOT_INTERFACE:= ${DESPOT}/interface
DESPOT_POMDPX:= ${DESPOT}/pomdpx
DESPOT_SOLVER:= ${DESPOT}/solver
DESPOT_UTIL:= ${DESPOT}/util
DESPOT_UTIL_TINYXML:= ${DESPOT}/util/tinyxml
DESPOT := ${CPP_FOLDER}/despot/build
AJAN_Agent := ${CPP_FOLDER}/AJAN_Agent/src

#temperaturesampler:
#	mkdir -p ${OUT_DIR} ${MY_JNI_HEADERS}
#	javac -h ${MY_JNI_HEADERS} -d ${OUT_DIR} ${TEMPERATURE_JAVA_FILES} ${AJAN_POMDP_FILES} ${AJAN_POMDP_IMPL_FILES}
#	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
#	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TemperatureSampler.cpp -o ${OUT_DIR}/temperatureSampler.o
#	#g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${DESPOT_SRC} -I${DESPOT_CORE} -I${DESPOT_INTERFACE} -I${DESPOT_POMDPX} -I${DESPOT_SOLVER} -I${DESPOT_UTIL} -I${DESPOT_UTIL_TINYXML} -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
#	#g++ -shared -fPIC -o ${OUT_DIR}/libtemperaturesampler.so ${OUT_DIR}/temperatureSampler.o ${OUT_DIR}/tigerState.o -lc
#	g++ -shared -fPIC  ${DESPOT}/*.o ${OUT_DIR}/tigerState.o  ${OUT_DIR}/temperatureSampler.o -o ${OUT_DIR}/libtemperaturesampler.so -lc

AJANPlanner:
	mkdir -p ${OUT_DIR} ${MY_JNI_HEADERS} ${AJAN}
	javac -h ${MY_JNI_HEADERS} -d ${OUT_DIR} ${TEMPERATURE_JAVA_FILES} ${AJAN_POMDP_FILES} ${AJAN_POMDP_IMPL_FILES}
	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/ajanAgent.cpp -o ${OUT_DIR}/ajanAgent.o
	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/ajanPlanner.cpp -o ${OUT_DIR}/ajanPlanner.o
	#g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${DESPOT_SRC} -I${DESPOT_CORE} -I${DESPOT_INTERFACE} -I${DESPOT_POMDPX} -I${DESPOT_SOLVER} -I${DESPOT_UTIL} -I${DESPOT_UTIL_TINYXML} -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
	#g++ -shared -fPIC -o ${OUT_DIR}/libtemperaturesampler.so ${OUT_DIR}/temperatureSampler.o ${OUT_DIR}/tigerState.o -lc
	g++ -shared -fPIC  ${DESPOT}/*.o ${OUT_DIR}/ajanAgent.o  ${OUT_DIR}/ajanPlanner.o -o ${OUT_DIR}/libajanplanner.so -lc

run_temperaturesampler: temperaturesampler1
	java -cp ${OUT_DIR} -Djava.library.path=${OUT_DIR} com.jni.example.TemperatureSampler

run_planner: AJANPlanner
	#java -cp ${OUT_DIR} -Djava.library.path=${OUT_DIR} com.ajan.POMDP.AJANPlanner
	java -cp ${OUT_DIR} -Djava.library.path=${OUT_DIR} de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.AJANPlanner

clean:
	rm -rf ${OUT_DIR} ${MY_JNI_HEADERS}